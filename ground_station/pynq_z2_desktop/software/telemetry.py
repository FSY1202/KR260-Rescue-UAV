"""Read-only LAN telemetry protocol and local snapshots. No vehicle commands."""
from collections import deque
import hashlib
import hmac
import json
import math
from pathlib import Path
import re
import time

from flight_demo import FlightState

VERSION = 1
MAX_PACKET = 4096
DEFAULT_STATE = '/run/pynq-telemetry/state.json'
IDENTIFIER = re.compile(r'^[A-Za-z0-9_-]{1,32}$')


def number(value, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('Telemetry number outside supported range')
    return value


def exact_keys(value, keys):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError('Telemetry fields do not match protocol')


def canonical(packet):
    return json.dumps(packet, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')


def encode_packet(packet, key=None):
    packet = dict(packet)
    if key:
        packet['mac'] = hmac.new(key, canonical(packet), hashlib.sha256).hexdigest()
    return canonical(packet)


def decode_packet(raw, key=None):
    if not 0 < len(raw) <= MAX_PACKET:
        raise ValueError('Telemetry packet too large or empty')
    def unique(pairs):
        result = {}
        for name, value in pairs:
            if name in result:
                raise ValueError('Duplicate JSON field')
            result[name] = value
        return result
    packet = json.loads(raw.decode('utf-8'), object_pairs_hook=unique,
                        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
    if not isinstance(packet, dict):
        raise ValueError('Expected telemetry object')
    if key:
        signature = packet.pop('mac', None)
        expected = hmac.new(key, canonical(packet), hashlib.sha256).hexdigest()
        if not isinstance(signature, str) or not hmac.compare_digest(signature, expected):
            raise ValueError('Telemetry authentication failed')
    exact_keys(packet, ['version','data_origin','vehicle_id','session_id','seq','position','attitude',
                        'flight','battery','sensors']+(['planned_path'] if 'planned_path' in packet else []))
    if type(packet['version']) is not int or packet['version'] != VERSION:
        raise ValueError('Unsupported telemetry version')
    if packet['data_origin'] not in ['vehicle','test']:
        raise ValueError('Invalid data origin')
    for field in ('vehicle_id','session_id'):
        if not isinstance(packet[field], str) or not IDENTIFIER.fullmatch(packet[field]):
            raise ValueError('Invalid telemetry identity')
    if type(packet['seq']) is not int or not 0 <= packet['seq'] < 2**53:
        raise ValueError('Invalid telemetry sequence')
    position, attitude, flight, battery, sensors = (packet[k] for k in ['position','attitude','flight','battery','sensors'])
    exact_keys(position, ['east_m','north_m','valid'])
    number(position['east_m'], -100000, 100000)
    number(position['north_m'], -100000, 100000)
    if type(position['valid']) is not bool:
        raise ValueError('Invalid position flag')
    exact_keys(attitude, ['roll_deg','pitch_deg','heading_deg'])
    number(attitude['roll_deg'], -180, 180)
    number(attitude['pitch_deg'], -90, 90)
    number(attitude['heading_deg'], 0, 359.999999999)
    exact_keys(flight, ['mode','armed','phase','relative_altitude_m','ground_speed_mps','climb_rate_mps'])
    if not isinstance(flight['mode'], str) or not re.fullmatch(r'[A-Z0-9_-]{1,16}',flight['mode']):
        raise ValueError('Invalid flight mode')
    if type(flight['armed']) is not bool or flight['phase'] not in ['standby','takeoff','cruise','hover','landing']:
        raise ValueError('Invalid flight state')
    number(flight['relative_altitude_m'], -1000, 10000)
    number(flight['ground_speed_mps'], 0, 200)
    number(flight['climb_rate_mps'], -100, 100)
    exact_keys(battery, ['voltage_v','current_a','percent'])
    number(battery['voltage_v'], 0, 100)
    number(battery['current_a'], -100, 1000)
    number(battery['percent'], 0, 100)
    exact_keys(sensors, ['imu','barometer','compass','gnss'])
    if any(type(v) is not bool for v in sensors.values()):
        raise ValueError('Invalid sensor flag')
    if 'planned_path' in packet:
        path=packet['planned_path']
        exact_keys(path,['id','valid','points'])
        if not isinstance(path['id'],str) or not IDENTIFIER.fullmatch(path['id']):
            raise ValueError('Invalid planned path identity')
        if type(path['valid']) is not bool or not isinstance(path['points'],list) or len(path['points'])>64:
            raise ValueError('Invalid planned path')
        if (path['valid'] and len(path['points'])<2) or (not path['valid'] and path['points']):
            raise ValueError('Invalid planned path availability')
        for p in path['points']:
            if not isinstance(p,list) or len(p)!=2:raise ValueError('Invalid planned XY point')
            for value in p:number(value,-100000,100000)
    return packet


class TelemetryStore:
    """Bounded trail; ordering and liveness use receiver time, not sender clocks."""
    def __init__(self, vehicle_id='uav1', timeout=2., trail_limit=1200, restart_grace=3.):
        if not IDENTIFIER.fullmatch(vehicle_id):
            raise ValueError('Invalid configured vehicle ID')
        number(timeout, .1, 60)
        number(restart_grace, timeout, 120)
        if type(trail_limit) is not int or not 2 <= trail_limit <= 10000:
            raise ValueError('Invalid trail limit')
        self.vehicle_id, self.timeout, self.restart_grace = vehicle_id, timeout, restart_grace
        self.packet = None
        self.received = None
        self.trail = deque(maxlen=trail_limit)
        self.accepted = self.rejected = self.missing = 0

    def accept(self, packet, now):
        if packet['vehicle_id'] != self.vehicle_id:
            self.rejected += 1
            return False
        if self.packet:
            if packet['session_id'] == self.packet['session_id']:
                if packet['seq'] <= self.packet['seq']:
                    self.rejected += 1
                    return False
                self.missing += packet['seq'] - self.packet['seq'] - 1
            elif now - self.received < self.restart_grace:
                self.rejected += 1
                return False
            else:
                self.trail.clear()
            if now - self.received > self.timeout and self.trail and self.trail[-1] is not None:
                self.trail.append(None)
        self.packet, self.received = packet, now
        p = packet['position']
        if p['valid']:
            point = [p['east_m'],p['north_m']]
            # Break the line after lost/invalid position, not across an outage.
            if self.trail and (self.trail[-1] is None or self.trail[-1] != point):
                self.trail.append(point)
            elif not self.trail:
                self.trail.append(point)
        elif self.trail and self.trail[-1] is not None:
            self.trail.append(None)
        self.accepted += 1
        return True

    def snapshot(self, now):
        age = None if self.received is None else max(0., now-self.received)
        return dict(format='PYNQ_TELEMETRY_1', timeout_s=self.timeout,
                    received_monotonic=self.received, published_monotonic=now,
                    connected=age is not None and age <= self.timeout,
                    packet=self.packet, trail=list(self.trail),
                    accepted=self.accepted,rejected=self.rejected,sequence_gaps=self.missing)


class TelemetryReader:
    def __init__(self, filename=DEFAULT_STATE):
        self.filename = Path(filename)
        self.last_read = -math.inf
        self.cached = None

    def read(self, now=None):
        now = time.monotonic() if now is None else now
        if now - self.last_read >= .05:
            self.last_read = now
            try:
                if self.filename.stat().st_size > 1024*1024:
                    raise ValueError('Snapshot too large')
                value = json.loads(self.filename.read_text(encoding='utf-8'))
                if value['format'] != 'PYNQ_TELEMETRY_1':
                    raise ValueError('Snapshot version')
                number(value['timeout_s'],.1,60)
                if value['packet'] is not None:
                    decode_packet(canonical(value['packet']))
                    number(value['received_monotonic'],0,1e15)
                if not isinstance(value['trail'],list) or len(value['trail'])>10000:
                    raise ValueError('Invalid trail')
                for p in value['trail']:
                    if p is not None:
                        if not isinstance(p,list) or len(p)!=2:
                            raise ValueError('Invalid trail point')
                        for n in p:number(n,-100000,100000)
                self.cached = value
            except (OSError, ValueError, KeyError, TypeError, RecursionError):
                self.cached = None
        value = self.cached
        connected = False
        age = None
        if value and value['packet'] is not None:
            age = now-value['received_monotonic']
            connected = 0 <= age <= value['timeout_s']
        return value, connected, age


def flight_state(packet):
    a,f,b = (packet[k] for k in ('attitude','flight','battery'))
    return FlightState('udp_test' if packet['data_origin']=='test' else 'udp_telemetry',f['phase'],f['mode'],f['armed'],
        a['roll_deg'],a['pitch_deg'],a['heading_deg'],f['relative_altitude_m'],
        f['ground_speed_mps'],f['climb_rate_mps'],b['voltage_v'],b['current_a'],b['percent'],
        'valid' if packet['position']['valid'] else 'invalid',
        'connected' if packet['sensors']['gnss'] else 'not_connected')


def atomic_write(filename, value):
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)
    temporary = filename.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, separators=(',', ':'), allow_nan=False), encoding='utf-8')
    temporary.replace(filename)
