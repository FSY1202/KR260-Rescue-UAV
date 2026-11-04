"""Deterministic read-only demo values. No flight controller or transport code."""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class FlightState:
    source: str
    phase: str
    mode: str
    armed: bool
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    relative_altitude_m: float
    ground_speed_mps: float
    climb_rate_mps: float
    voltage_v: float
    current_a: float
    battery_percent: float
    local_position: str = 'demo_ok'
    gnss: str = 'not_connected'

    def to_dict(self):
        return asdict(self)


DEFAULTS = dict(battery_cells=4, battery_start_percent=88., battery_end_percent=82.,
                minimum_altitude_m=1., maximum_altitude_m=1.1,
                minimum_speed_mps=.20, maximum_speed_mps=.25, fixed_voltage_v=16.8,
                minimum_current_a=2.8, maximum_current_a=3.,
                maximum_roll_deg=1., maximum_pitch_deg=.6)


def smoothstep(u):
    u = min(1., max(0., u))
    return u*u*(3.-2.*u)


class DemoFlight:
    period = 60.

    def __init__(self, config=None):
        self.config = dict(DEFAULTS)
        if config is not None:
            if not isinstance(config, dict) or set(config)-set(DEFAULTS):
                raise ValueError('Unknown flight demo configuration')
            self.config.update(config)
        c = self.config
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in c.values()):
            raise ValueError('Demo values must be finite numbers')
        if not (int(c['battery_cells']) == c['battery_cells'] and 1 <= c['battery_cells'] <= 12
                and 0 <= c['battery_end_percent'] <= c['battery_start_percent'] <= 100
                and 0 <= c['minimum_altitude_m'] <= c['maximum_altitude_m'] <= 100
                and 0 <= c['minimum_speed_mps'] <= c['maximum_speed_mps'] <= 20
                and 0 < c['fixed_voltage_v'] <= 100
                and 0 <= c['minimum_current_a'] <= c['maximum_current_a'] <= 100
                and 0 <= c['maximum_roll_deg'] <= 45
                and 0 <= c['maximum_pitch_deg'] <= 30):
            raise ValueError('Demo configuration outside supported range')

    @classmethod
    def from_file(cls, filename):
        return cls(json.loads(Path(filename).read_text()))

    def sample(self, elapsed):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Demo time must be finite and nonnegative')
        t = elapsed % self.period
        c = self.config
        altitude = speed = climb = roll = pitch = 0.
        heading = 350.
        armed = False
        mode, phase = 'POSCTL', 'standby'
        if 5 <= t < 10:
            phase, mode, armed = 'takeoff', 'TAKEOFF', True
            u = (t-5)/5
        elif 10 <= t < 45:
            phase, mode, armed = 'cruise', 'OFFBOARD', True
            u = (t-10)/35
            envelope = math.sin(math.pi*u)**2
            roll = c['maximum_roll_deg']*math.sin(2*math.pi*u)*envelope
            pitch = c['maximum_pitch_deg']*math.sin(4*math.pi*u)*envelope
            heading = (350+360*smoothstep(u)) % 360
        elif 45 <= t < 50:
            phase, mode, armed = 'hover', 'HOLD', True
        elif 50 <= t < 55:
            phase, mode, armed = 'landing', 'LAND', True
            u = (t-50)/5
        remaining = c['battery_start_percent']+(c['battery_end_percent']-c['battery_start_percent'])*t/60
        # Requested presentation ranges apply throughout the independent cycle.
        blend = .5*(1-math.cos(2*math.pi*t/self.period))
        altitude = c['minimum_altitude_m']+(c['maximum_altitude_m']-c['minimum_altitude_m'])*blend
        climb = (c['maximum_altitude_m']-c['minimum_altitude_m'])*math.pi/self.period*math.sin(2*math.pi*t/self.period)
        speed = c['minimum_speed_mps']+(c['maximum_speed_mps']-c['minimum_speed_mps'])*blend
        voltage = c['fixed_voltage_v']
        current = c['minimum_current_a']+(c['maximum_current_a']-c['minimum_current_a'])*blend
        return FlightState('preset_demo', phase, mode, armed, roll, pitch, heading,
                           altitude, speed, climb, voltage, current, remaining)


class DemoClock:
    def __init__(self):
        self.started = None

    def start(self, now):
        if self.started is None:
            self.started = now

    def elapsed(self, now):
        return 0. if self.started is None else max(0., now-self.started)
