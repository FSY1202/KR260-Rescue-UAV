#!/usr/bin/env python3
"""Manual KR260 localization. No arming, mode, setpoint or parameter writes.

Startup failure cleans up owned children. After READY, health problems only
report DEGRADED: no automatic restart and no bench-watchdog process teardown.
The bridge itself still refuses old data. This is NOT an airborne failsafe.
ROS imports are delayed so control commands and pure tests work without ROS.
"""
import argparse
from collections import deque
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import uuid

UNIT = 'kr260-position.service'
WORKSPACE = Path('/home/ubuntu/drone_ws')
RUNTIME_DIR = Path('/run/kr260-position')
WINDOW = 5.0
MAX_AGE_MS = 200.0
MAX_GAP = .35
STATES = {'STARTING', 'READY', 'DEGRADED', 'FAILED', 'STOPPED'}
NODE_NAMES = {'livox_ros_driver2_node', 'fastlio_mapping', 'bridge_node', 'MicroXRCEAgent'}


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.position-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def validate_configs(directory):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    expected = {'MID360_kr260_sdk2.json', 'mid360_kr260.yaml', 'bridge.yaml'}
    if set(manifest['files']) != expected:
        raise RuntimeError('configuration manifest has unexpected or missing files')
    for name, expected_hash in manifest['files'].items():
        if hashlib.sha256((directory/name).read_bytes()).hexdigest() != expected_hash:
            raise RuntimeError(f'configuration checksum mismatch: {name}; review/update manifest explicitly')
    return manifest['files']


def external_nodes(proc=Path('/proc')):
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry/'cmdline').read_bytes().split(b'\0')[0].decode()
            if Path(command).name in NODE_NAMES:
                found.append((int(entry.name), Path(command).name))
        except (OSError, UnicodeError):
            continue
    return found


class Stream:
    """Only bounded 5s pose histories; PX4 keeps counters/liveness, not a queue."""
    def __init__(self, timed=True):
        self.timed = timed
        self.events, self.ages = deque(), deque()
        self.last_stamp = self.last_good = None
        self.count = self.rejected = self.backwards = 0

    def record(self, now, stamp, age_ms=0.0, valid=True):
        self.count += 1
        self.trim(now)
        if not valid or stamp <= 0 or not math.isfinite(age_ms) or age_ms < 0:
            self.rejected += 1
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.backwards += int(stamp < self.last_stamp)
            self.rejected += 1
            return False
        self.last_stamp = stamp
        if self.timed:
            self.ages.append((now, age_ms))
            if age_ms > MAX_AGE_MS:
                self.rejected += 1
                return False
            self.events.append((now, age_ms))
        self.last_good = now
        return True

    def trim(self, now):
        for queue in (self.events, self.ages):
            while queue and queue[0][0] < now-WINDOW:
                queue.popleft()

    def stats(self, now, start=None):
        self.trim(now)
        start = now-WINDOW if start is None else start
        events = [(t, a) for t, a in self.events if start <= t <= now]
        ages = sorted(a for t, a in self.ages if start <= t <= now)
        gaps = ([events[0][0]-start, now-events[-1][0]] if events else [now-start])
        gaps.extend(b[0]-a[0] for a, b in zip(events, events[1:]))
        span = events[-1][0]-events[0][0] if len(events) >= 2 else 0
        result = dict(count=self.count, rejected=self.rejected, backwards=self.backwards,
                      last_good_age_s=None if self.last_good is None else now-self.last_good)
        if self.timed:
            result.update(window_count=len(events), hz=(len(events)-1)/span if span > 0 else 0.0,
                          p95_ms=ages[math.ceil(.95*len(ages))-1] if ages else None,
                          max_gap_s=max(gaps))
        return result


def healthy(stats):
    return (stats['hz']+.01 >= 8 and stats['p95_ms'] is not None
            and 0 <= stats['p95_ms'] <= MAX_AGE_MS and stats['max_gap_s'] <= MAX_GAP
            and stats['last_good_age_s'] is not None and stats['last_good_age_s'] <= MAX_GAP)


def external_readers(infos, own_name='kr260_position_monitor'):
    # These are companion-side observers, NOT proof of a PX4 DDS reader.
    excluded = {own_name, 'kr260_mission_controller', 'kr260_mission_recorder',
                'kr260_ego_controller', 'kr260_ego_recorder'}
    return sum(info.node_name not in excluded for info in infos)


class Runtime:
    def __init__(self, args):
        self.args = args
        self.children, self.log_handles = {}, []
        self.streams = {n: Stream(n != 'px4') for n in ('odom', 'vo', 'px4')}
        self.imu = deque()
        self.node = self.imu_sub = None
        self.stop_requested = False
        self.run_id = os.environ.get('INVOCATION_ID') or uuid.uuid4().hex
        self.record = dict(state='STARTING', reason='initializing', run_id=self.run_id,
                           supervisor_pid=os.getpid(), flight_clearance=False)
        self.log_dir = args.workspace/'logs'/('position_'+time.strftime('%Y%m%d_%H%M%S')+'_'+self.run_id[:8])
        self.bridge_cursor = self.bridge_errors = 0
        self.last_bridge_error = None
        self.ready_at = None
        self.health_good_since = None
        self.last_clock = None

    def publish(self, state=None, reason=None, **extra):
        if state:
            assert state in STATES
            self.record['state'] = state
        if reason is not None:
            self.record['reason'] = reason
        self.record.update(extra, updated_monotonic=time.monotonic(), wall_time=time.time(),
                           log_dir=str(self.log_dir),
                           children={n: dict(pid=p.pid, exit_code=p.poll()) for n, p in self.children.items()})
        atomic_json(self.args.runtime_dir/'state.json', self.record)
        print(json.dumps(self.record, ensure_ascii=False, allow_nan=False), flush=True)

    def stage(self, name):
        self.publish('STARTING', name, stage=name)

    def start(self, name, command, attempt=1):
        if name in self.children:
            raise RuntimeError(f'{name} already owned')
        handle = open(self.log_dir/f'{name}_{attempt}.log', 'wb', buffering=0)
        self.log_handles.append(handle)
        self.children[name] = subprocess.Popen(command, cwd=self.args.workspace,
            stdin=subprocess.DEVNULL, stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.publish(reason=f'started {name} attempt {attempt}')

    def stop(self, name):
        process = self.children.pop(name, None)
        if process is None:
            return
        # These are direct node binaries, not forking launch wrappers. Once
        # exited/reaped there is no live owned node to signal; never target a
        # recycled process-group number later in a long-running session.
        if process.poll() is not None:
            return
        # Only process groups created by this instance are eligible for signals.
        for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break

    def ensure_startup(self):
        if self.stop_requested:
            raise InterruptedError('stop requested')
        for name, process in self.children.items():
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited during startup ({process.returncode})')

    def spin(self, duration, startup=True):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            if self.stop_requested:
                raise InterruptedError('stop requested')
            if startup:
                self.ensure_startup()
            try:
                self.rclpy.spin_once(self.node, timeout_sec=min(.05, max(0, end-time.monotonic())))
            except Exception:
                # Preserve teammate's board-side graceful SIGINT handling.
                if self.stop_requested:
                    raise InterruptedError('stop requested') from None
                raise

    def wait_first(self, name, timeout):
        since = time.monotonic()
        while time.monotonic()-since < timeout:
            self.spin(.05)
            received = self.imu[-1] if name == 'imu' and self.imu else (
                self.streams[name].last_good if name != 'imu' else None)
            if received is not None and received >= since:
                return
        raise RuntimeError(f'no fresh {name} within {timeout}s')

    def window(self, name, seconds=5):
        start = time.monotonic()
        self.spin(seconds)
        return self.streams[name].stats(time.monotonic(), start)

    def imu_window(self):
        start = time.monotonic()
        self.spin(3)
        now = time.monotonic()
        values = [t for t in self.imu if start <= t <= now]
        span = values[-1]-values[0] if len(values) >= 2 else 0
        gaps = [now-start] if not values else [values[0]-start, now-values[-1]]
        gaps.extend(b-a for a, b in zip(values, values[1:]))
        hz = (len(values)-1)/span if span > 0 else 0
        if hz < 100 or max(gaps) > .25:
            raise RuntimeError(f'IMU unstable: hz={hz:.3f}, max_gap={max(gaps):.3f}')

    def release_imu(self):
        if self.imu_sub is not None:
            self.node.destroy_subscription(self.imu_sub)
            self.imu_sub = None
            self.imu.clear()

    def start_lio(self, command):
        for attempt in (1, 2):
            self.streams['odom'] = Stream()
            self.stage(f'FAST-LIO attempt {attempt}/2')
            self.start('fastlio', command, attempt)
            try:
                self.wait_first('odom', 15)
                windows = [self.window('odom') for _ in range(2)]
                self.publish(lio_attempt=attempt, startup_windows=windows)
                if all(healthy(window) for window in windows):
                    return
            except RuntimeError as error:
                self.publish(reason=str(error))
            self.stop('fastlio')
        raise RuntimeError('FAST-LIO failed both attempts; bridge NOT started')

    def initialize_ros(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import VehicleOdometry
        from sensor_msgs.msg import Imu
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('kr260_position_monitor')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        def imu(_msg):
            now = time.monotonic()
            self.imu.append(now)
            while self.imu and self.imu[0] < now-WINDOW:
                self.imu.popleft()

        def pose(name, message):
            if name == 'odom':
                stamp = message.header.stamp
                ns = stamp.sec*10**9+stamp.nanosec
                p, q = message.pose.pose.position, message.pose.pose.orientation
                values = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
                valid = stamp.sec >= 0 and 0 <= stamp.nanosec < 10**9
            else:
                ns = message.timestamp_sample*1000
                values = list(message.position)+list(message.q)
                valid = True
            valid = valid and all(math.isfinite(v) for v in values) and sum(v*v for v in values[3:]) > 1e-12
            self.streams[name].record(time.monotonic(), ns,
                (self.node.get_clock().now().nanoseconds-ns)/1e6, valid)

        self.imu_sub = self.node.create_subscription(Imu, '/livox/imu', imu, qos)
        self.subscriptions = [
            self.node.create_subscription(Odometry, '/Odometry', lambda m: pose('odom', m), qos),
            self.node.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', lambda m: pose('vo', m), qos),
            self.node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                lambda m: self.streams['px4'].record(time.monotonic(), m.timestamp), qos),
        ]

    def dds_readers(self):
        return external_readers(self.node.get_subscriptions_info_by_topic('/fmu/in/vehicle_visual_odometry'))

    def scan_bridge(self):
        path = self.log_dir/'bridge_1.log'
        if not path.exists():
            return False
        # Incremental, bounded reading; never subscribe to or deserialize clouds.
        size = path.stat().st_size
        truncated = size < self.bridge_cursor or size-self.bridge_cursor > 65536
        with path.open('rb') as stream:
            stream.seek(max(0, size-65536) if truncated else self.bridge_cursor)
            lines = stream.read(65536)
            self.bridge_cursor = stream.tell()
        count = lines.count(b'odometry timeout')+lines.count(b'reject odometry')
        self.bridge_errors += count
        now, wall = time.monotonic(), time.time()
        if truncated:
            self.last_bridge_error = now  # unseen/rotated data is not proof of health
        for line in lines.splitlines():
            if b'odometry timeout' not in line and b'reject odometry' not in line:
                continue
            # ROS logger timestamps use system time. Preserve the event's age;
            # reading a startup error after READY must not make it a new fault.
            stamp = re.search(rb'\[(\d{9,}(?:\.\d+)?)\]', line)
            event = now
            if stamp is not None:
                age = wall-float(stamp.group(1))
                if age >= WINDOW:
                    continue  # counted for diagnostics, but outside current window
                if age >= 0:
                    event = now-age
            # Missing/future timestamps fail closed; clock monitoring is separate.
            if self.last_bridge_error is None or event > self.last_bridge_error:
                self.last_bridge_error = event
        return bool(count or truncated)

    def monitor(self):
        """Health-only monitoring. Deliberately does not call stop/start."""
        now = time.monotonic()
        metrics = {name: stream.stats(now) for name, stream in self.streams.items()}
        bad = [name+' rate/age/gap' for name in ('odom', 'vo') if not healthy(metrics[name])]
        if metrics['px4']['last_good_age_s'] is None or metrics['px4']['last_good_age_s'] > 1:
            bad.append('PX4 output missing/stalled >1s')
        bad.extend(name+' exited' for name, p in self.children.items() if p.poll() is not None)
        readers = self.dds_readers()
        if readers < 1:
            bad.append('PX4 DDS reader missing')
        self.scan_bridge()
        if self.last_bridge_error is not None and now-self.last_bridge_error < WINDOW:
            bad.append('bridge freshness rejection/timeout (see bridge log)')
        clock = (now, self.node.get_clock().now().nanoseconds)
        if self.last_clock and abs((clock[1]-self.last_clock[1])/1e9-(now-self.last_clock[0])) > .05:
            bad.append('system clock jumped relative to monotonic clock >50ms')
        self.last_clock = clock
        backwards = sum(s.backwards for s in self.streams.values())
        if backwards > self.record.get('backwards', 0):
            bad.append('sample timestamp moved backward')
        if bad:
            self.health_good_since = None
            state, reason = 'DEGRADED', '; '.join(bad)
        elif self.record['state'] == 'DEGRADED':
            if self.health_good_since is None:
                self.health_good_since = now
            state = 'READY' if now-self.health_good_since >= 5 else 'DEGRADED'
            reason = 'diagnostics recovered; startup only, not flight clearance' if state == 'READY' else 'waiting for 5s healthy recovery'
        else:
            state, reason = 'READY', 'startup passed; no flight clearance'
        self.publish(state, reason, metrics=metrics, dds_external_readers=readers,
                     bridge_freshness_events=self.bridge_errors, backwards=backwards)

    def run(self):
        self.log_dir.mkdir(parents=True, exist_ok=False)
        self.stage('validating configs/environment')
        validate_configs(self.args.config_dir)
        if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp' or os.environ.get('ROS_DOMAIN_ID', '0') != '0':
            raise RuntimeError('expected Fast DDS and ROS_DOMAIN_ID=0')
        if external_nodes():
            raise RuntimeError('external localization/Agent processes exist; refusing to adopt or stop them')
        if Path('/sys/class/tty/ttyS0/device').resolve().name != 'a0020000.serial':
            raise RuntimeError('PL UART not prepared; use kr260_position.sh start')
        from ament_index_python.packages import get_package_prefix
        def binary(package, executable):
            path = Path(get_package_prefix(package))/'lib'/package/executable
            if not os.access(path, os.X_OK):
                raise RuntimeError(f'missing executable: {path}')
            return str(path)
        config = self.args.config_dir
        driver = [binary('livox_ros_driver2', 'livox_ros_driver2_node'), '--ros-args',
            '-p', 'xfer_format:=1', '-p', 'multi_topic:=0', '-p', 'data_src:=0',
            '-p', 'publish_freq:=10.0', '-p', 'output_data_type:=0', '-p', 'frame_id:=livox_frame',
            '-p', f'user_config_path:={config / "MID360_kr260_sdk2.json"}',
            '-p', 'cmdline_input_bd_code:=livox0000000001']
        lio = [binary('fast_lio', 'fastlio_mapping'), '--ros-args', '--params-file',
               str(config/'mid360_kr260.yaml'), '-p', 'use_sim_time:=false']
        bridge = [binary('lio_px4_bridge', 'bridge_node'), '--ros-args', '-r', '__node:=lio_px4_bridge',
                  '--params-file', str(config/'bridge.yaml')]
        self.initialize_ros()
        self.stage('starting Agent and radar')
        self.start('agent', ['/usr/local/bin/MicroXRCEAgent', 'serial', '--dev', '/dev/ttyS0', '-b', '921600', '-v4'])
        self.start('driver', driver)
        self.wait_first('px4', 15)
        self.wait_first('imu', 10)
        self.stage('IMU stability check and 8s warmup')
        self.imu_window()
        self.spin(8)
        self.imu_window()
        self.release_imu()
        self.start_lio(lio)
        self.stage('starting bridge and checking fresh output/DDS match')
        self.streams['vo'] = Stream()
        self.start('bridge', bridge)
        self.wait_first('vo', 10)
        stats = self.window('vo', 3)
        deadline = time.monotonic()+10
        while self.dds_readers() < 1 and time.monotonic() < deadline:
            self.spin(.1)
        if not healthy(stats) or self.dds_readers() < 1 or time.monotonic()-self.streams['px4'].last_good > 1:
            raise RuntimeError('bridge/PX4 DDS startup gates failed')
        self.ready_at = time.monotonic()
        self.publish('READY', 'startup gates passed; NOT flight clearance', startup_vo=stats,
                     dds_external_readers=self.dds_readers())
        # Runtime health issues NEVER raise through startup cleanup.
        while True:
            self.spin(5, startup=False)
            self.monitor()

    def close(self):
        for name in reversed(list(self.children)):
            self.stop(name)
        if self.node is not None:
            self.release_imu()
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        for handle in self.log_handles:
            handle.close()


def unit_info():
    result = subprocess.run(['systemctl', 'show', UNIT, '--no-pager',
        '--property=LoadState,ActiveState,SubState,InvocationID,MainPID'], capture_output=True, text=True)
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def read_status(runtime_dir, info):
    try:
        record = json.loads((runtime_dir/'state.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        record = {}
    active = info.get('ActiveState')
    if info.get('LoadState') != 'loaded':
        return dict(state='FAILED', reason='service not installed; see deployment README')
    if record.get('run_id') != info.get('InvocationID'):
        record = {}
    if record and record.get('state') not in STATES:
        return dict(state='FAILED', reason='invalid/corrupted runtime state file; inspect logs')
    if active == 'failed':
        return dict(record, state='FAILED', reason=record.get('reason', 'systemd startup/process failure'))
    if active not in ('active', 'activating', 'reloading'):
        if record.get('state') == 'FAILED':
            return record
        return dict(record, state='STOPPED', reason='service is not running')
    if not record or record.get('run_id') != info.get('InvocationID'):
        return dict(state='STARTING', reason='waiting for current invocation status')
    if record['state'] in ('READY', 'DEGRADED') and time.monotonic()-record.get('updated_monotonic', 0) > 15:
        return dict(record, state='DEGRADED', reason='monitor status heartbeat stale >15s')
    return record


def display(record):
    print(json.dumps(record, ensure_ascii=False, indent=2))
    return {'READY': 0, 'STOPPED': 0, 'DEGRADED': 2, 'FAILED': 3, 'STARTING': 4}[record['state']]


@contextmanager
def control_lock(workspace):
    import fcntl
    directory = workspace/'logs'
    directory.mkdir(parents=True, exist_ok=True)
    with (directory/'.kr260-position-control.lock').open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def system_action(action):
    prefix = [] if os.geteuid() == 0 else ['sudo']
    command = prefix+['systemctl', action, UNIT]
    if action == 'start':
        command.append('--no-block')
    return subprocess.run(command).returncode


def control(args):
    if args.action == 'logs':
        command = ['journalctl', '-u', UNIT, '-n', '80', '--no-pager']
        if args.follow:
            command.append('--follow')
        return subprocess.call(command)
    if args.action == 'status':
        return display(read_status(args.runtime_dir, unit_info()))
    with control_lock(args.workspace):
        info = unit_info()
        if args.action == 'stop':
            print('Stopping ONLY this service. Use only while landed and DISARMED.', flush=True)
            result = system_action('stop')
            if result:
                return result
            return display(read_status(args.runtime_dir, unit_info()))
        if info.get('LoadState') != 'loaded':
            return display(dict(state='FAILED', reason='service not installed'))
        if info.get('ActiveState') in ('active', 'activating', 'reloading'):
            print('Already started/starting; no process restarted.', flush=True)
            return display(read_status(args.runtime_dir, info))
        print('Starting localization ONLY. Aircraft must be landed and DISARMED. No flight clearance.', flush=True)
        if system_action('start'):
            return 3
        old_invocation = info.get('InvocationID')
        started = time.monotonic()
        deadline = time.monotonic()+180
        previous = None
        while time.monotonic() < deadline:
            new_info = unit_info()
            record = read_status(args.runtime_dir, new_info)
            # --no-block can briefly expose the previous failed invocation.
            if (record['state'] in ('READY', 'DEGRADED', 'FAILED') and
                    new_info.get('InvocationID') == old_invocation and
                    time.monotonic()-started < 5):
                time.sleep(.5)
                continue
            current = (record['state'], record.get('reason'))
            if current != previous:
                print(f'{current[0]}: {current[1]}', flush=True)
                previous = current
            if record['state'] in ('READY', 'DEGRADED', 'FAILED'):
                return display(record)
            time.sleep(.5)
        print('STARTING timed out at 180s; no restart/stop was issued. Check status/logs before taking any action.', file=sys.stderr)
        return 4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['run', 'start', 'status', 'logs', 'stop'])
    parser.add_argument('--config-dir', type=Path)
    parser.add_argument('--workspace', type=Path, default=WORKSPACE)
    parser.add_argument('--runtime-dir', type=Path,
                        default=Path(os.environ.get('KR260_POSITION_RUNTIME_DIR', RUNTIME_DIR)))
    parser.add_argument('--follow', action='store_true')
    args = parser.parse_args()
    if args.action != 'run':
        try:
            return control(args)
        except KeyboardInterrupt:
            return 130
        except (OSError, ValueError) as error:
            print(f'control failed: {error}', file=sys.stderr)
            return 3
    if args.config_dir is None:
        parser.error('run requires --config-dir')
    import fcntl
    args.runtime_dir.mkdir(parents=True, exist_ok=True)
    lock_handle = (args.runtime_dir/'runtime.lock').open('a')
    try:
        fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Another position runtime holds the lock; no process changed.', file=sys.stderr)
        lock_handle.close()
        return 1
    runtime = Runtime(args)
    def stop(_signum, _frame):
        runtime.stop_requested = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    failed = False
    try:
        runtime.run()
    except InterruptedError:
        pass
    except Exception as error:
        failed = True
        runtime.publish('FAILED', str(error))
    finally:
        runtime.close()
        if not failed:
            runtime.publish('STOPPED', 'explicit stop/signal; owned children cleaned up')
        lock_handle.close()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
