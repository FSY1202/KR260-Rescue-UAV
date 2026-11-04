#!/usr/bin/env python3
"""Shared EGO recording and ground-check helpers; no waypoint execution."""


import argparse


import contextlib


import hashlib


import importlib.util


import json


import os


from pathlib import Path


import shutil


import signal


import sqlite3


import subprocess


import sys


import time


import uuid


TOOL = 'kr260-offboard-flight-log-v1'


TOPICS = (
    '/Odometry', '/fmu/in/vehicle_visual_odometry',
    '/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_status_v3',
    '/fmu/out/vehicle_land_detected', '/fmu/out/failsafe_flags',
    '/fmu/out/vehicle_odometry', '/fmu/out/vehicle_command_ack_v1',
    '/fmu/in/offboard_control_mode', '/fmu/in/trajectory_setpoint',
    '/fmu/in/vehicle_command',  
)


MIN_FREE = 1024**3


TERMINAL = {'BLOCKED', 'TAKEN_OVER', 'COMPLETE', 'ABORTED'}


def atomic_json(path, data, durable=False):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    with tmp.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')
        stream.flush()
        if durable:
            os.fsync(stream.fileno())
    os.replace(tmp, path)
    if durable and os.name != 'nt':
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def owner():
    try:
        return {'pid': os.getpid(), 'boot': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                'start': Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19]}
    except OSError:
        return {'pid': os.getpid(), 'boot': 'test', 'start': 'test'}


def alive(identity):
    try:
        return (identity['boot'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                and identity['start'] == Path('/proc', str(identity['pid']), 'stat').read_text().rsplit(')', 1)[1].split()[19])
    except (OSError, KeyError):
        return False


@contextlib.contextmanager
def exclusive_lock(path):
    with Path(path).open('a+b') as stream:
        if os.name == 'nt':
            import msvcrt
            stream.seek(0)
            if stream.read(1) == b'':
                stream.write(b'0')
                stream.flush()
            stream.seek(0)
            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            yield
        finally:
            if os.name == 'nt':
                stream.seek(0)
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)


class SessionStore:
    def __init__(self, root, keep=20, topics=TOPICS):
        self.root = Path(root).absolute()
        self.topics = tuple(topics)
        if keep != 20:
            raise ValueError('this deployment retains exactly 20 sessions')
        self.keep = keep
        for part in (self.root, *self.root.parents):
            if part.is_symlink():
                raise ValueError('log root/ancestors must not be symlinks')
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        if self.root.resolve() != self.root:
            raise ValueError('noncanonical log root')

    def entries(self):
        records = []
        for path in self.root.iterdir():
            if path.is_symlink() or not path.is_dir():
                continue
            marker = path / 'session.json'
            if marker.is_symlink():
                continue
            data = read_json(marker)
            if (data.get('tool') == TOOL and isinstance(data.get('sequence'), int)
                    and path.name == data.get('directory')):
                records.append((data['sequence'], path, data))
        return sorted(records, key=lambda item: item[0])

    def recover(self):
        # A prior power loss is never silently labelled a successful flight.
        for _, path, data in self.entries():
            if data.get('state') == 'ACTIVE' and not alive(data.get('owner', {})):
                data.update(state='INCOMPLETE', reason='previous owner gone; bag may need reindexing')
                atomic_json(path / 'session.json', data, durable=True)

    def create(self, run_id):
        with exclusive_lock(self.root / '.retention.lock'):
            self.recover()
            records = self.entries()
            if any(data.get('state') == 'ACTIVE' for _, _, data in records):
                raise RuntimeError('another active managed recording exists; never delete or take it over')
            seq = max([read_json(self.root / 'index.json').get('last_sequence', 0)]
                      + [n for n, _, _ in records]) + 1
            directory = f'{seq:06d}_{time.strftime("%Y%m%d_%H%M%S")}_{run_id[:8]}'
            path = self.root / directory
            path.mkdir(mode=0o700)
            data = dict(tool=TOOL, sequence=seq, directory=directory, run_id=run_id,
                        state='ACTIVE', owner=owner(), started_wall=time.time(),
                        started_monotonic=time.monotonic(), topics=list(self.topics), armed_seen=False)
            atomic_json(path / 'session.json', data, durable=True)
            atomic_json(self.root / 'index.json', {'last_sequence': seq}, durable=True)
            # A new owned record exists before rotation; never remove an active one.
            try:
                self.prune()
            except Exception:
                data.update(state='INCOMPLETE', reason='retention failed; no task started')
                atomic_json(path / 'session.json', data, durable=True)
                raise
            return path

    def prune(self):
        records = self.entries()
        while len(records) > self.keep:
            candidates = [record for record in records if record[2].get('state') in
                          {'COMPLETE', 'CANCELLED', 'INCOMPLETE'}]
            if not candidates:
                raise RuntimeError('cannot rotate without deleting an active record')
            seq, path, data = candidates[0]
            if path.is_symlink() or path.resolve().parent != self.root:
                raise ValueError('unsafe deletion target')
            # Refuse even nested links. Unknown user material is never followed.
            if any(p.is_symlink() for p in path.rglob('*')):
                raise ValueError('refusing to delete a log containing symlinks')
            if read_json(path / 'session.json') != data:
                raise RuntimeError('record changed while rotating')
            shutil.rmtree(path)
            with (self.root / 'retention.jsonl').open('a', encoding='utf-8') as log:
                log.write(json.dumps({'deleted_sequence': seq, 'directory': path.name,
                                      'wall': time.time(), 'recoverable': False}) + '\n')
            records = self.entries()

    def close(self, path, outcome, reason, controller=None):
        if path.resolve().parent != self.root or path.is_symlink():
            raise ValueError('invalid managed record')
        data = read_json(path / 'session.json')
        if data.get('tool') != TOOL or data.get('owner') != owner():
            raise RuntimeError('not our recording')
        data.update(state=outcome, reason=reason, ended_wall=time.time(),
                    ended_monotonic=time.monotonic(), controller=controller or {},
                    armed_seen=bool((controller or {}).get('ever_armed') or (controller or {}).get('triggered')))
        atomic_json(path / 'session.json', data, durable=True)


def bag_counts(bag):
    """Read committed messages, not just a recorder PID or an empty DB file."""
    counts = {}
    for db in Path(bag).glob('*.db3'):
        with contextlib.closing(sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True, timeout=.05)) as conn:
            for name, count in conn.execute('SELECT topics.name,COUNT(*) FROM messages '
                                            'JOIN topics ON messages.topic_id=topics.id GROUP BY topics.name'):
                counts[name] = counts.get(name, 0) + count
    return counts


def recorder_command(path, qos, topics=TOPICS, node_name='kr260_ego_recorder'):
    return ['ros2', 'bag', 'record', '--storage', 'sqlite3', '--storage-preset-profile', 'resilient',
            '--max-cache-size', '0', '--max-bag-size', str(16*1024**2),
            '--qos-profile-overrides-path', str(qos), '--node-name', node_name,
            '-o', str(path / 'bag'), *topics]


def bag_progress(bag):
    # Index lookup, not a full COUNT/GROUP BY scan on every health check.
    total = 0
    for db in Path(bag).glob('*.db3'):
        with contextlib.closing(sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True, timeout=.05)) as conn:
            total += conn.execute('SELECT COALESCE(MAX(rowid),0) FROM messages').fetchone()[0]
    return total


def start_owned(cmd, path):
    handle = path.open('ab', buffering=0)
    try:
        # Native helper avoids Python preexec_fn after ROS/DDS threads have existed.
        # setpriv changes ONLY the parent-death signal, not UID/groups/capabilities.
        proc = subprocess.Popen(['/usr/bin/setpriv', '--pdeathsig', 'SIGTERM', '--', *cmd],
                                stdin=subprocess.DEVNULL, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True)
    except Exception:
        handle.close()
        raise
    return proc, handle


def end_owned(proc, timeout=10):
    if proc is None:
        return True
    if proc.poll() is not None:
        return proc.returncode == 0
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        proc.wait(timeout=3)
        return proc.returncode == 0
    try:
        proc.wait(timeout=timeout)
        return proc.returncode in (0, -signal.SIGINT)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3)
        return False


def controller_state(runtime, run_id):
    state = read_json(runtime / 'controller.json')
    if state.get('run_id') != run_id or not 0 <= time.monotonic() - state.get('updated_monotonic', 0) < .7:
        return {}
    return state


def ground_precheck(timeout=8):
    """Read-only check BEFORE creating/rotating any log or flight publishers."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleStatus, VehicleLandDetected, FailsafeFlags
    from rclpy.serialization import deserialize_message
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('kr260_ego_precheck', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    latest = {}
    critical_names = interface_checker().CRITICAL_FLAGS
    flag_decode_errors = []
    flag_raw_count = 0
    def receive(name, msg):
        previous = latest.get(name)
        age = (time.time_ns()//1000-msg.timestamp)/1e6
        if msg.timestamp and -.01 <= age <= 2 and (not previous or msg.timestamp>previous[0].timestamp):
            latest[name] = (msg, time.monotonic())
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v3', lambda msg: receive('status', msg), qos_profile_sensor_data)
    node.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', lambda msg: receive('land', msg), qos_profile_sensor_data)
    def receive_flags(raw):
        nonlocal flag_raw_count
        flag_raw_count += 1
        try:
            if len(raw) != 96:
                raise ValueError('failsafe_flags CDR length is not pinned firmware\'s 96 bytes')
            receive('flags', deserialize_message(raw, FailsafeFlags))
        except Exception as exc:
            flag_decode_errors.append(str(exc))
    node.create_subscription(FailsafeFlags, '/fmu/out/failsafe_flags', receive_flags,
                             qos_profile_sensor_data, raw=True)
    try:
        deadline = time.monotonic()+timeout
        missing = []
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
            if flag_decode_errors:
                raise RuntimeError('PX4_CDR_DECODE_ERROR failsafe_flags: ' + flag_decode_errors[0])
            for topic in ('/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode'):
                if node.get_publishers_info_by_topic(topic):
                    raise RuntimeError('external Offboard publisher; do not take over or rotate flight logs')
            status = latest.get('status')
            land = latest.get('land')
            if status and status[0].arming_state != VehicleStatus.ARMING_STATE_DISARMED:
                raise RuntimeError('already armed: prepare and log rotation refused')
            if status and land and max(time.monotonic()-status[1],time.monotonic()-land[1])<2:
                if status[0].nav_state != VehicleStatus.NAVIGATION_STATE_POSCTL or not land[0].landed:
                    raise RuntimeError('prepare requires Position mode, landed and disarmed; then arm and select Offboard on ground')
                missing = [topic for topic in ('/fmu/out/vehicle_local_position_v1', '/fmu/out/failsafe_flags',
                                               '/fmu/out/vehicle_command_ack_v1')
                           if not node.get_publishers_info_by_topic(topic)]
                if missing:
                    continue
                flags = latest.get('flags')
                if not flags or time.monotonic()-flags[1] >= 2:
                    continue
                critical = [name for name in critical_names if getattr(flags[0], name)]
                if critical:
                    raise RuntimeError('PX4 critical flags: ' + ', '.join(critical))
                if status[0].failsafe or not status[0].pre_flight_checks_pass:
                    raise RuntimeError('PX4 pre-flight checks not passed in Position or vehicle_status.failsafe active')
                return
        flags_published = bool(node.get_publishers_info_by_topic('/fmu/out/failsafe_flags'))
        raise RuntimeError('fresh PX4 telemetry missing: '+str(missing)+
                           f'; failsafe publisher={flags_published}, raw_frames={flag_raw_count}, valid_flags={bool(latest.get("flags"))}'+
                           '; no log rotation or control startup')
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def interface_checker():
    spec = importlib.util.spec_from_file_location('kr260_px4_interface_check', Path(__file__).with_name('px4_interface_check.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_ground_checks(workspace, runtime, run_id, update, children, handles, cancelled):
    """Heavy ROS initialization runs in an owned child; main thread owns all status writes."""
    result_path = runtime / 'precheck.result.json'
    result_path.unlink(missing_ok=True)
    proc, handle = start_owned([sys.executable, str(Path(__file__).resolve()), '_check-ground',
                               '--workspace', str(workspace), '--runtime', str(runtime)], runtime / 'precheck.log')
    children['precheck'] = proc
    handles.append(handle)
    deadline = time.monotonic() + 20
    while proc.poll() is None:
        update(state='PREPARING', reason='checking pinned PX4 interfaces and fresh Position ground telemetry',
               stage='ground_checks', stage_deadline_monotonic=deadline)
        if cancelled() or time.monotonic() >= deadline:
            end_owned(proc)
            raise RuntimeError('ground/interface checks cancelled or timed out after 20 seconds; see ' + str(runtime / 'precheck.log'))
        time.sleep(.2)
    result = read_json(result_path)
    if proc.returncode or not result.get('ok') or result.get('run_id') != run_id:
        raise RuntimeError(result.get('error', 'ground/interface check child failed; see ' + str(runtime / 'precheck.log')))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['_check-ground'])
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    args = parser.parse_args(argv)
    run_id = read_json(args.runtime / 'supervisor.json').get('run_id')
    try:
        lock = args.workspace / 'install/kr260_ego_support/share/kr260_ego_support/config/px4_interface_lock.json'
        checked = interface_checker().verify_interfaces(lock)
        ground_precheck()
        atomic_json(args.runtime / 'precheck.result.json', dict(ok=True, run_id=run_id, **checked))
        return 0
    except Exception as exc:
        atomic_json(args.runtime / 'precheck.result.json', dict(ok=False, run_id=run_id, error=str(exc)))
        print('GROUND_CHECK_FAILED: ' + str(exc), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
