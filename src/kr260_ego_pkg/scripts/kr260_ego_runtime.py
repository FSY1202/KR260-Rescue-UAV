#!/usr/bin/env python3
"""Independent EGO service. No auto-arm, auto-mode, control restart or flight clearance.

Reuse the proven recorder/retention/ground checks, not the fixed mission controller.
All lifecycle helpers own explicit child handles; no global pkill or Agent stop.
"""
import argparse
import importlib.util
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import time

UNIT = 'kr260-ego.service'
SELECTION_RELATIVE = Path('config/kr260_ego_active_stage.json')
SELECTED_RUNTIME = 'kr260_ego_selected_runtime.py'
VALIDATOR = 'kr260_ego_stage_validator.py'
ROS_SETUP = Path('/opt/ros/humble/setup.bash')
TERMINAL = {'BLOCKED', 'TAKEN_OVER', 'COMPLETE', 'ABORTED'}
EXTRA_TOPICS = ('/kr260/ego/status', '/kr260/ego/events', '/kr260/ego/goal',
                '/kr260/ego/bspline', '/kr260/ego/planner_health', '/kr260/ego/odometry',
                '/kr260/rescue/person', '/kr260/rescue/drop_request', '/kr260/rescue/drop_feedback')
PLANNER_REMAPS = {
    'odom_world': '/kr260/ego/odometry', 'grid_map/odom': '/kr260/ego/odometry',
    'grid_map/cloud': '/kr260/ego/cloud', 'planning/bspline': '/kr260/ego/bspline',
    '/move_base_simple/goal': '/kr260/ego/goal',
    'planning/data_display': '/kr260/ego/planning_data',
    'planning/broadcast_bspline_from_planner': '/kr260/ego/broadcast_out',
    'planning/broadcast_bspline_to_planner': '/kr260/ego/broadcast_in',
    'grid_map/occupancy': '/kr260/ego/occupancy', 'grid_map/occupancy_inflate': '/kr260/ego/occupancy_inflate',
    'grid_map/virtual_ceiling': '/kr260/ego/virtual_ceiling',
    **{key: '/kr260/ego/'+key for key in ('goal_point','global_list','init_list','optimal_list','a_star_list')},
}


def selection_path(path, directory=False):
    path = Path(path)
    if (not path.is_absolute() or '..' in path.parts
            or any(part.is_symlink() for part in (path, *path.parents))
            or not (path.is_dir() if directory else path.is_file())):
        raise RuntimeError('canonical nonsymlink selection input required: ' + str(path))
    return path.resolve(strict=True)


def selected_bundle(workspace, require_selection=False):
    """Read a separate selection receipt; preserve all stage manifests/helpers."""
    workspace = Path(workspace)
    receipt_path = workspace / SELECTION_RELATIVE
    required = require_selection or Path(__file__).name == SELECTED_RUNTIME
    if not receipt_path.exists() and not receipt_path.is_symlink():
        if required:raise RuntimeError('selected EGO receipt missing; refusing legacy fallback')
        return None
    workspace = selection_path(workspace, directory=True)
    receipt_path = selection_path(receipt_path)
    payload = receipt_path.read_bytes()
    receipt_sha = hashlib.sha256(payload).hexdigest()
    expected = os.environ.get('KR260_EGO_SELECTION_SHA256')
    if expected is not None and expected != receipt_sha:
        raise RuntimeError('EGO selection receipt changed since launcher validation')
    receipt = json.loads(payload)
    if (type(receipt.get('schema')) is not int or receipt['schema'] != 1
            or receipt.get('state') != 'SELECTED_NOT_FLIGHT_ACCEPTED'):
        raise RuntimeError('unsupported EGO selection receipt schema/state')
    stage = selection_path(receipt['stage'], directory=True)
    if stage.parent != workspace or not stage.name.startswith('ego_realtime_20261021_'):
        raise RuntimeError('selected stage is outside its dedicated workspace directory')
    hashes = receipt.get('sha256')
    if not isinstance(hashes, dict) or not hashes:
        raise RuntimeError('selection receipt must pin critical files')
    verified = set()
    for name, checksum in hashes.items():
        path = selection_path(name)
        if str(path) != name or not (path.is_relative_to(workspace) or path == ROS_SETUP):
            raise RuntimeError('selection hash path outside reviewed scope: ' + name)
        if not isinstance(checksum, str) or not re.fullmatch('[0-9a-f]{64}', checksum):
            raise RuntimeError('invalid selection SHA256: ' + name)
        actual = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):actual.update(chunk)
        if actual.hexdigest() != checksum:
            raise RuntimeError('selected artifact SHA256 differs: ' + name)
        verified.add(path)
    validator_path = workspace / 'scripts' / VALIDATOR
    runtime_path = workspace / 'scripts' / SELECTED_RUNTIME
    launcher_path = workspace / 'scripts/kr260_ego.sh'
    manifest = stage / 'manifest.json'
    for path in (validator_path, runtime_path, launcher_path, manifest):
        if path not in verified:raise RuntimeError('selection receipt lacks required input: ' + str(path))
    if hashes[str(manifest)] != receipt.get('manifest_sha256'):
        raise RuntimeError('selected manifest SHA256 differs from receipt')
    if Path(__file__).name == SELECTED_RUNTIME and Path(__file__).resolve() != runtime_path:
        raise RuntimeError('selected runtime executed from an unexpected path')
    # Execute only the already hashed validator bytes, avoiding a second source
    # read or pycache writes. Its existing seven-package acceptance stays intact.
    spec = importlib.util.spec_from_file_location('kr260_selected_stage_validator', validator_path)
    validator = importlib.util.module_from_spec(spec)
    validator_source=validator_path.read_bytes()
    if hashlib.sha256(validator_source).hexdigest()!=hashes[str(validator_path)]:
        raise RuntimeError('selected validator changed before loading')
    exec(compile(validator_source, str(validator_path), 'exec'), validator.__dict__)
    build_log = selection_path(receipt['build_log'])
    bundle = validator.validate_stage(stage, build_log, workspace=workspace)
    required_paths = {validator_path, runtime_path, launcher_path, manifest, stage / 'build.sh',
        bundle['planner'], bundle['controller'], bundle['archive'], bundle['link_proof'],
        bundle['config'], bundle['helper'], bundle['build_log'], *bundle['libs'].values(),
        bundle['share'] / 'config/planner.yaml', bundle['share'] / 'config/bag_qos.yaml',
        workspace / 'install/kr260_ego_support/lib/kr260_ego_support/ego_runtime_support.py',
        workspace / 'install/kr260_ego_support/share/kr260_ego_support/config/px4_interface_lock.json',
        workspace / 'install/local_setup.bash', stage / 'install/local_setup.bash', ROS_SETUP}
    missing = required_paths - verified
    if missing:raise RuntimeError('selection receipt lacks required input: ' + str(sorted(map(str, missing))))
    if receipt_path.read_bytes() != payload:
        raise RuntimeError('selection receipt changed during validation')
    return dict(bundle, selection_receipt=receipt_path, selection_sha256=receipt_sha)


def selected_stage(workspace, require_selection=False):
    bundle = selected_bundle(workspace, require_selection)
    return bundle['stage'] if bundle is not None else Path(workspace) / 'ego_stage_20261018'


def base_module(workspace):
    source = workspace / 'install/kr260_ego_support/lib/kr260_ego_support/ego_runtime_support.py'
    spec = importlib.util.spec_from_file_location('kr260_ego_helpers', source)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def validate_config(path, execute):
    import yaml
    value = yaml.safe_load(Path(path).read_text())
    p = value['kr260_ego_controller']['ros__parameters']
    numeric = ('height_m', 'forward_m', 'left_m', 'speed_mps', 'acceleration_mps2', 'settle_seconds',
               'airframe_radius_m', 'airframe_half_height_m')
    for key in numeric:
        if isinstance(p[key], bool) or not isinstance(p[key], (int, float)) or not math.isfinite(p[key]):
            raise ValueError('finite numeric config required: ' + key)
    if not (.6 <= p['height_m'] <= 1.4 and 0 < p['speed_mps'] <= .2 and
            0 < p['acceleration_mps2'] <= .2 and p['settle_seconds'] >= 2):
        raise ValueError('mission exceeds initial height/speed/acceleration/settle limits')
    lower, upper = p['site_min'], p['site_max']
    if not isinstance(lower, list) or not isinstance(upper, list) or len(lower) != 3 or len(upper) != 3:
        raise ValueError('site_min/site_max need 3 coordinates')
    for i, (lo, hi, floor, ceiling) in enumerate(zip(lower, upper, [-1, -2, .6], [4, 2, 1.4])):
        if not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in (lo, hi)) or not floor <= lo < hi <= ceiling:
            raise ValueError(f'site axis {i} outside initial envelope')
    if not all(lo <= goal <= hi for lo, goal, hi in zip(lower, [p['forward_m'], p['left_m'], p['height_m']], upper)):
        raise ValueError('goal outside confirmed site')
    if not all(lo <= start <= hi for lo, start, hi in zip(lower, [0, 0, p['height_m']], upper)):
        raise ValueError('climb endpoint outside confirmed site')
    route=p.get('route_points_m',[])
    if not isinstance(route,list) or len(route)%3 or len(route)>96:
        raise ValueError('route_points_m needs at most 32 forward/left/up triples')
    previous=[0.,0.,p['height_m']]
    for index in range(0,len(route),3):
        goal=route[index:index+3]
        if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in goal):
            raise ValueError('route coordinates must be finite numbers')
        if not all(lo<=v<=hi for lo,v,hi in zip(lower,goal,upper)) or abs(goal[2]-p['height_m'])>1e-6:
            raise ValueError('route goal outside site or not at the configured cruise height')
        if math.sqrt(sum((a-b)**2 for a,b in zip(goal,previous)))<.15:
            raise ValueError('route goals need at least 0.15m separation')
        previous=goal
    if execute:
        if not all(p.get(k) is True for k in ('site_confirmed', 'takeoff_clear', 'landing_clear')):
            raise ValueError('execution blocked: measured site/takeoff/landing declarations are missing; preview only')
        if not (0 < p['airframe_radius_m'] <= .7 and 0 < p['airframe_half_height_m'] <= .5):
            raise ValueError('execution blocked: measured airframe (INCLUDING propellers) dimensions missing/unsupported')
    return p


def recording_store(b, workspace, execute, topics):
    if execute:
        return b.SessionStore(workspace / 'flight_logs', topics=topics)
    class PreviewStore(b.SessionStore):
        def prune(self):
            # Preview is not a flight preparation; never evict real flight bags.
            # No unrequested automatic deletion of preview diagnostics either.
            return
    return PreviewStore(workspace / 'ego_preview_logs', topics=topics)


def info():
    result = subprocess.run(['systemctl', 'show', UNIT, '-p', 'ActiveState', '-p', 'InvocationID'], text=True, capture_output=True, check=False)
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def display(b, runtime):
    unit, data = info(), b.read_json(runtime / 'supervisor.json')
    if unit.get('ActiveState') == 'failed' and data.get('run_id') != unit.get('InvocationID'):
        return dict(state='FAILED', reason='service failed before publishing status; inspect logs', last_record=data)
    if data.get('run_id') != unit.get('InvocationID'):
        return dict(state='PREPARING' if unit.get('ActiveState') in ('active', 'activating') else 'STOPPED', service_active=unit.get('ActiveState'), last_record=data)
    data['service_active'] = unit.get('ActiveState')
    if unit.get('ActiveState') == 'failed':
        data.update(state='FAILED')
    elif unit.get('ActiveState') == 'active' and not 0 <= time.monotonic()-data.get('updated_monotonic', 0) < 2:
        data.update(state='DEGRADED', reason='supervisor stale; do NOT restart in air')
    return data


def print_operator_notice(data):
    """Terminal-only presentation; never grants readiness or changes state."""
    state = data.get('state')
    if state not in ('READY_TO_ARM', 'WAIT_OFFBOARD_ON_GROUND', 'FAILED', 'BLOCKED'):
        return
    lines = {
        'READY_TO_ARM': ('READY_TO_ARM | 任务准备完成',
                         '下一步：Position 下手动解锁，再于 10 秒内地面切入 Offboard。',
                         '仅本次状态有效；不是飞行安全许可。'),
        'WAIT_OFFBOARD_ON_GROUND': ('WAIT_OFFBOARD_ON_GROUND | 已解锁，等待 Offboard',
                                    '保持地面；在解锁后 10 秒内切入 Offboard。'),
        'FAILED': ('FAILED | 启动失败，不要解锁', str(data.get('reason', ''))),
        'BLOCKED': ('BLOCKED | 任务被阻止，不要解锁', str(data.get('reason', ''))),
    }[state]
    color = '\033[1;32m' if state == 'READY_TO_ARM' else '\033[1;33m' if state == 'WAIT_OFFBOARD_ON_GROUND' else '\033[1;31m'
    colored = sys.stdout.isatty() and 'NO_COLOR' not in os.environ
    border = '=' * 76
    message = '\n' + border + '\n' + '\n'.join('  ' + line for line in lines) + '\n' + border + '\n'
    print((color + message + '\033[0m') if colored else message, flush=True)


def planning_parameter(enable, require_old=None):
    """Lease only the optional output. No algorithm params, process restart or filters."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    context = rclpy.context.Context()
    node = executor = None
    try:
        rclpy.init(context=context)
        node = rclpy.create_node('kr260_ego_cloud_lease', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        getter = node.create_client(GetParameters, '/laser_mapping/get_parameters')
        setter = node.create_client(SetParameters, '/laser_mapping/set_parameters')
        if not getter.wait_for_service(timeout_sec=3) or not setter.wait_for_service(timeout_sec=3):
            raise RuntimeError('FAST-LIO planning output parameter service unavailable; no restart attempted')
        request = GetParameters.Request();request.names = ['planning_cloud.enable']
        future = getter.call_async(request);executor.spin_until_future_complete(future, timeout_sec=3)
        if not future.done() or future.exception():
            raise RuntimeError('planning cloud get parameter timed out')
        values = future.result().values
        if len(values) != 1 or values[0].type != ParameterType.PARAMETER_BOOL:
            raise RuntimeError('FAST-LIO does not provide optional planning output; install staged extension while grounded')
        old = values[0].bool_value
        if require_old is not None and old != require_old:
            raise RuntimeError('planning cloud already leased/changed by another operator; not taking over')
        req = SetParameters.Request();req.parameters = [Parameter(name='planning_cloud.enable', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enable))]
        future = setter.call_async(req);executor.spin_until_future_complete(future, timeout_sec=3)
        if not future.done() or future.exception() or not all(r.successful for r in future.result().results):
            raise RuntimeError('planning cloud parameter change not acknowledged')
        return old
    finally:
        # All resources use the same private context. Never construct the global
        # executor, and unwind even if initialization or an earlier cleanup fails.
        try:
            if executor is not None:
                try:
                    if node is not None:executor.remove_node(node)
                finally:
                    executor.shutdown()
        finally:
            try:
                if node is not None:node.destroy_node()
            finally:
                if context.ok():context.shutdown()


def supervisor(b, workspace, runtime):
    run = os.environ.get('INVOCATION_ID')
    if not run:
        raise RuntimeError('service invocation required')
    runtime.mkdir(parents=True, exist_ok=True)
    with b.exclusive_lock(workspace / '.kr260_offboard.lock'), b.exclusive_lock(runtime / 'owner.lock'):
        return owned(b, workspace, runtime, run)


def request_localization(workspace):
    """CLI only: request existing guarded start, never force-restart localization."""
    print('Starting/checking localization; EGO will wait for stable READY before creating control publishers.',flush=True)
    result=subprocess.run([str(workspace / 'scripts/kr260_position.sh'),'start'],check=False)
    # Position exit 2 means DEGRADED, not a successful launch. Allow the owned
    # supervisor to wait with a deadline; do not treat it as READY here.
    if result.returncode not in (0,2):
        raise RuntimeError('localization start failed (code %s); EGO not started'%result.returncode)


def wait_for_localization(b, update, cancelled, active=None, clock=None, sleep=None):
    """Preparation only. No ROS subscriptions/publishers, mode or process changes."""
    clock=clock or time.monotonic;sleep=sleep or time.sleep
    active=active or (lambda:subprocess.run(['systemctl','is-active','--quiet','kr260-position.service'],
                                          capture_output=True).returncode==0)
    deadline=clock()+45;since=None;first_stamp=None;previous_stamp=None;run=None;last_report=-1e9
    last_state='missing';last_reason='no localization state'
    while clock()<deadline:
        if cancelled():raise RuntimeError('localization preparation cancelled; no controller started')
        if not active():raise RuntimeError('localization service inactive; no controller started')
        now=clock();localized=b.read_json('/run/kr260-position/state.json')
        last_state=localized.get('state','missing');last_reason=localized.get('reason','no reason')
        stamp=localized.get('updated_monotonic');new_run=localized.get('run_id')
        valid_stamp=isinstance(stamp,(int,float)) and not isinstance(stamp,bool) and math.isfinite(stamp)
        fresh=valid_stamp and 0<=now-stamp<15
        if last_state in ('FAILED','STOPPED'):
            raise RuntimeError('localization %s: %s; no controller started'%(last_state,last_reason))
        if last_state=='READY' and fresh and new_run:
            if since is None or new_run!=run or (previous_stamp is not None and stamp<previous_stamp):
                since=now;first_stamp=stamp;run=new_run
            previous_stamp=stamp
            # Require an advancing monitor update, not five seconds of one stale
            # READY snapshot from a stopped monitor process.
            if now-since>=5 and stamp>first_stamp:
                update(state='PREPARING',stage='ground/interface checks',reason='localization stable READY; no controller started yet')
                return localized
        else:
            since=None;first_stamp=None;previous_stamp=None;run=None
        if now-last_report>=1:
            stable=0 if since is None else max(0,now-since)
            update(state='PREPARING',stage='waiting for stable localization',
                   reason='localization %s; stable %.1f/5s: %s'%(last_state,stable,last_reason))
            last_report=now
        sleep(.5)
    raise RuntimeError('localization did not become stable READY within 45s: %s: %s; no controller started'%(last_state,last_reason))


def owned(b, workspace, runtime, run):
    execute = b.read_json(workspace / 'config/kr260_ego_request.json').get('execute') is True
    stage = selected_stage(workspace)
    share = stage / 'install/kr260_ego_pkg/share/kr260_ego_pkg'
    config = workspace / 'config/kr260_ego.yaml';params = validate_config(config, execute)
    topics = tuple(t for t in b.TOPICS if not t.startswith('/kr260/mission/')) + EXTRA_TOPICS
    if not execute:topics+=('/kr260/ego/preview_odometry',)
    state = dict(run_id=run, state='PREPARING', execute=execute, flight_clearance=False, owner=b.owner())
    def update(**fields):
        state.update(fields, updated_monotonic=time.monotonic());b.atomic_json(runtime / 'supervisor.json', state)
    for name in ('controller.json', 'recorder.ready', 'request.stop', 'request.land', 'stop.approved'):
        (runtime / name).unlink(missing_ok=True)
    update(reason='ground/interface checks; no control publishers started')
    children, handles = {}, []
    path = None;store = None;lease = False;last = {};outcome = 'INCOMPLETE';reason = 'interrupted';cancel = False;logger_bad = False;ever_armed=False
    def on_signal(_s, _f):
        nonlocal cancel
        cancel = True
    signal.signal(signal.SIGINT, on_signal);signal.signal(signal.SIGTERM, on_signal)
    try:
        wait_for_localization(b,update,lambda:cancel)
        # Existing checker is an owned child and verifies pinned types and Position/landed/disarmed.
        checked = b.run_ground_checks(workspace, runtime, run, update, children, handles, lambda: cancel)
        update(stage='recording', reason='starting lightweight bag', px4_interface_commit=checked['firmware_commit'])
        store = recording_store(b,workspace,execute,topics)
        if shutil.disk_usage(store.root).free < b.MIN_FREE:
            raise RuntimeError('free disk <1 GiB')
        path = store.create(run)
        for source in (config, share / 'config/planner.yaml', share / 'config/bag_qos.yaml',
                       workspace / 'install/kr260_ego_support/share/kr260_ego_support/config/px4_interface_lock.json',
                       stage / 'manifest.json'):
            shutil.copy2(source, path / source.name)
        binary = stage / 'install/kr260_ego_pkg/lib/kr260_ego_pkg/kr260_ego_node'
        planner = stage / 'install/ego_planner/lib/ego_planner/ego_planner_node'
        b.atomic_json(path / 'version.json', dict(
            px4_interface_commit=checked['firmware_commit'], run_id=run, execute=execute,
            effective_overrides=dict(execute=execute,run_id=run,runtime_dir=str(runtime)),
            topics=topics, ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
            sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (config, binary, planner, Path(__file__))}), durable=True)
        update(record_dir=str(path), bag_path=str(path / 'bag'), recording='STARTING',
               record_kind='FLIGHT' if execute else 'PREVIEW', record_store=str(store.root))
        children['recorder'], handle = b.start_owned(b.recorder_command(path, share / 'config/bag_qos.yaml', topics, 'kr260_ego_recorder'), path / 'recorder.log');handles.append(handle)
        deadline = time.monotonic()+15
        while True:
            try: counts = b.bag_counts(path / 'bag')
            except sqlite3.Error: counts = {}
            if counts.get('/fmu/out/vehicle_local_position_v1', 0) and counts.get('/fmu/out/vehicle_status_v3', 0):break
            if cancel or time.monotonic()>deadline or children['recorder'].poll() is not None:
                raise RuntimeError('recording did not commit fresh PX4 position/status within 15s')
            update();time.sleep(.2)
        # Parameter call runs in a short-lived owned child: no fork after DDS initialization in this supervisor.
        command = [sys.executable, str(Path(__file__).resolve()), '_cloud-on', '--workspace', str(workspace), '--runtime', str(runtime)]
        children['cloud_on'], handle = b.start_owned(command, path / 'cloud_lease.log');handles.append(handle)
        deadline=time.monotonic()+15
        while children['cloud_on'].poll() is None:
            if cancel or time.monotonic()>deadline:raise RuntimeError('planning output lease timed out')
            update(reason='enabling optional 5 Hz dewarped cloud');time.sleep(.2)
        lease = (runtime / 'cloud.lease').is_file() and (runtime / 'cloud.lease').read_text().strip()==run
        if children['cloud_on'].returncode or not lease:raise RuntimeError('planning output lease failed; inspect cloud_lease.log')
        radius = params['airframe_radius_m']+.15 if params['airframe_radius_m']>0 else .5
        height = params['airframe_half_height_m']+.10 if params['airframe_half_height_m']>0 else .3
        cmd = [str(planner), '--ros-args', '-r', '__node:=kr260_ego_planner', '--params-file', str(share / 'config/planner.yaml'),
               '-p', f'kr260.run_id:={run}', '-p', f'kr260.radius:={radius}', '-p', f'kr260.half_height:={height}',
               '-p', 'kr260.site_min:='+json.dumps(params['site_min']), '-p', 'kr260.site_max:='+json.dumps(params['site_max'])]
        for source,target in PLANNER_REMAPS.items():
            if not execute and target=='/kr260/ego/odometry':target='/kr260/ego/preview_odometry'
            cmd.extend(['-r',source+':='+target])
        children['planner'], handle = b.start_owned(cmd, path / 'planner.log');handles.append(handle)
        if params.get('rescue_enabled',True):
            bridge = binary.parent / 'vision_status_bridge.py'
            if not bridge.is_file():raise RuntimeError('vision status bridge missing; rebuild EGO package')
            children['vision_bridge'], handle = b.start_owned([sys.executable,str(bridge)], path / 'vision_bridge.log');handles.append(handle)
        b.atomic_json(runtime / 'recorder.ready', time.monotonic())
        children['controller'], handle = b.start_owned([str(binary), '--ros-args', '--params-file', str(config),
            '-p', 'execute:='+str(execute).lower(), '-p', f'run_id:={run}', '-p', f'runtime_dir:={runtime}'], path / 'controller.log');handles.append(handle)
        last_count=sum(counts.values());last_growth=time.monotonic();last_check=0.;last_summary=0.;ground_since=None
        while True:
            now=time.monotonic();current=b.controller_state(runtime, run)
            if current:last=current;ever_armed=ever_armed or bool(current.get('armed'))
            if children['controller'].poll() is not None:
                if (runtime / 'stop.approved').is_file() and (runtime / 'stop.approved').read_text().strip()==run:
                    outcome='COMPLETE' if last.get('state')=='COMPLETE' else ('INCOMPLETE' if last.get('triggered') else 'CANCELLED')
                    reason=last.get('reason','ground stop approved');break
                raise RuntimeError('controller exited unexpectedly; no automatic restart')
            if cancel:
                (runtime / 'request.stop').touch();cancel=False
            if now-last_check>=1:
                try:
                    progress=b.bag_progress(path / 'bag');free=shutil.disk_usage(path).free
                    if progress>last_count:last_growth=now
                    last_count=progress
                except (OSError, sqlite3.Error) as exc:
                    progress=last_count;free=0;state['last_error']=str(exc)
                bad=children['recorder'].poll() is not None or now-last_growth>2 or free<(128*1024**2 if last.get('triggered') else b.MIN_FREE)
                logger_bad=logger_bad or bad
                if logger_bad:(runtime / 'recorder.ready').unlink(missing_ok=True)
                if free<128*1024**2:b.end_owned(children['recorder'])
                state.update(recording='FAILED' if logger_bad else 'RECORDING', free_bytes=free, bag_messages=progress, records=len(store.entries()))
                last_check=now
            if not logger_bad:b.atomic_json(runtime / 'recorder.ready', now)
            if now-last_summary>=5:
                position=b.read_json('/run/kr260-position/state.json')
                print(json.dumps(dict(kind='EGO_5S_SUMMARY',state=last.get('state'),
                    map_age_ms=last.get('map_age_ms'),planner_ms=last.get('planner_ms'),
                    control_max_gap_s=last.get('max_control_gap_s'),trajectory_id=last.get('trajectory_id'),
                    localization_state=position.get('state'),localization_windows=position.get('metrics'),
                    recording='FAILED' if logger_bad else 'RECORDING')),flush=True)
                last_summary=now
            if children['planner'].poll() is not None:state['last_error']='planner exited; controller must brake/latch, no restart'
            if current:
                update(state=current['state'], reason=current['reason'], controller=current, children={k:dict(pid=p.pid,exit_code=p.poll()) for k,p in children.items()})
                if current.get('safe_to_stop') and current['state'] in TERMINAL:
                    ground_since=ground_since or now
                    if now-ground_since>=5:(runtime / 'request.stop').touch()
                else:ground_since=None
            else:update(state='PREPARING' if now-deadline<10 else 'DEGRADED',reason='waiting for fresh owned controller status; never auto restart')
            time.sleep(.2)
        # Always keep the fresh ground tail, including cancelled/not-flown sessions.
        tail=(ground_since if ground_since is not None else time.monotonic())+5
        while time.monotonic()<tail:update(state='FINALIZING',reason='five-second ground recording tail');time.sleep(.2)
    except Exception as exc:
        reason=str(exc);update(state='FAILED',reason=reason);print(reason,file=sys.stderr,flush=True)
    finally:
        controller=children.get('controller')
        # Never convert supervisor/log IO failure into an airborne cleanup. Keep
        # owning the lock and children until fresh ground confirmation arrives.
        # This is not crash tolerance (power loss/process SIGKILL remains PX4's job).
        while controller and controller.poll() is None:
            current=b.controller_state(runtime,run)
            if (not execute) or (current and current.get('safe_to_stop')):
                break
            print('SUPERVISOR FAULT: awaiting fresh landed/disarmed state; NO controller kill/restart',file=sys.stderr,flush=True)
            if current and current.get('triggered') and current.get('nav_state')==14:
                try:(runtime / 'request.land').touch()
                except OSError:pass
            try:update(state='DEGRADED',reason=reason+'; awaiting ground, controller retained')
            except OSError:pass
            time.sleep(1)
        # Routine airborne faults never leave this loop; controller handles them without cleanup/restart.
        # A supervisor crash is not a validated in-flight failover. PX4 loss handling still applies.
        for key in ('precheck','cloud_on','controller','vision_bridge','planner'):b.end_owned(children.get(key))
        clean=b.end_owned(children.get('recorder'))
        if lease:
            proc,handle=b.start_owned([sys.executable,str(Path(__file__).resolve()),'_cloud-off','--workspace',str(workspace),'--runtime',str(runtime)], (path or runtime) / 'cloud_off.log')
            handles.append(handle)
            try:proc.wait(timeout=12)
            except subprocess.TimeoutExpired:b.end_owned(proc)
        for handle in handles:handle.close()
        if path:
            if logger_bad or not clean or not (path / 'bag/metadata.yaml').exists():outcome='INCOMPLETE';reason+='; recorder incomplete'
            store.close(path,outcome,reason,dict(last,ever_armed=ever_armed))
        update(state='COMPLETE' if outcome=='COMPLETE' else 'STOPPED' if outcome=='CANCELLED' else 'FAILED',reason=reason,outcome=outcome,recording='CLOSED' if clean else 'INCOMPLETE')
    return 0 if outcome in ('COMPLETE','CANCELLED') else 1


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['preview','start','status','logs','land','stop','_run','_cloud-on','_cloud-off','_selected-stage'])
    parser.add_argument('--workspace',type=Path,default=Path('/home/ubuntu/drone_ws'))
    parser.add_argument('--runtime',type=Path,default=Path('/run/kr260-ego'))
    parser.add_argument('--require-selection',action='store_true')
    a=parser.parse_args(argv)
    if a.command=='_selected-stage':
        bundle=selected_bundle(a.workspace,True)
        print(str(bundle['stage'])+'\n'+bundle['selection_sha256']);return 0
    if a.require_selection or Path(__file__).name==SELECTED_RUNTIME:
        selected_bundle(a.workspace,True)
        if a.command in ('_run','_cloud-on','_cloud-off') and not os.environ.get('KR260_EGO_SELECTION_SHA256'):
            raise RuntimeError('internal selected runtime requires launcher-pinned selection receipt')
    b=base_module(a.workspace)
    if a.command=='_run':return supervisor(b,a.workspace,a.runtime)
    if a.command in ('_cloud-on','_cloud-off'):
        run=os.environ.get('INVOCATION_ID');marker=a.runtime / 'cloud.lease'
        if not run or b.read_json(a.runtime / 'supervisor.json').get('run_id')!=run:raise RuntimeError('owned service required')
        if a.command=='_cloud-on':
            # The bag may have taken seconds to commit; reject arming/mode changes
            # since the earlier check before enabling any extra localization load.
            b.ground_precheck(timeout=3)
            planning_parameter(True,require_old=False);marker.write_text(run)
        elif marker.is_file() and marker.read_text().strip()==run:
            planning_parameter(False,require_old=True);marker.unlink()
        return 0
    if a.command=='logs':return subprocess.call(['journalctl','-u',UNIT,'-n','80','--no-pager'])
    if a.command in ('start','preview'):
        unit=info()
        if unit.get('ActiveState')=='deactivating':raise RuntimeError('service stopping; do not queue restart')
        if unit.get('ActiveState') not in ('active','activating'):
            execute=a.command=='start';validate_config(a.workspace / 'config/kr260_ego.yaml',execute)
            # Reject conflicting control/preview before starting or touching localization.
            with b.exclusive_lock(a.workspace / '.kr260_offboard.lock'):
                if execute:request_localization(a.workspace)
                request=a.workspace / 'config/kr260_ego_request.json';b.atomic_json(request,dict(execute=execute))
            subprocess.run(['sudo','systemctl','start',UNIT],check=True)
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            d=display(b,a.runtime);print(d.get('state'),d.get('reason',''),flush=True)
            if d.get('state') in ('READY_TO_ARM','WAIT_OFFBOARD_ON_GROUND','PREVIEW','FAILED','STOPPED',*TERMINAL) or d.get('controller',{}).get('triggered'):
                print(json.dumps(d,indent=2,ensure_ascii=False));print_operator_notice(d);return int(d.get('state') in ('FAILED','STOPPED',*TERMINAL))
            time.sleep(.5)
        return 2
    d=display(b,a.runtime)
    if a.command=='status':print(json.dumps(d,indent=2,ensure_ascii=False));print_operator_notice(d);return 0
    unit=info()
    if a.command=='stop' and unit.get('ActiveState') not in ('active','activating','deactivating'):
        print('No owned EGO service running; no process signalled.');return 0
    c=b.controller_state(a.runtime,unit.get('InvocationID',''))
    if not c:raise RuntimeError('fresh owned controller status missing; cannot safely stop/restart')
    if a.command=='land':
        if not c.get('triggered') or c.get('nav_state')!=14 or c.get('state') in TERMINAL:raise RuntimeError('EGO does not own active Offboard control')
        (a.runtime / 'request.land').touch();print('Land requested, NOT confirmed.');return 0
    if not c.get('safe_to_stop'):raise RuntimeError('stop refused: require fresh landed/disarmed telemetry')
    (a.runtime / 'request.stop').touch()
    deadline=time.monotonic()+35
    while time.monotonic()<deadline:
        if info().get('ActiveState') not in ('active','activating','deactivating'):print(json.dumps(display(b,a.runtime),indent=2));return 0
        time.sleep(.2)
    raise RuntimeError('stop not confirmed; no forced kill')


if __name__=='__main__':
    try:sys.exit(main())
    except Exception as exc:print('ERROR: '+str(exc),file=sys.stderr);sys.exit(1)
