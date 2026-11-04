#!/usr/bin/env python3
"""Read-only PX4 schema and recorded CDR checks. Never publishes flight inputs."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sqlite3

PINNED_COMMIT = '8ff7255ba764d343c0a11136eb489a6a5d5d0784'
TOPIC_TYPES = {
    '/fmu/out/vehicle_local_position_v1': 'VehicleLocalPosition',
    '/fmu/out/vehicle_status_v3': 'VehicleStatus',
    '/fmu/out/vehicle_land_detected': 'VehicleLandDetected',
    '/fmu/out/failsafe_flags': 'FailsafeFlags',
    '/fmu/out/vehicle_odometry': 'VehicleOdometry',
    '/fmu/in/vehicle_visual_odometry': 'VehicleOdometry',
    '/fmu/out/vehicle_command_ack_v1': 'VehicleCommandAck',
    '/fmu/in/offboard_control_mode': 'OffboardControlMode',
    '/fmu/in/trajectory_setpoint': 'TrajectorySetpoint',
    '/fmu/in/vehicle_command': 'VehicleCommand',
}
CRITICAL_FLAGS = (
    'angular_velocity_invalid', 'attitude_invalid', 'local_altitude_invalid',
    'local_position_invalid', 'local_velocity_invalid', 'manual_control_signal_lost',
    'battery_unhealthy', 'battery_low_remaining_time', 'fd_critical_failure',
    'fd_esc_arming_failure', 'fd_imbalanced_prop', 'fd_motor_failure', 'geofence_breached',
)


def normalize_type(value):
    base, sep, array = value.partition('[')
    base = {'float': 'float32', 'double': 'float64', 'boolean': 'bool'}.get(base, base)
    return base + (sep + array if sep else '')


def fixed_cdr_size(fields):
    sizes = {'bool': 1, 'int8': 1, 'uint8': 1, 'int16': 2, 'uint16': 2,
             'int32': 4, 'uint32': 4, 'int64': 8, 'uint64': 8, 'float32': 4, 'float64': 8}
    offset = 0
    for typename, _ in fields:
        base, sep, array = normalize_type(typename).partition('[')
        if base not in sizes:
            raise RuntimeError('unsupported non-fixed interface type: ' + typename)
        count = int(array.removesuffix(']')) if sep else 1
        size = sizes[base]
        offset = (offset + size - 1) // size * size + size * count
    return 4 + offset  # ROS 2 CDR encapsulation, alignment relative to payload.


def verify_interfaces(manifest_path, message_module=None, serializer=None):
    manifest = json.loads(Path(manifest_path).read_text(encoding='utf-8'))
    if manifest.get('firmware', {}).get('git_commit') != PINNED_COMMIT:
        raise RuntimeError('unknown PX4 firmware/interface lock; no implicit fallback')
    if message_module is None:
        message_module = importlib.import_module('px4_msgs.msg')
    if serializer is None:
        from rclpy.serialization import serialize_message
        serializer = serialize_message
    checks = {}
    for name, definition in manifest['messages'].items():
        typ = getattr(message_module, name)
        actual = [[normalize_type(value), key] for key, value in typ.get_fields_and_field_types().items()]
        expected = definition['wire_fields']
        if actual != expected:
            raise RuntimeError(f'PX4_INTERFACE_MISMATCH {name}: generated field order/types differ from firmware {PINNED_COMMIT}')
        if getattr(typ, 'MESSAGE_VERSION', 0) != definition['message_version']:
            raise RuntimeError(f'PX4_INTERFACE_MISMATCH {name}: MESSAGE_VERSION differs')
        expected_size = fixed_cdr_size(expected)
        actual_size = len(serializer(typ()))
        # DDS implementations may retain 0-3 terminal CDR alignment bytes.
        # This is validation only: received bytes are never changed.
        if actual_size not in (expected_size, (expected_size+3)//4*4):
            raise RuntimeError(f'PX4_TYPESUPPORT_MISMATCH {name}: serialized size {actual_size}, expected {expected_size}')
        checks[name] = {'cdr_size': actual_size, 'message_version': definition['message_version'],
                        'wire_sha256': hashlib.sha256(json.dumps(expected, separators=(',', ':')).encode()).hexdigest()}
    if set(checks) != {'VehicleOdometry', 'TimesyncStatus', 'VehicleLocalPosition', 'VehicleStatus',
                       'VehicleLandDetected', 'FailsafeFlags', 'OffboardControlMode', 'TrajectorySetpoint',
                       'VehicleCommand', 'VehicleCommandAck'}:
        raise RuntimeError('incomplete PX4 interface lock; all ten interfaces are required')
    return {'firmware_commit': PINNED_COMMIT, 'messages': checks}


def validate_bag(bag, limit=None):
    """Decode actual stored CDR without padding/trimming/reinterpreting any bytes."""
    from rclpy.serialization import deserialize_message, serialize_message
    messages = importlib.import_module('px4_msgs.msg')
    result = {}
    for path in sorted(Path(bag).glob('*.db3')):
        with sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True, timeout=2) as conn:
            for topic, type_name in TOPIC_TYPES.items():
                default_size = len(serialize_message(getattr(messages, type_name)()))
                query = ('SELECT messages.timestamp,messages.data FROM messages JOIN topics '
                         'ON messages.topic_id=topics.id WHERE topics.name=? ORDER BY messages.id DESC')
                args = (topic,)
                if limit is not None:
                    query += ' LIMIT ?'
                    args += (limit,)
                for received, payload in conn.execute(query, args):
                    try:
                        if len(payload) not in (default_size, (default_size+3)//4*4):
                            raise ValueError(f'CDR length {len(payload)} incompatible with fixed layout {default_size}')
                        msg = deserialize_message(bytes(payload), getattr(messages, type_name))
                    except Exception as exc:
                        raise RuntimeError(f'PX4_CDR_DECODE_ERROR {topic}: {exc}') from exc
                    entry = result.setdefault(topic, {'decoded': 0, 'min_age_ms': None, 'max_age_ms': None})
                    entry['decoded'] += 1
                    if hasattr(msg, 'timestamp'):
                        age = (received/1000-msg.timestamp)/1000
                        entry['min_age_ms'] = age if entry['min_age_ms'] is None else min(entry['min_age_ms'], age)
                        entry['max_age_ms'] = age if entry['max_age_ms'] is None else max(entry['max_age_ms'], age)
                    if type_name == 'FailsafeFlags':
                        entry.setdefault('observed_true_flags', set()).update(
                            k for k in msg.get_fields_and_field_types() if getattr(msg, k) is True)
    for entry in result.values():
        if 'observed_true_flags' in entry:
            entry['observed_true_flags'] = sorted(entry['observed_true_flags'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    report = verify_interfaces(args.manifest)
    if args.bag:
        report['bag'] = validate_bag(args.bag)
    text = json.dumps(report, indent=2, allow_nan=False)
    if args.report:
        args.report.write_text(text + '\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
