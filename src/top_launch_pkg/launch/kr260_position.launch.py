"""KR260 localization only. READY means startup passed, never flight clearance.

Use scripts/kr260_position.sh start for SSH-independent operation. Direct launch
is a foreground debugging entry and requires an already prepared PL UART.
The runtime owns and stages Agent, Livox, FAST-LIO and bridge processes; no EGO,
Offboard, RViz, vehicle commands, or bench teardown watchdog is included.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    package = 'top_launch_pkg'
    runtime = os.path.join(get_package_prefix(package), 'lib', package,
                           'kr260_position_runtime.py')
    config = os.path.join(get_package_share_directory(package), 'config', 'kr260')
    process = ExecuteProcess(
        cmd=['/usr/bin/python3', runtime, 'run', '--config-dir', config],
        name='kr260_position_runtime', output='screen',
        additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'ROS_DOMAIN_ID': '0'},
    )
    def exited(event, _context):
        if event.returncode not in (0, -2, -15):
            raise RuntimeError('position runtime failed; inspect status and component logs')
        return [EmitEvent(event=Shutdown(reason='position runtime exited; see state/logs'))]

    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=process, on_exit=exited)),
        process,
    ])
