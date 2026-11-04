"""Independent planning/controller launch. No driver, old adapter or traj_server.

Execution is intentionally not a launch argument: only the owned service may
construct the execution controller after recording and ground gates succeed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('kr260_ego_pkg'))
    return LaunchDescription([
        DeclareLaunchArgument('run_id', default_value='manual-preview'),
        DeclareLaunchArgument('controller_config', default_value=str(share/'config/kr260_ego.yaml')),
        DeclareLaunchArgument('runtime_dir', default_value='/tmp/kr260-ego-candidate-preview'),
        Node(package='ego_planner', executable='ego_planner_node', name='kr260_ego_planner',
             parameters=[str(share / 'config/planner.yaml'), {'kr260.run_id': LaunchConfiguration('run_id')}],
             remappings=[('odom_world', '/kr260/ego/preview_odometry'), ('grid_map/odom', '/kr260/ego/preview_odometry'),
                         ('grid_map/cloud', '/kr260/ego/cloud'), ('planning/bspline', '/kr260/ego/bspline'),
                         ('planning/data_display', '/kr260/ego/planning_data'),
                         ('planning/broadcast_bspline_from_planner', '/kr260/ego/broadcast_out'),
                         ('planning/broadcast_bspline_to_planner', '/kr260/ego/broadcast_in'),
                         ('grid_map/occupancy', '/kr260/ego/occupancy'),
                         ('grid_map/occupancy_inflate', '/kr260/ego/occupancy_inflate'),
                         ('grid_map/virtual_ceiling', '/kr260/ego/virtual_ceiling'),
                         *[(name, '/kr260/ego/'+name) for name in ('goal_point','global_list','init_list','optimal_list','a_star_list')],
                         ('/move_base_simple/goal', '/kr260/ego/goal')], output='screen'),
        Node(package='kr260_ego_pkg', executable='kr260_ego_node', name='kr260_ego_controller',
             parameters=[LaunchConfiguration('controller_config'), {'execute': False, 'run_id': LaunchConfiguration('run_id'),
                'runtime_dir':LaunchConfiguration('runtime_dir')}], output='screen')])
