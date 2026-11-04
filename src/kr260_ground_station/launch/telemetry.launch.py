"""Only read-only telemetry; never includes EGO/position/vehicle controllers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default=Path(get_package_share_directory('kr260_ground_station'))/'config/sender.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(default)),
        Node(package='kr260_ground_station',executable='telemetry_sender',
             name='kr260_ground_station_sender',parameters=[LaunchConfiguration('config')],output='screen')])
