"""Vision metadata and optional physical actuator. No flight commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_servo', default_value='false'),
        DeclareLaunchArgument('servo_dir', default_value=''),
        Node(package='kr260_ego_pkg', executable='vision_status_bridge.py', output='screen'),
        Node(package='kr260_ego_pkg', executable='servo_drop_executor.py', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_servo')),
             parameters=[{'servo_dir': LaunchConfiguration('servo_dir')}]),
    ])
