import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)


def generate_launch_description():
    ld = LaunchDescription()

    crawler_bot_dir = get_package_share_directory('crawler_bot')
    params_file = LaunchConfiguration('params_file')

    declare_param_file = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(crawler_bot_dir, 'config', 'robot.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

    realsense_node = Node(
            package='crawler_bot',
            executable='realsense_run',
            parameters=[params_file],
            name='realsense')

    crawler_bot_node = Node(
            package='crawler_bot',
            executable='crawler_bot_run',
            parameters=[params_file],
            name='crawler_bot')

    ld.add_action(declare_param_file)
    ld.add_action(realsense_node)
    ld.add_action(crawler_bot_node)

    return ld
