"""Gazebo (LIMO + myCobot, single robot) + MoveIt 2 for robot_0."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', name))


def generate_launch_description():
    sim = IncludeLaunchDescription(
        launch_file('limo_simulation', 'gazebo_models_diff.launch.py'),
        launch_arguments={'n_robots': '1', 'with_arm': 'true', 'rviz': 'false',
                          'gui': LaunchConfiguration('gui'),
                          'seed': LaunchConfiguration('seed')}.items())
    moveit = IncludeLaunchDescription(
        launch_file('limo_mycobot_moveit_config', 'moveit.launch.py'),
        launch_arguments={'robot_name': 'robot_0'}.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo client'),
        DeclareLaunchArgument('moveit_delay', default_value='8.0',
                              description='Seconds before starting MoveIt'),
        DeclareLaunchArgument('seed', default_value='0', description='Spawn pose seed'),
        sim,
        # move_group waits for joint states anyway; the delay just keeps the logs readable
        TimerAction(period=LaunchConfiguration('moveit_delay'), actions=[moveit]),
    ])
