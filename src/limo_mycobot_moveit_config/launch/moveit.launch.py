"""MoveIt 2 (move_group + rviz MotionPlanning) for one LIMO + myCobot 280 already running.

Start the simulation first with the arm, e.g.
  ros2 launch limo_simulation gazebo_models_diff.launch.py with_arm:=true
then
  ros2 launch limo_mycobot_moveit_config moveit.launch.py robot_name:=robot_0
(or sim_moveit.launch.py for both). move_group runs in the robot namespace, so it reads
/<robot_name>/joint_states and executes on /<robot_name>/arm_controller/follow_joint_trajectory.
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from limo_simulation import launch_utils as lu

PACKAGE = 'limo_mycobot_moveit_config'


def config_path(name):
    return os.path.join(get_package_share_directory(PACKAGE), 'config', name)


def load_yaml(name):
    with open(config_path(name)) as f:
        return yaml.safe_load(f)


def moveit_nodes(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    ompl = load_yaml('ompl_planning.yaml')
    description = {
        # same URDF the simulation spawned (prefixed links, arm controllers)
        'robot_description': lu.robot_description(robot_name, with_arm=True),
        'robot_description_semantic': xacro.process_file(
            config_path('limo_mycobot.srdf.xacro'),
            mappings={'robot_name': robot_name}).toxml(),
        'robot_description_kinematics': load_yaml('kinematics.yaml'),
        'robot_description_planning': load_yaml('joint_limits.yaml'),
    }
    move_group_params = {
        **description,
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl,
        **load_yaml('moveit_controllers.yaml'),
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.05,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
        'use_sim_time': True,
    }
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=robot_name,
        output='screen',
        parameters=[move_group_params],
    )

    with open(config_path('moveit.rviz')) as f:
        rviz_text = f.read().replace('ROBOT_NAME', robot_name)
    rviz_file = os.path.join(lu.tmp_dir(), f'moveit_{robot_name}.rviz')
    with open(rviz_file, 'w') as f:
        f.write(rviz_text)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        namespace=robot_name,
        output='log',
        arguments=['-d', rviz_file],
        parameters=[{**description, 'planning_pipelines': ['ompl'], 'ompl': ompl,
                     'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot_0',
                              description='Robot whose arm MoveIt plans for (robot_0, ...)'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start rviz with the MotionPlanning panel'),
        OpaqueFunction(function=moveit_nodes),
    ])
