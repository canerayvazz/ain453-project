#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
_GAZEBO_SYSTEM_RESOURCE = '/usr/share/gazebo-11'

def generate_launch_description():
    pkg_share = FindPackageShare('pf_localization')
    pkg_share_dir = get_package_share_directory('pf_localization')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'room_tags.world'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'duckiebot.xacro'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]), launch_arguments={'world': world_file, 'verbose': 'true'}.items())
    robot_state_publisher_node = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}])
    spawn_entity_node = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen', arguments=['-entity', 'duckiebot', '-topic', 'robot_description', '-package_to_model', '-x', '2.5', '-y', '2.0', '-z', '0.05', '-Y', '0.0', '-timeout', '60.0', '-unpause'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'pf_loc.rviz'])
    pf_params_file = PathJoinSubstitution([pkg_share, 'config', 'pf_params.yaml'])
    pf_node = Node(package='pf_localization', executable='pf_node', name='pf_node', output='screen', parameters=[pf_params_file, {'use_sim_time': use_sim_time}])
    rviz_node = Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', rviz_config], parameters=[{'use_sim_time': use_sim_time}])
    delayed_spawn = TimerAction(period=8.0, actions=[spawn_entity_node])
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock from Gazebo'), AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=PathJoinSubstitution([pkg_share, '..'])), AppendEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=_GAZEBO_SYSTEM_RESOURCE), AppendEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=pkg_share_dir), gazebo, robot_state_publisher_node, delayed_spawn, pf_node, rviz_node])
