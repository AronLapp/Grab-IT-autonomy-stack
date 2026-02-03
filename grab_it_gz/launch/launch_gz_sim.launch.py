#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'grab_it_gz'
    pkg_share = get_package_share_directory(pkg_name)

    world_path = os.path.join(pkg_share, 'model', 'worlds', 'empty_world.sdf')
    xacro_path = os.path.join(pkg_share, 'model', 'xacro', 'robot.urdf.xacro')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument('robot_name', default_value='grab_it'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    # Gazebo resource paths for model:// resolution
    set_gz_resource_path_pkg = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share)
    set_gz_resource_path_model = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'model'))

    # Xacro -> robot_description
    # IMPORTANT: add explicit spaces as separate tokens
    robot_description = Command([
        'xacro', ' ',
        xacro_path, ' ',
        'name:=', robot_name, ' ',
        'controllers_yaml:=', controllers_yaml_path
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r "{world_path}"'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z,
            '-Y', yaw,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
    )


    return LaunchDescription([
        *declare_args,
        set_gz_resource_path_pkg,
        set_gz_resource_path_model,
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=7.0, actions=[joint_state_broadcaster_spawner, arm_controller_spawner]),
    ])
