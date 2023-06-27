import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


import math


def generate_launch_description():
    
    robot_name = "px4_1"

    # launch rviz with the teleop panel configuration
    config_name = LaunchConfiguration('config_name', default='default.rviz')
    config_path = PathJoinSubstitution([get_package_share_directory(
        'ground_station_launch'), 'config', 'rviz', config_name])
    global_frame = LaunchConfiguration('global_frame', default='vicon/world')

    # Rviz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_path,       # set the config
                   '-f', global_frame],     # overwrite the global frame
        output='screen')

    # launch vicon
    vicon = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('vicon_receiver'), 'launch', 'client.launch.py')]))



    vicon_px4_bridge_node = Node(
            package='vicon_px4_bridge', executable='bridge', output='screen',
            parameters=[{'px4_name': robot_name, 'vicon_name': robot_name}]
        )
    # define some static tfs
    vicon_world_NED = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = [
                "0","0","0", f"{math.pi/2}", "0", f"{math.pi}", "/vicon/world/NED", "/vicon/world"]
            )


    return LaunchDescription([
        rviz,
        vicon,
        vicon_px4_bridge_node,
        vicon_world_NED,
        ])


