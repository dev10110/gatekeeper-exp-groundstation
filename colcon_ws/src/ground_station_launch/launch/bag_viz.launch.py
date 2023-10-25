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

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode



import math


def generate_launch_description():
    
    robot_name = "px4_1"

    # launch rviz with the teleop panel configuration
    config0_name = "teleop.rviz"
    config0_path = "/root/colcon_ws/src/ground_station_launch/config/" + config0_name
    config1_name = "groundstation.rviz"
    config1_path = "/root/colcon_ws/src/ground_station_launch/config/" + config1_name
    config2_name = "groundstation_mapping.rviz"
    config2_path = "/root/colcon_ws/src/ground_station_launch/config/" + config2_name
    global_frame = LaunchConfiguration('global_frame', default='vicon/world')

    # Rviz node
    rviz0 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config0_path,       # set the config
                   '-f', global_frame],     # overwrite the global frame
        output='screen')
    
    rviz1 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config1_path,       # set the config
                   '-f', global_frame],     # overwrite the global frame
        output='screen')
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config2_path,       # set the config
                   '-f', global_frame],     # overwrite the global frame
        output='screen')


    return LaunchDescription([
        rviz0,
        # rviz1,
        rviz2,
        ])


