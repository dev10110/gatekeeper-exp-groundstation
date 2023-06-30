
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

    gatekeeper_mpc = Node(
            package='gatekeeper', executable='mpc', output='screen',
        )

    setpoint = Node(
            package="dasc_ros_utils", executable="setpointPublisher", output="screen",
            remappings=[
                ("trajectory", "mpc_trajectory")
                ]
            )


    return LaunchDescription([
        gatekeeper_mpc, 
        setpoint
        ])


