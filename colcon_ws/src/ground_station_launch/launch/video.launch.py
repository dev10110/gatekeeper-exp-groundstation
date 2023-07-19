

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
    
    video_node = Node(
            package='video_view',
            executable='video_viewer_node',
            parameters=[
                {"image_topic": "/camera/color/image_raw"},
                {"image_transport": "theora"},
                {"rotate": "rotate_180"},
                {"qos": "SENSOR_DATA"}
                ],
            output='screen',
        )



    return LaunchDescription([
        video_node
        ])


