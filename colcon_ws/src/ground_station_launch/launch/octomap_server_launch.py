#!/usr/bin/env python

import os.path as osp

from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import \
    PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    params = {'resolution': 0.1,
              'frame_id': 'vicon/world',
              'base_frame_id': 'base_footprint',
              'use_height_map': False,
              'colored_map': False,
              'color_factor': 0.8,
              'filter_ground': False,
              'filter_speckles': False,
              'compress_map': True,
              'incremental_2D_projection': False,
              'occupancy_min_z': -0.25,
              'occupancy_max_z': 2.5,
              'sensor_model/max_range': 5.0,
              'sensor_model/hit': 0.7,
              'sensor_model/miss': 0.4,
              'sensor_model/min': 0.12,
              'sensor_model/max': 0.97,
              'color/r': 0.0,
              'color/g': 0.0,
              'color/b': 1.0,
              'color/a': 1.0,
              'color_free/r': 0.0,
              'color_free/g': 0.0,
              'color_free/b': 1.0,
              'color_free/a': 1.0,
              'publish_free_space': False,
    }
    
    remap = [('cloud_in', '/camera/depth/color/points')]
    
    node = Node(package='octomap_server',
                 executable='octomap_server_node',
                 output='screen',
                 remappings=remap,
                 parameters=[params])
    
    return LaunchDescription([node])
