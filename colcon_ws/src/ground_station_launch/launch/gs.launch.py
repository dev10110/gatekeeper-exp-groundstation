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

    # plot the lab polygon
    lab_poly = Node(
            package="dasc_ros_utils",
            executable="publish_lab_poly.py"
            )

    # video viewer
    video_viewer = Node(
            package="video_view",
            executable="video_viewer_node",
            parameters = [
                {"image_topic": "/camera/color/image_raw"},
                {"image_transport": "theora"},
                {"rotate": "rotate_180"},
                {"qos": "SENSOR_DATA"},
                ]
            )

    # esdf splitter
    esdf_splitter = Node(
            package="esdf_splitter",
            executable="esdf_splitter"
            )


    # decompros viz node
    decomp_ros_nvblox_viz = Node(
           namespace="nvblox_node",
           package="decomp_ros",
           executable="vizPoly_node",
           parameters = [
               {"color_a": 0.4},
               {"line_w": 1.0}
               ]
           )

    # decomp_ros_viz = Node(
    #         package="decomp_ros",
    #         namespace="camera",
    #         executable="vizPoly_node"
    #         )

    return LaunchDescription([
        rviz0,
        # rviz1,
        rviz2,
        # vicon,
        # vicon_px4_bridge_node,
        # vicon_world_NED,
        decomp_ros_nvblox_viz,
        lab_poly,
        esdf_splitter,
        # video_viewer,
        ])


