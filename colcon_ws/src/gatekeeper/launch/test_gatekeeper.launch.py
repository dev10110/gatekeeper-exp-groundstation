import launch
import math
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():


    # define some static tfs
    vicon_world_NED = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = [
                "0.0","0","0", f"{math.pi/2}", "0", f"{math.pi}", "/vicon/world/NED", "/vicon/world"]
            )

    setpoint_pub = Node(
            package="dasc_ros_utils",
            executable="setpointPublisher",
            parameters=[
                {"trajectory_topic": "nominal_trajectory"}
                ]
            )


    return launch.LaunchDescription([
        vicon_world_NED,
        setpoint_pub
        ]
        )
