import launch
import math
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess


def generate_launch_description():


    # get the goal pose using the joystick
    joy_node = Node(
            package="joy",
            executable="joy_node"
            )

    # convert joystick to goal pose
    joy_to_goal = Node(
            package="gatekeeper",
            executable="joystick_to_goal_pose"
            )

    # use mpc to construct a trajectory out of the setpoint
    mpc_node = Node(
            package="gatekeeper",
            executable="mpc", 
            remappings=[
                ("mpc_trajectory", "nominal_trajectory")
                ]
            )

    # now run gatekeeper node to check that the trajectory can be safe
    #gatekeeper_node = ExecuteProcess(
    #        cmd = [ 
    #            'julia', '--project', 'node-gatekeeper.jl'
    #            
    #            ],
    #        cwd = "/root/colcon_ws/src/gatekeeper/gatekeeper",
    #        output="both",

    #        )

    setpoint_pub = Node(
            package="dasc_ros_utils",
            executable="setpointPublisher",
            parameters=[
                {"trajectory_topic": "committed_trajectory"}
                ]
            )


    return launch.LaunchDescription([
        joy_node, 
        joy_to_goal,
        # mpc_node,
        # gatekeeper_node,
        setpoint_pub
        ]
        )
