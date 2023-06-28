import numpy as np
import gatekeeper.mpc_planner_OSQP as mpc_planner
import time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs

import math

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Vector3
from gatekeeper_msgs.msg import DITrajectory
from decomp_ros_msgs.msg import Polyhedron, PolyhedronStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration


def toPoint(x,y,z):
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p

def toVector3(x,y,z):
    p = Vector3()
    p.x = x
    p.y = y
    p.z = z
    return p



class MPCPlanner(Node):

    def __init__(self):
        super().__init__("mpc_planner")

        ## parameters
        self.frame_id = "vicon/world/NED"
        self.recompute_period_s = 0.25 # seconds

        ## subscribers
        self.subscription = self.create_subscription(
                PoseStamped,
                "goal_pose",
                self.goal_callback, 10)

        self.sfc_sub = self.create_subscription(
                PolyhedronStamped,
                "sfc",
                self.sfc_callback, 10)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        ## publishers
        self.di_traj_pub = self.create_publisher(
                DITrajectory,
                "mpc_trajectory", 10
                )
        self.di_traj_viz_pub = self.create_publisher(
                PoseArray,
                "mpc_trajectory_viz", 10
                )

        self.mpc_goal_pose_pub_ = self.create_publisher(
                PoseStamped, 
                "mpc_goal_viz", 10
                )

        ## initialize mpc object
        self.mpc = mpc_planner.MPCPlanner(N = 20, DT=0.25, max_accel=10.0)
        self.get_logger().info("Initialize the MPC planner!")

        # initialize the self.state
        self.state = PoseStamped()
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.state.header.frame_id = self.frame_id

        # initialize the goal
        self.goal_msg = PoseStamped()
        self.goal_msg.header.stamp = self.get_clock().now().to_msg()
        self.goal_msg.header.frame_id = self.frame_id 
        self.goal_msg.pose.position.x = 1.0;

        # start a timer
        timer = self.create_timer(self.recompute_period_s, self.solve_mpc)

    def goal_callback(self, msg):

        # transform the goal to the desired frame
        if self.transform_goal_pose(msg):



        p = msg.pose.position
        self.get_logger().info(f"I got a goal position of {p.x}, {p.y}, {p.z} in frame {msg.header.frame_id}")
        
        self.goal_msg = msg
        
        self.goal_msg.pose.position.z = 0.5
        self.get_logger().info(f"Changing z to {self.goal_msg.pose.position.z}")

    def goal_callback(self, msg):
        
        # try to get the transform to the desired frame
        try:
            trans = self.tf_buffer.lookup_transform(
                self.frame_id,
                msg.header.frame_id,
                Time(seconds=0)
                ) 
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {msg.header.frame_id} to {self.frame_id}: {ex}')
            return 

        self.goal_pose = tf2_geometry_msgs.do_transform_pose_stamped(msg, trans)

        # FOR DEBUGGING!
        # publish the goal pose
        self.mpc_goal_pose_pub_.publish(self.goal_pose)

        return True

        

    def state_callback(self, msg):

        self.state = msg

    def get_rot_and_trans(trans): ## Transform msg
        q = trans.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])

        v = trans.translation
        t = np.array([v.x, v.y, v.z])

        return R, t

    def sfc_callback(self, msg):
        
        # try to get the transform to the desired frame
        try:
            trans = self.tf_buffer.lookup_transform(
                self.frame_id,
                msg.header.frame_id,
                Time(seconds=0) # TODO: FIX TIME!
                )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {msg.header.frame_id} to {self.frame_id}: {ex}')
            return 

        ## get rotation matrix and translation vector
        R, t = self.get_rot_and_t(trans) # we might need to get the other direction

        N_sfc = len(msg.poly.a)
        
        A = np.zeros([N_sfc, 3])
        for i in range(N_sfc):
            A[i, 0] = msg.poly.a[i].x
            A[i, 1] = msg.poly.a[i].y
            A[i, 2] = msg.poly.a[i].z

        # transform to the px4_world frame
        A_world = A @ R
        b_world = msg.poly.b + A @ R @ t

        self.mpc.set_safe_polyhedron(A_world, b_world)

        return



    def solve_mpc(self):

        # set the target location
        gp = self.goal_pose.pose.position
        self.mpc.set_target_location([gp.x, gp.y, gp.z])

        # set the current state

        # set the sfc

        # solve
        res = self.mpc.solve()

        if res:
            print("success")
            self.publish_trajectory()
            return;

        else:
            # print a warning
            print("ERROR!")
            return;


    def publish_trajectory(self):
        xs = self.mpc.sol_x
        us = self.mpc.sol_u

        di_msg = DITrajectory()
        di_msg.header.frame_id = self.frame_id 
        di_msg.header.stamp = self.get_clock().now().to_msg()

        di_msg.dt = self.mpc.DT

        for x in xs:
            p = toPoint(x[0],x[2],x[4])
            v = toVector3(x[1], x[3], x[5])
            di_msg.pos.append(p)
            di_msg.vel.append(v)
        for u in us:
            a = toVector3(u[0], u[1], u[2])
            di_msg.acc.append(a)

        # todo: yaw

        self.di_traj_pub.publish(di_msg)

        ## now publish the PoseArray msg
        pose_array_msg = PoseArray()
        pose_array_msg.header = di_msg.header
        for x in xs:
            p = toPoint(x[0],x[2],x[4])
            pose = Pose()
            pose.position = p
            pose_array_msg.poses.append(pose)

        self.di_traj_viz_pub.publish(pose_array_msg)


def main(args=None):

    rclpy.init(args=args)

    node = MPCPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

