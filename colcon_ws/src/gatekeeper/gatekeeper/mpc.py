import numpy as np
import gatekeeper.mpc_planner_OSQP as mpc_planner
import time

from transforms3d.taitbryan import euler2quat

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs

import math

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Vector3, Twist, Accel
from dasc_msgs.msg import DITrajectory
from decomp_ros_msgs.msg import Polyhedron, PolyhedronStamped
from px4_msgs.msg import VehicleLocalPosition
from builtin_interfaces.msg import Time as HeaderTime

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy



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
        self.frame_id = "vicon/world"
        self.recompute_period_s = 1.0# seconds

        ## subscribers
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
                PoseStamped,
                "goal_pose",
                self.goal_callback, 10)

        self.create_subscription(
                PolyhedronStamped,
                "sfc",
                self.sfc_callback, 10)

        self.create_subscription(
                VehicleLocalPosition, 
                "px4_1/fmu/out/vehicle_local_position", self.state_callback, sensor_qos)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        ## publishers
        self.di_traj_pub = self.create_publisher(
                DITrajectory,
                "mpc_trajectory", 10
                )
        self.di_traj_viz_pub = self.create_publisher(
                PoseArray,
                "mpc_trajectory/viz", 10)

        self.mpc_goal_pose_pub_ = self.create_publisher(
                PoseStamped, 
                "mpc_goal_viz", sensor_qos)

        ## initialize mpc object
        self.mpc = mpc_planner.MPCPlanner(N = 100, DT=0.01, max_accel=1000.0)
        self.get_logger().info("Initialize the MPC planner!")

        # initialize the self.state
        self.state = None # VehicleLocalPosition() # assumed to be in NED coordinates

        # initialize the goal
        self.goal_pose = None # PoseStamped()

        # start a timer
        timer = self.create_timer(self.recompute_period_s, self.solve_mpc)
        
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

        # also trigger a replanning
        # self.solve_mpc()

        return True

    def state_callback(self, msg):
        
        if not msg.xy_valid:
            return;
        if not msg.z_valid:
            return;
        if not msg.v_xy_valid:
            return;
        if not msg.v_z_valid:
            return;

        self.state = msg

        self.state_callback_timestamp_us = msg.timestamp_sample; # self.get_clock().now()

    def get_rot_and_trans(trans): ## Transform msg
        q = trans.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])

        v = trans.translation
        t = np.array([v.x, v.y, v.z])

        return R, t

    def sfc_callback(self, msg):

        return; # TODO: FIX!
        
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
        # R, t = self.get_rot_and_t(trans) # we might need to get the other direction

        # N_sfc = len(msg.poly.a)
        # 
        # A = np.zeros([N_sfc, 3])
        # for i in range(N_sfc):
        #     A[i, 0] = msg.poly.a[i].x
        #     A[i, 1] = msg.poly.a[i].y
        #     A[i, 2] = msg.poly.a[i].z

        # # transform to the px4_world frame
        # A_world = A @ R
        # b_world = msg.poly.b + A @ R @ t

        # self.mpc.set_safe_polyhedron(A_world, b_world)

        return



    def solve_mpc(self):
        print("solving mpc")
        
        if self.goal_pose is None:
            self.get_logger().info("goal pose is not initialized")
            return
        if self.state is None:
            self.get_logger().info("state is unknown")
            return 

        start_time = time.time()

        # set the target location
        gp = self.goal_pose.pose.position
        self.mpc.set_target_location([gp.x, gp.y, gp.z])

        # set the current state
        self.mpc.set_state(np.array([
                self.state.y,
                self.state.vy,
                self.state.x,
                self.state.vx,
                -self.state.z,
                -self.state.vz])) # DOES THE NED -> ENU CONVERSION HERE

        # set the sfc

        # solve
        res = self.mpc.solve()
        end_time = time.time()
        self.get_logger().info(f"MPC solve time: {(end_time - start_time)*1000:.1f}ms")

        if res:
            self.publish_trajectory()
        else:
            # print a warning
            self.get_logger().info("ERROR!")
        
        exit()




    def publish_trajectory(self):
        xs = self.mpc.sol_x
        us = self.mpc.sol_u

        di_msg = DITrajectory()
        di_msg.header.frame_id = self.frame_id 
        time_s =  self.state_callback_timestamp_us // (10 ** 6)
        time_ns =  (self.state_callback_timestamp_us  - time_s * 10**6) * (10**3)
        di_msg.header.stamp = HeaderTime(sec=time_s, nanosec=time_ns)

        di_msg.dt = self.mpc.DT

        #   # create the yaws from the solution
        #   yaws = [];
        #   for i in range(self.mpc.N):
        #       yaws.append(np.pi/2)
        #       continue

        #       vx = xs[i][1]
        #       vy = xs[i][3]
        #       eps = 0.05
        #       if (vx**2 < eps and vy**2 < eps):
        #           if len(yaws) == 0:
        #               ux = us[i][0]
        #               uy = us[i][1]
        #               if (ux**2 < eps and uy**2 < eps):
        #                 yaws.append(self.state.heading)
        #               else:
        #                   yaws.append(np.arctan2(uy, ux))
        #           else:
        #               yaws.append(yaws[-1]) # append the last direction
        #       else:
        #           yaws.append(np.arctan2(vy, vx))

        ## now construct the actual solution
        for i in range(self.mpc.N):
            if i == 0: # skip publishing the very first one
                continue 
            x = xs[i]
            u = us[i]

            pose = Pose()
            pose.position = toPoint(x[0],x[2],x[4])
            pose.orientation = self.goal_pose.pose.orientation # just use the goal yaw
            # q = euler2quat(yaws[i], 0,  0)
            # pose.orientation.x = q[1]
            # pose.orientation.y = q[2]
            # pose.orientation.z = q[3]
            # pose.orientation.w = q[0]
            di_msg.poses.append(pose)

            twist = Twist()
            twist.linear = toVector3(x[1],x[3],x[5])
            # TODO: fix angular
            di_msg.twists.append(twist)

            acc = Accel()
            acc.linear = toVector3(u[0], u[1], u[2])
            # TODO: fix angular
            di_msg.accelerations.append(acc)
        
        # ## now rotate to the goal pose
        # for i in range(5):
        #     x = xs[-1]

        #     pose = Pose()
        #     pose.position = toPoint(x[0],x[2],x[4])
        #     # pose.orientation = self.goal_pose.pose.orientation
        #     di_msg.poses.append(pose)

        #     twist = Twist()
        #     di_msg.twists.append(twist)

        #     acc = Accel()
        #     di_msg.accelerations.append(acc)

        self.di_traj_pub.publish(di_msg)

        ## now publish the PoseArray msg
        pose_array_msg = PoseArray()
        pose_array_msg.header = di_msg.header
        pose_array_msg.poses = di_msg.poses

        self.di_traj_viz_pub.publish(pose_array_msg)


def main(args=None):

    rclpy.init(args=args)

    node = MPCPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

