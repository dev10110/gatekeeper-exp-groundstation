# subscribe to new path messages
# subscribe to new state messages
# determine where we are currently
# assign times to each pose in path# 
# interpolate path -> trajectory for the next T seconds

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist, Accel
from dasc_msgs.msg import DITrajectory


import scipy as sp
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp


def poseDist(pose1 , pose2):

    p1 = pose1.position
    p2 = pose2.position

    lin_dist = np.linalg.norm([p1.x-p2.x, p1.y-p2.y, p1.z-p2.z])

    q1 = Rot.from_quat([pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w])
    q2 = Rot.from_quat([pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w])

    qd = q2.inv() * q1
    ang_dist = abs(qd.magnitude())

    return lin_dist, ang_dist

class PathToTrajectory(Node):

    def __init__(self):
        super().__init__('pathToTrajectory')

        print("Started!!")

        ## subscribers
        self.create_subscription(Path, 'plan', self.path_callback, 10)


        ## publishers
        self.pub_traj_msg = self.create_publisher(DITrajectory, "trajectory", 10)
        self.pub_traj_viz = self.create_publisher(PoseArray, "trajectory/viz", 10)

        ## parameters
        self.lin_vel = 1.0; # m/s
        self.ang_vel = np.pi/2 # rad/s
        self.dt = 0.2

        self.override_z = 1.0


    def path_callback(self, path_msg):
        
        N = len(path_msg.poses)

        print(f"Got path with {N} poses")

        if N <= 1:
            print("LESS THAN 2 POSES IN THE PATH MESSAGE!")
            return

        # check the frames
        frame = path_msg.header.frame_id
        for i in range(N):
            if path_msg.poses[i].header.frame_id != frame:
                print("THE POSES ARE NOT ALL IN THE SAME FRAME!")
                return

        if self.override_z is not None:
            for i in range(N):
                path_msg.poses[i].pose.position.z = self.override_z

        times = [0]
        for i in range(N-1):
            # get the next pose
            pose = path_msg.poses[i].pose
            next_pose = path_msg.poses[i+1].pose

            # get the linear distance
            lin_dist, ang_dist = poseDist(pose, next_pose)

            # get the times
            tau = max(lin_dist / self.lin_vel, ang_dist / self.ang_vel)
            tau = max(tau, 0.001) # make sure there is atleast a non-zero interpolation

            # push to array
            times.append(times[-1] + tau)

        ## interpolate
        pos = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path_msg.poses])
        Rs = [Rot.from_quat([p.pose.orientation.x, p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]) for p in path_msg.poses]
        Rs = Rot.concatenate(Rs)

        cs = CubicSpline(times, pos)
        slerp = Slerp(times, Rs)

        interp_ts = np.arange(start=0, stop=times[-1], step=self.dt)
        interp_pos = cs(interp_ts)
        interp_vel = cs(interp_ts, 1)
        interp_acc = cs(interp_ts, 2)
        interp_Rs = slerp(interp_ts)

        ## now we can construct the traj message
        traj_msg = DITrajectory()
        traj_msg.header = path_msg.header
        traj_msg.dt = self.dt

        for i in range(len(interp_ts)):
            pose = Pose()
            pose.position.x = interp_pos[i, 0]
            pose.position.y = interp_pos[i, 1]
            pose.position.z = interp_pos[i, 2]
            q = interp_Rs[i].as_quat()
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            traj_msg.poses.append(pose)

            twist = Twist()
            twist.linear.x = interp_vel[i, 0]
            twist.linear.y = interp_vel[i, 1]
            twist.linear.z = interp_vel[i, 2]
            # TODO(dev): add in the yawspeed
            traj_msg.twists.append(twist)

            acc = Accel()
            acc.linear.x = interp_acc[i, 0]
            acc.linear.y = interp_acc[i, 1]
            acc.linear.z = interp_acc[i, 2]
            traj_msg.accelerations.append(acc)


        ## now publish the traj msg
        self.pub_traj_msg.publish(traj_msg)
        print(f"published new trajectory with {len(traj_msg.poses)} poses")


        ## now publish the viz
        viz_msg = PoseArray()
        viz_msg.header = traj_msg.header
        viz_msg.poses = traj_msg.poses
        self.pub_traj_viz.publish(viz_msg)








def main(args=None):
    rclpy.init(args=args)

    node = PathToTrajectory()

    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
