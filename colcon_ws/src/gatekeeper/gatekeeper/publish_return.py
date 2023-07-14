import numpy as np

import rclpy
from rclpy.node import Node


from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist, Accel
from dasc_msgs.msg import DITrajectory


class NominalTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__("nominal_trajectory_publisher")

        ## publishers
        self.pub_ = self.create_publisher(DITrajectory, "nominal_trajectory", 10)
        self.pub_viz_ = self.create_publisher(PoseArray, "nominal_trajectory/viz", 10)

        ## subscribers
        # should subscribe to current state, but for now I'll skip that step

        ## parameter
        self.dt = 0.1
        self.x0 = 1.25
        self.T = 2.0
        self.vx = -self.x0 / self.T


        self.create_path()


    def create_path(self):

        msg = DITrajectory()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vicon/world"
        msg.dt = self.dt

        z = 0.5 + np.random.rand()



        # create the reference trajectory to follow
        for t in np.arange(start=0.0, step=self.dt, stop=self.T):
            pose = Pose()
            pose.position.x = self.x0  + self.vx * t
            pose.position.z = z 

            twist = Twist()
            twist.linear.x = self.vx

            acc = Accel()

            msg.poses.append(pose)
            msg.twists.append(twist)
            msg.accelerations.append(acc)
        

        ## publish the msg
        self.publish(msg)

    def publish(self, msg):

        self.pub_.publish(msg)

        viz_msg = PoseArray()
        viz_msg.header = msg.header
        viz_msg.poses = msg.poses

        self.pub_viz_.publish(viz_msg)

        print("done")





def main(args = None):

    rclpy.init(args = args)

    node =  NominalTrajectoryPublisher()

    # rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()


