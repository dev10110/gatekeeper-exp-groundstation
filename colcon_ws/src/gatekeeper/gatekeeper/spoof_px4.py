import numpy as np

import rclpy
from rclpy.node import Node


from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist, Accel
# from dasc_msgs.msg import DITrajectory
from px4_msgs.msg import VehicleLocalPosition


class SpoofPX4(Node):

    def __init__(self):
        super().__init__("spoof_px4")

        ## publishers
        self.pub_ = self.create_publisher(VehicleLocalPosition, "/px4_1/fmu/out/vehicle_local_position", 10)

        ## subscribers

        ## timers
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.callback)


    def to_px4_time(self, rosTime):

        s, ns = rosTime.seconds_nanoseconds()

        return int(s*1e6) + int(ns//1e3)


    def callback(self):

        msg = VehicleLocalPosition()


        now_ = self.get_clock().now()

        msg.timestamp = self.to_px4_time(now_);
        msg.timestamp_sample = self.to_px4_time(now_);

        msg.xy_valid = True
        msg.z_valid = True
        msg.v_xy_valid = True
        msg.v_z_valid = True
        msg.heading_good_for_control = True

        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.heading = 1.57

        msg.vx = 0.0
        msg.vy = 0.0
        msg.vz = 0.0

        ## publish the msg
        self.pub_.publish(msg)


def main(args = None):

    rclpy.init(args = args)

    node =  SpoofPX4()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()


