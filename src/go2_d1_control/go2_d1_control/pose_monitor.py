#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        self.create_subscription(PoseStamped, '/perception/bottle_pose', self.cb, 10)
        self.get_logger().info('Pose monitor: printing range and yaw_err (rad)')
    def cb(self, msg):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        d = math.sqrt(x*x + y*y + z*z)
        yaw_err = math.atan2(y, x)  # +ve -> bottle to the left
        self.get_logger().info(f"base_link -> bottle: x={x:.2f} y={y:.2f} z={z:.2f} | range={d:.2f} yaw_err={yaw_err:.2f} rad")
def main():
    rclpy.init(); rclpy.spin(PoseMonitor()); rclpy.shutdown()
if __name__ == '__main__': main()
