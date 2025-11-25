#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class HUD(Node):
    def __init__(self):
        super().__init__('csr_hud')
        self.sub = self.create_subscription(PoseStamped, '/perception/bottle_pose_base', self.cb, 10)
    def cb(self, msg):
        x,y = msg.pose.position.x, msg.pose.position.y
        r = math.hypot(x,y)
        print(f"range:{x:.2f}  lateral:{y:.2f}  r:{r:.2f}")

def main():
    rclpy.init(); rclpy.spin(HUD()); rclpy.shutdown()
if __name__=='__main__': main()
