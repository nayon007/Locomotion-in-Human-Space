#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WBCRouter(Node):
    def __init__(self):
        super().__init__('wbc_router')
        self.declare_parameter('mode','osqp')  # raw | pocs | osqp
        self.mode = self.get_parameter('mode').get_parameter_value().string_value.lower()
        # in
        self.sub_des  = self.create_subscription(Twist, '/cmd_vel_des',  self.on_des, 10)
        self.sub_stab = self.create_subscription(Twist, '/cmd_vel_stab', self.on_stab, 10)
        # out to robot
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
        # taps for logger (stable topic names)
        self.pub_tap_des  = self.create_publisher(Twist, '/wbc_eval/cmd_des', 10)
        self.pub_tap_out  = self.create_publisher(Twist, '/wbc_eval/cmd_out', 10)
        # state
        self.latest_des  = Twist()
        self.latest_stab = Twist()
        self.pub_mode = self.create_publisher(String, '/wbc_eval/mode', 10)
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz
        self.get_logger().info(f"WBCRouter: mode={self.mode} (raw => /cmd_vel_des, pocs/osqp => /cmd_vel_stab)")

    def on_des(self, m: Twist):
        self.latest_des = m
        self.pub_tap_des.publish(m)

    def on_stab(self, m: Twist):
        self.latest_stab = m

    def tick(self):
        out = self.latest_des if self.mode == 'raw' else self.latest_stab
        self.pub_cmd.publish(out)
        self.pub_tap_out.publish(out)
        self.pub_mode.publish(String(data=self.mode))

def main():
    rclpy.init(); n = WBCRouter()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
