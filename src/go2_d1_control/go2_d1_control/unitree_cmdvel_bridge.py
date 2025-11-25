#!/usr/bin/env python3
import json, rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelToUnitree(Node):
    def __init__(self):
        super().__init__('cmdvel_to_unitree')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('topic_out', '/api/sport/request')
        self.declare_parameter('schema', 'move_vx_vy_vyaw')  # in case you need to change keys

        self.pub = self.create_publisher(String, self.get_parameter('topic_out').value, 10)
        self.cmd = Twist()
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.dt = 1.0/float(self.get_parameter('publish_hz').value)
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(f"Bridging /cmd_vel -> {self.get_parameter('topic_out').value} at {1/self.dt:.1f} Hz")

    def on_cmd(self, msg: Twist):
        self.cmd = msg

    def step(self):
        vx = float(self.cmd.linear.x)
        vy = float(self.cmd.linear.y)
        wz = float(self.cmd.angular.z)
        payload = {"cmd":"move","vx":vx,"vy":vy,"vyaw":wz,"duration":self.dt}
        self.pub.publish(String(data=json.dumps(payload)))

def main():
    rclpy.init(); rclpy.spin(CmdVelToUnitree()); rclpy.shutdown()
if __name__ == '__main__': main()
