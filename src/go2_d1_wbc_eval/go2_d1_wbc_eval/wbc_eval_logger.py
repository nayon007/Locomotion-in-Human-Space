#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WBCEvalLogger(Node):
    def __init__(self):
        super().__init__('wbc_eval_logger')
        self.declare_parameter('mode', 'raw')  # raw | pocs | osqp
        self.mode = self.get_parameter('mode').value
        self.pub = self.create_publisher(String, '/wbc_eval/status', 10)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info(f'WBC eval logger started (mode={self.mode}).')

    def on_timer(self):
        self.pub.publish(String(data=f'mode={self.mode} alive'))

def main():
    rclpy.init()
    n = WBCEvalLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
