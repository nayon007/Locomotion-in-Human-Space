#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class PoseTfRelay(Node):
    def __init__(self):
        super().__init__('pose_tf_relay')
        self.declare_parameter('in_topic', '/perception/bottle_pose')
        self.declare_parameter('out_topic', '/perception/bottle_pose_base')
        self.declare_parameter('target_frame', 'base_link')

        self.in_topic  = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.target    = self.get_parameter('target_frame').value

        self.buf = tf2_ros.Buffer()
        self.lst = tf2_ros.TransformListener(self.buf, self)

        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.sub = self.create_subscription(PoseStamped, self.in_topic, self.cb, qos_profile_sensor_data)

        self.get_logger().info(f"Relaying {self.in_topic} -> {self.out_topic} in frame={self.target}")

    def cb(self, msg: PoseStamped):
        try:
            # Use latest available transform (time=0)
            if not self.buf.can_transform(self.target, msg.header.frame_id, Time(), timeout=Duration(seconds=0.1)):
                self.get_logger().warn_once(f"No TF {self.target} <- {msg.header.frame_id} yet; will retry...")
                return
            tf = self.buf.lookup_transform(self.target, msg.header.frame_id, Time())
            out = do_transform_pose(msg, tf)
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.target
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"TF relay failed: {e}")

def main():
    rclpy.init()
    rclpy.spin(PoseTfRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
