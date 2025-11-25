import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class Viz(Node):
    def __init__(self):
        super().__init__('perception_viz')
        self.pub = self.create_publisher(MarkerArray, '/handover/markers', 10)
        self.create_subscription(PoseStamped, '/perception/bottle_pose', self.on_bottle, 10)
        self.create_subscription(PoseStamped, '/perception/hand_pose', self.on_hand, 10)
        self.markers = MarkerArray()
    def on_bottle(self, msg):
        m = Marker()
        m.header = msg.header; m.ns = "bottle"; m.id = 10
        m.type = Marker.ARROW; m.scale.x = 0.15; m.scale.y = 0.03; m.scale.z = 0.03
        m.pose = msg.pose
        m.color.g = 1.0; m.color.a = 0.8
        self.markers.markers.append(m); self.pub.publish(self.markers)
    def on_hand(self, msg):
        m = Marker()
        m.header = msg.header; m.ns = "hand"; m.id = 11
        m.type = Marker.SPHERE; m.scale.x = 0.06; m.scale.y = 0.06; m.scale.z = 0.06
        m.pose = msg.pose
        m.color.r = 1.0; m.color.a = 0.8
        self.markers.markers.append(m); self.pub.publish(self.markers)

def main():
    rclpy.init()
    rclpy.spin(Viz())
    rclpy.shutdown()
