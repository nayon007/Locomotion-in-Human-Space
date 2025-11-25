#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String

def clamp(v,a,b): return max(a, min(b, v))

class ApproachController(Node):
    def __init__(self):
        super().__init__('approach_controller')
        # topics
        self.declare_parameter('pose_topic', '/perception/bottle_pose_base')
        self.declare_parameter('cmd_topic',  '/cmd_vel')
        self.declare_parameter('status_topic','/approach/state')
        # gains
        self.declare_parameter('x_goal', 0.55)
        self.declare_parameter('kx',     0.8)
        self.declare_parameter('kyaw',   1.2)
        self.declare_parameter('kvy',    0.2)
        # limits
        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('max_vy', 0.20)
        self.declare_parameter('max_wz', 0.60)
        # ready window + stability
        self.declare_parameter('tol_x',  0.05)   # +/- 5 cm
        self.declare_parameter('tol_y',  0.05)   # +/- 5 cm
        self.declare_parameter('stable_count', 8)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_pub   = self.create_publisher(Twist,   self.get_parameter('cmd_topic').value,   10)
        self.state_pub = self.create_publisher(String,  self.get_parameter('status_topic').value, 10)
        self.ready_pub = self.create_publisher(Bool,    '/approach/ready', 10)
        self.sub       = self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 10)

        self.x_goal  = float(self.get_parameter('x_goal').value)
        self.kx      = float(self.get_parameter('kx').value)
        self.kyaw    = float(self.get_parameter('kyaw').value)
        self.kvy     = float(self.get_parameter('kvy').value)
        self.max_vx  = float(self.get_parameter('max_vx').value)
        self.max_vy  = float(self.get_parameter('max_vy').value)
        self.max_wz  = float(self.get_parameter('max_wz').value)
        self.tol_x   = float(self.get_parameter('tol_x').value)
        self.tol_y   = float(self.get_parameter('tol_y').value)
        self.stable_N = int(self.get_parameter('stable_count').value)

        self.ready = False
        self.stable = 0
        self.get_logger().info(
            f"ApproachController: goal={self.x_goal:.2f}m, kx={self.kx}, kyaw={self.kyaw}, kvy={self.kvy}"
        )

    def on_pose(self, ps: PoseStamped):
        x = float(ps.pose.position.x)   # forward in base_link
        y = float(ps.pose.position.y)   # left(+)/right(-)

        # errors
        ex = self.x_goal - x
        ey = -y  # drive lateral to 0

        # simple P-control
        vx = clamp(self.kx*ex,  -self.max_vx, self.max_vx)
        vy = clamp(self.kvy*ey, -self.max_vy, self.max_vy)
        wz = clamp(self.kyaw*math.atan2(y, max(1e-6, x)), -self.max_wz, self.max_wz)

        at_x = abs(ex) <= self.tol_x
        at_y = abs(y)  <= self.tol_y

        # stability latch
        if at_x and at_y:
            self.stable += 1
        else:
            self.stable = 0

        new_ready = self.stable >= self.stable_N

        # if latched, stop base completely
        if new_ready:
            vx = vy = wz = 0.0

        # detect state change for logging
        if new_ready != self.ready:
            self.get_logger().info(f"{'READY' if new_ready else 'tracking'} at x={x:.2f} y={y:.2f}")

        self.ready = new_ready

        # publish
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = vx, vy, wz
        self.cmd_pub.publish(tw)

        st = String()
        st.data = f"{'READY' if self.ready else 'tracking'} x={x:.2f} y={y:.2f} -> vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}"
        self.state_pub.publish(st)
        self.ready_pub.publish(Bool(data=self.ready))

def main():
    rclpy.init()
    node = ApproachController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()

