#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from unitree_api.msg import Request as UnitreeRequest

def clamp(x, lo, hi): return max(lo, min(hi, x))

QOS_BE1 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class CmdvelToUnitreeReq(Node):
    def __init__(self):
        super().__init__("cmdvel_to_unitree_req")

        # signs / limits
        self.declare_parameter("sign_vx", -1.0)   # many Unitree firmwares need -x to go forward
        self.declare_parameter("sign_vy",  1.0)
        self.declare_parameter("sign_wz", -1.0)
        self.declare_parameter("max_vx", 0.25)
        self.declare_parameter("max_vy", 0.20)
        self.declare_parameter("max_wz", 0.60)
        self.declare_parameter("rate_hz", 20.0)

        # ZERO behavior
        self.declare_parameter("zero_behavior", "stand")
        self.declare_parameter("zero_eps", 1e-3)

        # Perception gate
        self.declare_parameter("use_detector_gate", True)
        self.declare_parameter("detector_topic", "/perception/bottle_pose")
        self.declare_parameter("detector_fresh_s", 0.6)

        # Allow yaw search through when stale
        self.declare_parameter("allow_search_when_stale", True)
        self.declare_parameter("search_wz_min", 0.05)  # pass-through threshold

        # params -> fields
        g = self.get_parameter
        self.sign_vx = float(g("sign_vx").value)
        self.sign_vy = float(g("sign_vy").value)
        self.sign_wz = float(g("sign_wz").value)
        self.max_vx  = float(g("max_vx").value)
        self.max_vy  = float(g("max_vy").value)
        self.max_wz  = float(g("max_wz").value)
        self.rate    = float(g("rate_hz").value)
        self.zero_behavior = str(g("zero_behavior").value).lower()
        self.zero_eps = float(g("zero_eps").value)
        self.use_gate = bool(g("use_detector_gate").value)
        self.det_fresh= float(g("detector_fresh_s").value)
        self.allow_search = bool(g("allow_search_when_stale").value)
        self.search_wz_min = float(g("search_wz_min").value)

        # topics
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.on_twist, 10)
        self.pub_req = self.create_publisher(UnitreeRequest, "/api/sport/request", QOS_BE1)

        # gate header from detector (always published by perception now)
        det_header_topic = str(g("detector_topic").value) + "/header"
        self.last_det = 0.0
        self.sub_det = self.create_subscription(Header, det_header_topic, self.on_det_header, 10)

        self.last_cmd = (0.0, 0.0, 0.0)
        self.timer = self.create_timer(1.0/self.rate, self.tick)

        self.get_logger().info("cmdvel->Unitree: gating on detector freshness; yaw search allowed when stale.")

    def on_det_header(self, hdr: Header):
        self.last_det = self.get_clock().now().nanoseconds * 1e-9

    def fresh(self) -> bool:
        if not self.use_gate: return True
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self.last_det) <= self.det_fresh

    def on_twist(self, msg: Twist):
        vx = clamp(msg.linear.x,  -self.max_vx, self.max_vx)
        vy = clamp(msg.linear.y,  -self.max_vy, self.max_vy)
        wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        self.last_cmd = (vx, vy, wz)

    def publish_req(self, api_id: int, parameter: str = ""):
        m = UnitreeRequest()
        m.header.identity.api_id = api_id
        m.parameter = parameter
        self.pub_req.publish(m)

    def tick(self):
        vx, vy, wz = self.last_cmd
        mag = abs(vx) + abs(vy) + abs(wz)
        is_fresh = self.fresh()

        # If stale and not allowed to search -> stop
        if mag <= self.zero_eps or (not is_fresh and not self.allow_search):
            if self.zero_behavior == "stand":
                self.publish_req(1002, "")   # STAND
            elif self.zero_behavior == "damp":
                self.publish_req(1007, "")   # DAMP
            return

        # If stale but search is allowed: pass yaw-only
        if not is_fresh and self.allow_search:
            if abs(wz) >= self.search_wz_min:
                js = json.dumps({"x": 0.0, "y": 0.0, "z": float(self.sign_wz * wz)})
                self.publish_req(1008, js)
            else:
                if self.zero_behavior == "stand":
                    self.publish_req(1002, "")
                elif self.zero_behavior == "damp":
                    self.publish_req(1007, "")
            return

        # Fresh -> full motion
        js = json.dumps({
            "x": float(self.sign_vx * vx),
            "y": float(self.sign_vy * vy),
            "z": float(self.sign_wz * wz),
        })
        self.publish_req(1008, js)

def main():
    rclpy.init()
    n = CmdvelToUnitreeReq()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

