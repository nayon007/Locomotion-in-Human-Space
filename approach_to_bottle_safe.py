#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException   # <-- correct import
from unitree_api.msg import Request as UnitreeRequest

# BEST_EFFORT depth=1 (matches robot DDS)
QOS_BE1 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

def fill_req(js: str) -> UnitreeRequest:
    m = UnitreeRequest()
    for f in ("data","json","content","msg","message"):
        if hasattr(m, f):
            setattr(m, f, js); return m
    if hasattr(m, "data"): m.data = js
    return m

class ApproachToBottle(Node):
    def __init__(self):
        super().__init__("approach_to_bottle")

        # Parameters
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("bottle_topic", "/perception/bottle_pose")
        self.declare_parameter("stop_distance_m", 0.60)
        self.declare_parameter("stop_band_m",    0.08)
        self.declare_parameter("yaw_dead_rad",   0.12)
        self.declare_parameter("yaw_ok_rad",     0.20)
        self.declare_parameter("max_vx",         0.25)
        self.declare_parameter("max_wz",         0.6)
        self.declare_parameter("kx",             0.8)
        self.declare_parameter("kyaw",           1.2)
        self.declare_parameter("lost_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.target_frame = self.get_parameter("target_frame").value
        self.stop_d = float(self.get_parameter("stop_distance_m").value)
        self.band   = float(self.get_parameter("stop_band_m").value)
        self.yaw_dead = float(self.get_parameter("yaw_dead_rad").value)
        self.yaw_ok   = float(self.get_parameter("yaw_ok_rad").value)
        self.kx = float(self.get_parameter("kx").value)
        self.kyaw = float(self.get_parameter("kyaw").value)
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_wz = float(self.get_parameter("max_wz").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout_s").value)
        self.dt = 1.0/float(self.get_parameter("publish_rate_hz").value)

        # IO
        self.sub = self.create_subscription(PoseStamped,
                                            self.get_parameter("bottle_topic").value,
                                            self.on_pose,
                                            qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_ready = self.create_publisher(Bool,  "/approach/ready", 10)
        self.pub_state = self.create_publisher(String,"/approach/state", 10)

        # one-shot "stand" when ready
        self.pub_sport = self.create_publisher(UnitreeRequest, "/api/sport/request", QOS_BE1)

        # TF
        self.tf_buf = Buffer(); self.tf = TransformListener(self.tf_buf, self)

        self.last_pose = None
        self.last_rx   = 0.0
        self.sent_stand = False
        self.in_hold    = False

        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"Approach controller → stop @ {self.stop_d:.2f} m; yaw_ok {self.yaw_ok:.2f} rad")

    def on_pose(self, msg: PoseStamped):
        try:
            if msg.header.frame_id and msg.header.frame_id != self.target_frame:
                tf = self.tf_buf.lookup_transform(self.target_frame, msg.header.frame_id, rclpy.time.Time())
                # R from quaternion
                q = tf.transform.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = [[1-2*(y*y+z*z), 2*(x*y-w*z),     2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z),   2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),     1-2*(x*x+y*y)]]
                t = (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z)
                p = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                v2 = (R[0][0]*p[0]+R[0][1]*p[1]+R[0][2]*p[2] + t[0],
                      R[1][0]*p[0]+R[1][1]*p[1]+R[1][2]*p[2] + t[1],
                      R[2][0]*p[0]+R[2][1]*p[1]+R[2][2]*p[2] + t[2])
                self.last_pose = v2
            else:
                self.last_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            self.last_rx = time.time()
        except TransformException as e:
            self.get_logger().warn(f"TF fail: {e}")
            self.last_pose = None

    def publish_stand_once(self):
        if self.sent_stand: return
        self.sent_stand = True
        req = UnitreeRequest()
        req.header.identity.api_id = 1002   # STAND
        req.parameter = ""
        self.pub_sport.publish(req)
        self.get_logger().info("Sent one-shot STAND (1002)")

    def tick(self):
        if time.time() - self.last_rx > self.lost_timeout:
            self.last_pose = None

        cmd = Twist(); state = "idle"
        if self.last_pose is None:
            self.in_hold = False; self.sent_stand = False
            state = "lost"
        else:
            x, y, z = self.last_pose
            # yaw to target
            import math
            yaw_err = math.atan2(y, x)
            wz = max(-self.max_wz, min(self.max_wz, self.kyaw * yaw_err))
            ex = x - self.stop_d
            vx_cmd = self.kx * ex if abs(yaw_err) < self.yaw_ok else 0.0

            if self.in_hold:
                if (x < self.stop_d - self.band) or (x > self.stop_d + self.band) or (abs(yaw_err) > self.yaw_ok):
                    self.in_hold = False
                else:
                    vx_cmd = 0.0; wz = 0.0
            else:
                if (self.stop_d - self.band) <= x <= (self.stop_d + self.band) and abs(yaw_err) <= self.yaw_dead:
                    self.in_hold = True; vx_cmd = 0.0; wz = 0.0

            vx_cmd = max(-self.max_vx, min(self.max_vx, vx_cmd))
            if self.in_hold:
                state = f"ready x={x:.2f} y={y:.2f}"
                self.publish_stand_once()
            else:
                self.sent_stand = False
                cmd.linear.x = float(vx_cmd)
                cmd.angular.z = float(wz)
                state = f"tracking vx={cmd.linear.x:.2f} wz={cmd.angular.z:.2f} | x={x:.2f} y={y:.2f}"

        self.pub_cmd.publish(cmd)
        self.pub_state.publish(String(data=state))
        self.pub_ready.publish(Bool(data=self.in_hold))

def main():
    rclpy.init()
    n = ApproachToBottle()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
