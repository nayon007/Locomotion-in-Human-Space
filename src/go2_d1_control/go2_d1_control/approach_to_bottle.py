#!/usr/bin/env python3
import math, time, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, Header
from tf2_ros import Buffer, TransformListener, TransformException
from unitree_api.msg import Request as UnitreeRequest

def sat(x, lo, hi): return max(lo, min(hi, x))

QOS_BE1 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class ApproachToBottle(Node):
    def __init__(self):
        super().__init__("approach_to_bottle")

        # Frames / topics
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("bottle_topic", "/perception/bottle_pose")
        self.declare_parameter("handover_ready_topic", "/perception/handover_ready")
        self.declare_parameter("require_ready", False)
        self.declare_parameter("publish_rate_hz", 20.0)

        # Targets / bands
        self.declare_parameter("stop_distance_m", 0.60)
        self.declare_parameter("stop_band_m",    0.08)
        self.declare_parameter("yaw_dead_rad",   0.12)

        # Controller gains
        self.declare_parameter("kx", 0.9)
        self.declare_parameter("kyaw", 1.4)
        self.declare_parameter("ky_lat", 1.0)

        # Limits
        self.declare_parameter("max_vx", 0.25)
        self.declare_parameter("max_vy", 0.20)
        self.declare_parameter("max_wz", 0.60)

        # vx scaling vs misalignment
        self.declare_parameter("yaw_for_vx_cutoff_rad", 0.60)  # big yaw -> reduce vx
        self.declare_parameter("y_for_vx_cutoff_m",     0.25)  # big lateral -> reduce vx

        # Filters / stability
        self.declare_parameter("ema_alpha", 0.5)
        self.declare_parameter("outlier_jump_m", 0.6)
        self.declare_parameter("min_detections", 2)
        self.declare_parameter("fresh_window_s", 0.5)
        self.declare_parameter("lost_timeout_s", 0.7)

        # Search behavior when lost
        self.declare_parameter("search_mode", "yaw_sweep")   # "none" or "yaw_sweep"
        self.declare_parameter("search_wz", 0.25)
        self.declare_parameter("search_period_s", 4.0)

        # Auto-stand nudges
        self.declare_parameter("send_stand_in_hold", True)
        self.declare_parameter("stand_repeat_s", 0.8)

        # IMU stabilization
        self.declare_parameter("use_imu", True)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("kd_wz", 0.2)                 # yaw rate damping
        self.declare_parameter("tilt_scale_start_deg", 5.0)  # start scaling vx
        self.declare_parameter("tilt_stop_deg", 15.0)        # zero vx above this

        # detector freshness gate
        self.declare_parameter("emit_header_for_gate", True) # publish a Header so the bridge can gate
        self.declare_parameter("header_topic", "/perception/bottle_pose/header")

        # Read parameters
        gp = lambda n: self.get_parameter(n).value
        self.target_frame = gp("target_frame")
        self.stop_d   = float(gp("stop_distance_m")); self.band = float(gp("stop_band_m"))
        self.yaw_dead = float(gp("yaw_dead_rad"))
        self.kx = float(gp("kx")); self.kyaw = float(gp("kyaw")); self.ky_lat = float(gp("ky_lat"))
        self.max_vx = float(gp("max_vx")); self.max_vy = float(gp("max_vy")); self.max_wz = float(gp("max_wz"))
        self.yaw_vx_cut = float(gp("yaw_for_vx_cutoff_rad"))
        self.y_vx_cut   = float(gp("y_for_vx_cutoff_m"))
        self.alpha = float(gp("ema_alpha")); self.jump_m = float(gp("outlier_jump_m"))
        self.min_det = int(gp("min_detections")); self.win_s = float(gp("fresh_window_s"))
        self.lost_timeout = float(gp("lost_timeout_s"))
        self.search_mode = str(gp("search_mode")).lower()
        self.search_wz = float(gp("search_wz")); self.search_T = float(gp("search_period_s"))
        self.send_stand_hold = bool(gp("send_stand_in_hold"))
        self.stand_repeat = float(gp("stand_repeat_s"))
        self.dt = 1.0/float(gp("publish_rate_hz"))
        self.require_ready = bool(gp("require_ready"))
        self.use_imu = bool(gp("use_imu")); self.imu_topic = str(gp("imu_topic"))
        self.kd_wz = float(gp("kd_wz"))
        self.tilt_start = math.radians(float(gp("tilt_scale_start_deg")))
        self.tilt_stop  = math.radians(float(gp("tilt_stop_deg")))
        self.emit_header= bool(gp("emit_header_for_gate"))
        self.header_topic = str(gp("header_topic"))

        # IO
        self.sub_pose  = self.create_subscription(PoseStamped, gp("bottle_topic"), self.on_pose, qos_profile_sensor_data)
        self.sub_ready = self.create_subscription(Bool, gp("handover_ready_topic"), self.on_ready, 10)
        if self.use_imu:
            self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.on_imu, 10)
        self.pub_cmd   = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_ready = self.create_publisher(Bool,  "/approach/ready", 10)
        self.pub_state = self.create_publisher(String,"/approach/state", 10)
        self.pub_sport = self.create_publisher(UnitreeRequest, "/api/sport/request", QOS_BE1)
        self.pub_hdr   = self.create_publisher(Header, self.header_topic, 10) if self.emit_header else None

        self.tf_buf = Buffer(); self.tf = TransformListener(self.tf_buf, self)
        self.last_meas = None; self.filt_xy = None; self.last_rx = 0.0
        self.det_times = collections.deque(maxlen=20)
        self.in_hold = False; self.ready_in = False; self.last_stand_pub = 0.0

        # IMU state
        self.imu_wz = 0.0; self.tilt = 0.0

        self.create_timer(self.dt, self.tick)
        self.get_logger().info("ApproachToBottle: follow-only (vx scales), yaw+vy center, IMU-damped.")

    # ---------- Subscribers ----------
    def on_ready(self, msg: Bool): self.ready_in = bool(msg.data)

    def on_imu(self, msg: Imu):
        self.imu_wz = float(msg.angular_velocity.z)
        # roll/pitch magnitude from quaternion
        q = msg.orientation
        sinr_cosp = 2*(q.w*q.x + q.y*q.z)
        cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2*(q.w*q.y - q.z*q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.tilt = math.hypot(roll, pitch)

    def on_pose(self, msg: PoseStamped):
        try:
            if msg.header.frame_id and msg.header.frame_id != self.target_frame:
                tf = self.tf_buf.lookup_transform(self.target_frame, msg.header.frame_id, rclpy.time.Time())
                q = tf.transform.rotation; t = tf.transform.translation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = [[1-2*(y*y+z*z), 2*(x*y-w*z),     2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z),   2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),     1-2*(x*x+y*y)]]
                p = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                x2 = R[0][0]*p[0]+R[0][1]*p[1]+R[0][2]*p[2] + t.x
                y2 = R[1][0]*p[0]+R[1][1]*p[1]+R[1][2]*p[2] + t.y
                self.last_meas = (x2, y2)
            else:
                self.last_meas = (msg.pose.position.x, msg.pose.position.y)

            now = time.time()
            self.last_rx = now; self.det_times.append(now)

            # emit a header so the bridge can gate on detector freshness
            if self.pub_hdr:
                hdr = Header(); hdr.stamp = self.get_clock().now().to_msg()
                self.pub_hdr.publish(hdr)

            # EMA with jump guard
            x, y = self.last_meas
            if self.filt_xy is None:
                self.filt_xy = (x, y)
            else:
                fx, fy = self.filt_xy
                if abs(x-fx) < 0.6 and abs(y-fy) < 0.6:
                    fx = self.alpha*x + (1-self.alpha)*fx
                    fy = self.alpha*y + (1-self.alpha)*fy
                    self.filt_xy = (fx, fy)
        except TransformException as e:
            self.get_logger().warn(f"TF fail: {e}")
            self.last_meas = None

    # ---------- Helpers ----------
    def stable(self):
        now = time.time()
        return sum(1 for t in self.det_times if now-t <= self.win_s) >= self.min_det

    def stand(self, why: str):
        now = time.time()
        if now - self.last_stand_pub < self.stand_repeat: return
        self.last_stand_pub = now
        m = UnitreeRequest(); m.header.identity.api_id = 1002; m.parameter = ""
        self.pub_sport.publish(m)
        self.get_logger().info(f"STAND (1002) [{why}]")

    def emit(self, cmd: Twist, state: str, ready: bool):
        self.pub_cmd.publish(cmd)
        self.pub_state.publish(String(data=state))
        self.pub_ready.publish(Bool(data=ready))

    # ---------- Main loop ----------
    def tick(self):
        now = time.time()
        cmd = Twist(); state = "idle"

        # stale -> lost
        if (now - self.last_rx) > self.lost_timeout:
            self.last_meas = None; self.filt_xy = None; self.in_hold = False

        # require “ready”?
        if self.require_ready and not self.ready_in:
            self.stand("waiting_ready")
            self.emit(cmd, "waiting_ready", False); return

        # no pose or not stable -> rest or search (no small stepping)
        if self.last_meas is None or self.filt_xy is None or not self.stable():
            if self.search_mode == "yaw_sweep":
                phase = math.sin(2.0*math.pi*((now % self.search_T)/self.search_T))
                cmd.angular.z = sat(self.search_wz * phase, -self.max_wz, self.max_wz)
                self.emit(cmd, f"search wz={cmd.angular.z:.2f}", False)
            else:
                self.stand("lost")
                self.emit(Twist(), "lost->rest", False)
            return

        # Filtered pose in base_link
        x, y = self.filt_xy
        yaw_err = math.atan2(y, x)
        dist_err = x - self.stop_d

        # HOLD band
        if (self.stop_d - self.band) <= x <= (self.stop_d + self.band) and abs(yaw_err) <= self.yaw_dead:
            self.in_hold = True
            if self.send_stand_hold: self.stand("hold")
            self.emit(Twist(), f"ready x={x:.2f} y={y:.2f}", True)
            return
        self.in_hold = False

        # Always center: yaw + vy
        wz = sat(self.kyaw * yaw_err, -self.max_wz, self.max_wz)
        vy = sat(self.ky_lat * y,     -self.max_vy, self.max_vy)

        # IMU yaw rate damping
        if self.use_imu:
            wz -= self.kd_wz * self.imu_wz
            wz = sat(wz, -self.max_wz, self.max_wz)

        # vx scales with misalignment
        yaw_scale = sat(1.0 - abs(yaw_err)/self.yaw_vx_cut, 0.0, 1.0)
        y_scale   = sat(1.0 - abs(y)/self.y_vx_cut,         0.0, 1.0)
        vx = self.kx * max(0.0, dist_err) * yaw_scale * y_scale

        # IMU tilt limiter on vx
        if self.use_imu and self.tilt > self.tilt_start:
            if self.tilt >= self.tilt_stop:
                vx = 0.0
            else:
                s = 1.0 - (self.tilt - self.tilt_start) / (self.tilt_stop - self.tilt_start)
                vx *= max(0.0, min(1.0, s))

        cmd.linear.x = sat(vx, 0.0, self.max_vx)
        cmd.linear.y = vy
        cmd.angular.z= wz

        state = (f"follow vx={cmd.linear.x:.2f} vy={cmd.linear.y:.2f} "
                 f"wz={cmd.angular.z:.2f} yaw={math.degrees(yaw_err):.1f} "
                 f"tilt={math.degrees(self.tilt):.1f} "
                 f"dist={x:.2f}")
        self.emit(cmd, state, False)

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
