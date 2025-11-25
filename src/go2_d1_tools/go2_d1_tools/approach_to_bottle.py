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
def dead(x, eps):    return 0.0 if abs(x) < eps else x

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
        self.declare_parameter("bottle_topic", "/perception/bottle_pose")  # camera-frame by default
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
        self.declare_parameter("yaw_for_vx_cutoff_rad", 0.60)
        self.declare_parameter("y_for_vx_cutoff_m",     0.25)

        # Filters / stability
        self.declare_parameter("ema_alpha", 0.5)
        self.declare_parameter("outlier_jump_m", 0.6)
        self.declare_parameter("min_detections", 2)
        self.declare_parameter("fresh_window_s", 0.5)
        self.declare_parameter("lost_timeout_s", 0.7)

        # Search when lost
        self.declare_parameter("search_mode", "yaw_sweep")
        self.declare_parameter("search_wz", 0.25)
        self.declare_parameter("search_period_s", 4.0)

        # Auto-stand nudges
        self.declare_parameter("send_stand_in_hold", True)
        self.declare_parameter("stand_repeat_s", 0.8)

        # IMU stabilization
        self.declare_parameter("use_imu", False)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("kd_wz", 0.2)
        self.declare_parameter("tilt_scale_start_deg", 5.0)
        self.declare_parameter("tilt_stop_deg", 15.0)

        # detector freshness header (bridge listens to this)
        self.declare_parameter("emit_header_for_gate", False)  # now detector publishes the header
        self.declare_parameter("header_topic", "/perception/bottle_pose/header")

        # Command smoothing
        self.declare_parameter("cmd_lpf_alpha", 0.6)
        self.declare_parameter("accel_limit_vx", 0.6)
        self.declare_parameter("accel_limit_vy", 0.8)
        self.declare_parameter("accel_limit_wz", 2.0)
        self.declare_parameter("deadband_vy",  0.02)
        self.declare_parameter("deadband_wz",  0.02)
        self.declare_parameter("vx_nonnegative", True)

        # Read params
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
        self.cmd_alpha = float(gp("cmd_lpf_alpha"))
        self.ax_lim = float(gp("accel_limit_vx"))
        self.ay_lim = float(gp("accel_limit_vy"))
        self.aw_lim = float(gp("accel_limit_wz"))
        self.db_vy = float(gp("deadband_vy"))
        self.db_wz = float(gp("deadband_wz"))
        self.vx_nonneg = bool(gp("vx_nonnegative"))

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

        # TF
        self.tf_buf = Buffer(); self.tf = TransformListener(self.tf_buf, self)

        # State
        self.last_meas = None; self.filt_xy = None; self.last_rx = 0.0
        self.det_times = collections.deque(maxlen=20)
        self.in_hold = False; self.ready_in = False; self.last_stand_pub = 0.0
        self.imu_wz = 0.0; self.tilt = 0.0
        self.prev_cmd = Twist()

        self.create_timer(self.dt, self.tick)
        self.get_logger().info("ApproachToBottle: center (yaw+vy), forward-only vx, smoothed.")

    # ---------- Subscribers ----------
    def on_ready(self, msg: Bool): self.ready_in = bool(msg.data)

    def on_imu(self, msg: Imu):
        self.imu_wz = float(msg.angular_velocity.z)
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

            # (Header moved to detector; we can still emit if enabled)
            if self.pub_hdr:
                hdr = Header(); hdr.stamp = self.get_clock().now().to_msg()
                self.pub_hdr.publish(hdr)

            # EMA with jump guard
            x, y = self.last_meas
            if self.filt_xy is None:
                self.filt_xy = (x, y)
            else:
                fx, fy = self.filt_xy
                if abs(x-fx) < self.jump_m and abs(y-fy) < self.jump_m:
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

    def rate_limit(self, target: Twist) -> Twist:
        out = Twist()
        dvx = max(-self.ax_lim*self.dt, min(self.ax_lim*self.dt, target.linear.x - self.prev_cmd.linear.x))
        dvy = max(-self.ay_lim*self.dt, min(self.ay_lim*self.dt, target.linear.y - self.prev_cmd.linear.y))
        dwz = max(-self.aw_lim*self.dt, min(self.aw_lim*self.dt, target.angular.z - self.prev_cmd.angular.z))
        rl = Twist()
        rl.linear.x  = self.prev_cmd.linear.x  + dvx
        rl.linear.y  = self.prev_cmd.linear.y  + dvy
        rl.angular.z = self.prev_cmd.angular.z + dwz
        a = max(0.0, min(1.0, self.cmd_alpha))
        out.linear.x  = (1-a)*rl.linear.x  + a*target.linear.x
        out.linear.y  = (1-a)*rl.linear.y  + a*target.linear.y
        out.angular.z = (1-a)*rl.angular.z + a*target.angular.z
        self.prev_cmd = out
        return out

    # ---------- Main loop ----------
    def tick(self):
        now = time.time()
        raw = Twist()

        # stale -> lost
        if (now - self.last_rx) > self.lost_timeout:
            self.last_meas = None; self.filt_xy = None; self.in_hold = False

        # require “ready”?
        if self.require_ready and not self.ready_in:
            self.stand("waiting_ready")
            self.emit(raw, "waiting_ready", False); return

        # lost or not yet stable -> search or stand
        if self.last_meas is None or self.filt_xy is None or not self.stable():
            if self.search_mode == "yaw_sweep":
                phase = math.sin(2.0*math.pi*((now % self.search_T)/self.search_T))
                raw.angular.z = sat(self.search_wz * phase, -self.max_wz, self.max_wz)
                self.emit(self.rate_limit(raw), f"search wz={raw.angular.z:.2f}", False)
            else:
                self.stand("lost")
                self.emit(Twist(), "lost->rest", False)
            return

        # Filtered pose in base_link (x forward, y left)
        x, y = self.filt_xy
        yaw_err = math.atan2(y, x)
        dist_err = x - self.stop_d

        # HOLD band: freeze
        if (self.stop_d - self.band) <= x <= (self.stop_d + self.band) and abs(yaw_err) <= self.yaw_dead:
            self.in_hold = True
            if self.send_stand_hold: self.stand("hold")
            self.emit(self.rate_limit(Twist()), f"ready x={x:.2f} y={y:.2f}", True)
            return
        self.in_hold = False

        # Center
        wz = sat(self.kyaw * yaw_err, -self.max_wz, self.max_wz)
        vy = sat(self.ky_lat * y,     -self.max_vy, self.max_vy)
        wz = dead(wz, self.db_wz)
        vy = dead(vy, self.db_vy)

        # IMU damping
        if self.use_imu:
            wz -= self.kd_wz * self.imu_wz
            wz = sat(wz, -self.max_wz, self.max_wz)

        # Forward-only
        yaw_scale = sat(1.0 - abs(yaw_err)/self.yaw_vx_cut, 0.0, 1.0)
        y_scale   = sat(1.0 - abs(y)/self.y_vx_cut,         0.0, 1.0)
        vx = self.kx * max(0.0, dist_err) * yaw_scale * y_scale
        if self.vx_nonneg: vx = max(0.0, vx)

        # Near-person lateral softening
        near_scale = sat(x / max(self.stop_d, 1e-3), 0.4, 1.0)

        raw.linear.x  = sat(vx, 0.0, self.max_vx)
        raw.linear.y  = sat(vy * near_scale, -self.max_vy, self.max_vy)
        raw.angular.z = wz

        self.emit(self.rate_limit(raw),
                  f"follow vx={raw.linear.x:.2f} vy={raw.linear.y:.2f} wz={raw.angular.z:.2f} yaw={math.degrees(yaw_err):.1f} dist={x:.2f}",
                  False)

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

