#!/usr/bin/env python3
import os, csv, math, time, pathlib
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Header, Bool, String
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformException

# Helpers
def clamp(v, lo, hi): return max(lo, min(hi, v))
def l2(*xs): return math.sqrt(sum(x*x for x in xs))

class CSRLogger(Node):
    """
    Subscribes to:
      - /perception/bottle_pose         (PoseStamped, camera optical frame)
      - /perception/bottle_pose_base    (PoseStamped, base_link) [optional]
      - /perception/bottle_pose/header  (Header)  [freshness ticks]
      - /camera/color/camera_info       (Camera intrinsics)
      - /cmd_vel                        (commanded base velocities)
      - /approach/state                 (debug text, optional)
      - /approach/ready                 (Bool, optional)

    Declares "entry" when in stop-band continuously for tau_enter_s.
    Declares "hold" when in stop-band continuously for tau_hold_s (after entry).
    Writes:
      - <out_dir>/<trial_id>/timeseries.csv
      - <out_dir>/<trial_id>/metrics.csv (one row)
    Then exits (for repeatable per-trial runs).
    """
    def __init__(self):
        super().__init__('csr_logger')

        # --- Parameters (mirrors paper section) ---
        self.declare_parameter('trial_id', 'trial_000')
        self.declare_parameter('out_dir', str(pathlib.Path.home() / 'go2_ws' / 'experiments'))
        self.declare_parameter('target_frame', 'base_link')

        # Band & timing
        self.declare_parameter('stop_distance_m', 0.60)
        self.declare_parameter('stop_band_m',    0.08)
        self.declare_parameter('yaw_dead_rad',   0.12)
        self.declare_parameter('tau_enter_s',    0.25)  # band must hold this long to declare entry
        self.declare_parameter('tau_hold_s',     1.00)  # band must hold this long to declare success
        self.declare_parameter('max_duration_s', 60.0)  # safety cutoff

        # Freshness window (for diagnostics)
        self.declare_parameter('fresh_window_s', 0.6)

        # Topics (leave defaults consistent with your stack)
        self.declare_parameter('pose_cam_topic',   '/perception/bottle_pose')
        self.declare_parameter('pose_base_topic',  '/perception/bottle_pose_base')
        self.declare_parameter('fresh_header_topic','/perception/bottle_pose/header')
        self.declare_parameter('camera_info_topic','/camera/color/camera_info')
        self.declare_parameter('cmd_vel_topic',    '/cmd_vel')
        self.declare_parameter('approach_state_topic','/approach/state')
        self.declare_parameter('approach_ready_topic','/approach/ready')

        # --- Resolve params ---
        gp = lambda n: self.get_parameter(n).value
        self.trial_id   = str(gp('trial_id'))
        self.out_dir    = str(gp('out_dir'))
        self.target_frame = str(gp('target_frame'))
        self.stop_d     = float(gp('stop_distance_m'))
        self.band       = float(gp('stop_band_m'))
        self.yaw_dead   = float(gp('yaw_dead_rad'))
        self.tau_enter  = float(gp('tau_enter_s'))
        self.tau_hold   = float(gp('tau_hold_s'))
        self.Tmax       = float(gp('max_duration_s'))
        self.fresh_win  = float(gp('fresh_window_s'))

        self.topic_cam  = str(gp('pose_cam_topic'))
        self.topic_base = str(gp('pose_base_topic'))
        self.topic_hdr  = str(gp('fresh_header_topic'))
        self.topic_info = str(gp('camera_info_topic'))
        self.topic_cmd  = str(gp('cmd_vel_topic'))
        self.topic_ast  = str(gp('approach_state_topic'))
        self.topic_ard  = str(gp('approach_ready_topic'))

        # --- IO setup ---
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.info = None
        self.last_hdr_time = 0.0

        # States for band timing
        self.in_band = False
        self.band_since = None
        self.entry_time = None   # T
        self.hold_time  = None   # success time

        # For jerk proxy (keep history of velocities)
        self.ts_hist = []
        self.vx_hist, self.vy_hist = [], []

        # Timeseries logging
        trial_dir = pathlib.Path(self.out_dir) / self.trial_id
        trial_dir.mkdir(parents=True, exist_ok=True)
        self.timeseries_path = str(trial_dir / 'timeseries.csv')
        self.metrics_path    = str(trial_dir / 'metrics.csv')
        self._ts_file = open(self.timeseries_path, 'w', newline='')
        self._ts_csv  = csv.writer(self._ts_file)
        self._ts_csv.writerow([
            't','fresh','u','v','ex_px','ey_px',
            'Z_cam','x_base','y_base','yaw_err','dist_err',
            'in_band','cmd_vx','cmd_vy','cmd_wz'
        ])
        self._ts_file.flush()

        # Metric accumulators (after T)
        self.max_residual_px = 0.0
        self.max_residual_cm = 0.0

        # Subscriptions
        self.create_subscription(CameraInfo, self.topic_info, self.on_info, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.topic_cam, self.on_pose_cam, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.topic_base, self.on_pose_base, qos_profile_sensor_data)
        self.create_subscription(Header, self.topic_hdr, self.on_header, 10)
        self.create_subscription(Twist, self.topic_cmd, self.on_cmd, 10)
        self.create_subscription(String, self.topic_ast, self.on_state, 10)
        self.create_subscription(Bool, self.topic_ard, self.on_ready, 10)

        # Per-tick timer (20 Hz) to handle timeout and write
        self.t0 = time.time()
        self.create_timer(0.05, self.on_timer)

        self.get_logger().info(f"CSRLogger: writing to {trial_dir}")

        # Working cache for latest signals
        self.last_cam = None      # (X,Y,Z) in camera frame
        self.last_base = None     # (x,y) in base_link
        self.last_cmd = (0.0,0.0,0.0)  # vx,vy,wz
        self.last_state = ""
        self.last_ready = False

    # ---------- Subscribers ----------
    def on_info(self, msg: CameraInfo):
        self.info = msg

    def on_header(self, hdr: Header):
        self.last_hdr_time = self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, m: Twist):
        self.last_cmd = (float(m.linear.x), float(m.linear.y), float(m.angular.z))
        # jerk buffers
        t = self.get_clock().now().nanoseconds * 1e-9
        self.ts_hist.append(t)
        self.vx_hist.append(self.last_cmd[0])
        self.vy_hist.append(self.last_cmd[1])
        if len(self.ts_hist) > 10000:
            self.ts_hist = self.ts_hist[-2000:]
            self.vx_hist = self.vx_hist[-2000:]
            self.vy_hist = self.vy_hist[-2000:]

    def on_state(self, s: String):
        self.last_state = s.data

    def on_ready(self, b: Bool):
        self.last_ready = bool(b.data)

    def on_pose_cam(self, ps: PoseStamped):
        self.last_cam = (float(ps.pose.position.x),
                         float(ps.pose.position.y),
                         float(ps.pose.position.z))
        # Also try to produce base_link if missing
        if self.last_base is None:
            try:
                tf = self.tf_buf.lookup_transform(self.target_frame, ps.header.frame_id, rclpy.time.Time())
                # Manual transform of point
                q = tf.transform.rotation; t = tf.transform.translation
                x,y,z,w = q.x,q.y,q.z,q.w
                R = np.array([
                    [1-2*(y*y+z*z), 2*(x*y-w*z),     2*(x*z+w*y)],
                    [2*(x*y+w*z),   1-2*(x*x+z*z),   2*(y*z-w*x)],
                    [2*(x*z-w*y),   2*(y*z+w*x),     1-2*(x*x+y*y)]
                ])
                p = np.array(self.last_cam)
                pb = R @ p + np.array([t.x, t.y, t.z])
                self.last_base = (float(pb[0]), float(pb[1]))
            except Exception:
                pass

    def on_pose_base(self, ps: PoseStamped):
        self.last_base = (float(ps.pose.position.x),
                          float(ps.pose.position.y))

    # ---------- Core timer ----------
    def on_timer(self):
        now_wall = time.time()
        t = now_wall - self.t0

        # timeout end
        if t >= self.Tmax:
            self.get_logger().warn("Max duration reached; writing metrics and exiting.")
            self.finish_and_exit(success=False)
            return

        # need intrinsics + at least camera or base pose
        if self.info is None or (self.last_cam is None and self.last_base is None):
            return

        # project to pixels (from camera frame)
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        if self.last_cam is not None:
            X, Y, Z = self.last_cam
            if Z > 1e-6:
                u = fx * (X / Z) + cx
                v = fy * (Y / Z) + cy
            else:
                u, v = float('nan'), float('nan')
        else:
            # if no camera pose, leave u,v NaN but we can still log base
            u = v = float('nan')
            Z = float('nan')

        ex_px = (u - cx) if not math.isnan(u) else float('nan')
        ey_px = (v - cy) if not math.isnan(v) else float('nan')

        # base frame quantities (x forward, y left)
        if self.last_base is not None:
            x_base, y_base = self.last_base
            yaw_err = math.atan2(y_base, x_base)
            dist_err = x_base - self.stop_d
        else:
            x_base = y_base = yaw_err = dist_err = float('nan')

        # band membership (requires base quantities)
        currently_in_band = False
        if not math.isnan(x_base) and not math.isnan(yaw_err):
            if (self.stop_d - self.band) <= x_base <= (self.stop_d + self.band) and abs(yaw_err) <= self.yaw_dead:
                currently_in_band = True

        # update band timers
        if currently_in_band:
            if not self.in_band:
                self.in_band = True
                self.band_since = now_wall
            # entry when held for tau_enter
            if self.entry_time is None and (now_wall - self.band_since) >= self.tau_enter:
                self.entry_time = now_wall
                self.get_logger().info(f"ENTRY at t={self.entry_time - self.t0:.2f}s")
            # after entry, hold success when held for tau_hold
            if self.entry_time is not None and (now_wall - self.band_since) >= (self.tau_hold):
                self.hold_time = now_wall
                self.get_logger().info(f"HOLD success at t={self.hold_time - self.t0:.2f}s")
                self.finish_and_exit(success=True)
                return
        else:
            self.in_band = False
            self.band_since = None

        # residuals after entry (while inside band)
        if self.entry_time is not None and currently_in_band:
            ed_cm = abs((x_base - self.stop_d) * 100.0) if not math.isnan(x_base) else 0.0
            e_px = math.sqrt((ex_px if not math.isnan(ex_px) else 0.0)**2 +
                             (ey_px if not math.isnan(ey_px) else 0.0)**2)
            self.max_residual_cm = max(self.max_residual_cm, ed_cm)
            self.max_residual_px = max(self.max_residual_px, e_px)

        # freshness
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        fresh = 1 if ((now_ros - self.last_hdr_time) <= self.fresh_win) else 0

        # write timeseries row
        vx, vy, wz = self.last_cmd
        self._ts_csv.writerow([
            f"{t:.3f}", fresh,
            f"{u:.3f}" if not math.isnan(u) else "", f"{v:.3f}" if not math.isnan(v) else "",
            f"{ex_px:.3f}" if not math.isnan(ex_px) else "", f"{ey_px:.3f}" if not math.isnan(ey_px) else "",
            f"{Z:.3f}" if not math.isnan(Z) else "",
            f"{x_base:.3f}" if not math.isnan(x_base) else "", f"{y_base:.3f}" if not math.isnan(y_base) else "",
            f"{yaw_err:.5f}" if not math.isnan(yaw_err) else "", f"{dist_err:.3f}" if not math.isnan(dist_err) else "",
            int(currently_in_band),
            f"{vx:.3f}", f"{vy:.3f}", f"{wz:.3f}"
        ])
        self._ts_file.flush()

    # ---------- End & metrics ----------
    def finish_and_exit(self, success: bool):
        # Compute metrics per Table~\ref{tab:metrics}
        # 1) Load timeseries back into numpy for convenience
        try:
            self._ts_file.flush()
        except Exception:
            pass

        rows = []
        with open(self.timeseries_path, 'r') as f:
            r = csv.DictReader(f)
            for row in r:
                rows.append(row)

        def parsef(row, key):
            try:
                s = row[key]
                return float(s) if s not in ("", "nan", "NaN") else float('nan')
            except Exception:
                return float('nan')

        t = np.array([float(row['t']) for row in rows])
        ex = np.array([parsef(row,'ex_px') for row in rows])
        ey = np.array([parsef(row,'ey_px') for row in rows])
        x  = np.array([parsef(row,'x_base') for row in rows])
        yaw= np.array([parsef(row,'yaw_err') for row in rows])
        band = np.array([int(row['in_band']) for row in rows])
        vx = np.array([parsef(row,'cmd_vx') for row in rows])
        vy = np.array([parsef(row,'cmd_vy') for row in rows])
        fresh = np.array([int(row['fresh']) for row in rows])

        # Entry time T (relative)
        if self.entry_time is not None:
            T = self.entry_time - self.t0
        else:
            # never entered; pick NaN
            T = float('nan')

        # Entry accuracy at T
        def value_at_T(arr):
            if len(t)==0 or math.isnan(T): return float('nan')
            k = int(np.argmin(np.abs(t - T)))
            return arr[k] if k < len(arr) else float('nan')

        exT = value_at_T(ex); eyT = value_at_T(ey)
        # Use distance error at T in centimeters
        d_err_cm_T = float('nan')
        if not math.isnan(T) and len(x)>0:
            k = int(np.argmin(np.abs(t - T))); 
            if k < len(x) and not math.isnan(x[k]):
                d_err_cm_T = abs((x[k]-self.stop_d)*100.0)

        # Jerk proxy of filtered base velocities (use second diff on vx,vy)
        jerk = None
        if len(vx) >= 3:
            dt = np.diff(t)
            dt = np.where(dt<=0.0, 1e-3, dt)
            # second difference divided by dt^2 for each axis
            def second_diff(v):
                return (v[2:] - 2*v[1:-1] + v[:-2]) / (dt[1:]*dt[1:])
            jx = second_diff(vx)
            jy = second_diff(vy)
            # align lengths (t[2:-])
            jerk = np.sqrt(jx*jx + jy*jy)
            jerk_proxy = float(np.nanmean(jerk)) if jerk.size>0 else float('nan')
        else:
            jerk_proxy = float('nan')

        # Stop-band stability: time-to-entry, dwell achieved, residual motion
        time_to_entry = T if not math.isnan(T) else float('nan')

        # dwell achieved: from first time entry condition true until first leave
        dwell_achieved = float('nan')
        if not math.isnan(T):
            # find first index where band condition holds after T, then count contiguous hold
            idx_T = int(np.argmin(np.abs(t - T)))
            i = idx_T
            while i < len(t) and band[i] == 1: i += 1
            if i > idx_T:
                dwell_achieved = float(t[i-1] - t[idx_T])

        # residual motion after entry (pixels+cm max already tracked, but recompute defensively)
        residual_cm = 0.0
        residual_px = 0.0
        if not math.isnan(T):
            idx_T = int(np.argmin(np.abs(t - T)))
            # consider only while in band
            inband_after_T = (band[idx_T:] == 1)
            if inband_after_T.any():
                # distance error in cm
                d_cm = np.abs((x[idx_T:] - self.stop_d) * 100.0)
                d_cm[~inband_after_T] = 0.0
                residual_cm = float(np.nanmax(d_cm))
                # pixel residual
                e_px = np.sqrt(np.nan_to_num(ex[idx_T:])**2 + np.nan_to_num(ey[idx_T:])**2)
                e_px[~inband_after_T] = 0.0
                residual_px = float(np.nanmax(e_px))
            else:
                residual_cm = float('nan'); residual_px = float('nan')
        else:
            residual_cm = float('nan'); residual_px = float('nan')

        # Freshness effectiveness
        fresh_frac = float(np.mean(fresh)) if len(fresh)>0 else float('nan')

        # Write metrics.csv (one row)
        with open(self.metrics_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'trial_id','entry_ex_px','entry_ey_px','entry_ed_cm',
                'time_to_entry_s','dwell_achieved_s','residual_inband_cm',
                'residual_inband_px','jerk_proxy_mps3','fresh_fraction'
            ])
            w.writerow([
                self.trial_id,
                f"{exT:.2f}" if not math.isnan(exT) else "",
                f"{eyT:.2f}" if not math.isnan(eyT) else "",
                f"{d_err_cm_T:.1f}" if not math.isnan(d_err_cm_T) else "",
                f"{time_to_entry:.2f}" if not math.isnan(time_to_entry) else "",
                f"{dwell_achieved:.2f}" if not math.isnan(dwell_achieved) else "",
                f"{residual_cm:.1f}" if not math.isnan(residual_cm) else "",
                f"{residual_px:.1f}" if not math.isnan(residual_px) else "",
                f"{jerk_proxy:.4f}" if not math.isnan(jerk_proxy) else "",
                f"{fresh_frac:.3f}" if not math.isnan(fresh_frac) else "",
            ])

        self.get_logger().info(f"Saved metrics -> {self.metrics_path}")
        try:
            self._ts_file.close()
        except Exception:
            pass
        # exit node
        rclpy.shutdown()

def main():
    rclpy.init()
    n = CSRLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    try:
        n.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()
