#!/usr/bin/env python3
import os, csv, time, math, statistics
import rclpy, numpy as np
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, Float32
import tf2_ros
from tf2_ros import TransformException

FOOT_LINKS = ['FL_foot','FR_foot','RR_foot','RL_foot']

def edges_ccw(pts): return [(pts[i], pts[(i+1)%len(pts)]) for i in range(len(pts))]
def halfspace(p0,p1):
    dx,dy = (p1[0]-p0[0]), (p1[1]-p0[1])
    n = np.array([dy,-dx],float); n_norm = np.linalg.norm(n)
    if n_norm<1e-9: return np.array([0.0,0.0]), -1e9
    n/=n_norm; b = float(np.dot(n,p0)); return n,b

class WBCLogger(Node):
    def __init__(self):
        super().__init__('wbc_logger')
        # params
        self.declare_parameter('mode','raw')                 # raw|pocs|osqp (for folder naming)
        self.declare_parameter('data_dir','wbc_data')        # relative to $HOME/go2_ws (or absolute)
        self.declare_parameter('base_frame','base_link')
        self.declare_parameter('foot_links', FOOT_LINKS)
        self.declare_parameter('tilt_limit_deg', 15.0)       # match stabilizer
        self.declare_parameter('poly_shrink_m', 0.02)        # match stabilizer
        self.declare_parameter('log_rate_hz', 50.0)

        gp = lambda n: self.get_parameter(n).value
        self.mode = str(gp('mode')).lower()
        self.base = str(gp('base_frame'))
        self.feet_names = list(gp('foot_links'))
        self.tilt_lim = float(gp('tilt_limit_deg'))
        self.shrink   = float(gp('poly_shrink_m'))
        self.dt = 1.0/float(gp('log_rate_hz'))

        # dirs
        root = str(gp('data_dir'))
        if not os.path.isabs(root):
            root = os.path.join(os.path.expanduser('~'), 'go2_ws', root)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.trial_dir = os.path.join(root, f'{stamp}_{self.mode}')
        os.makedirs(self.trial_dir, exist_ok=True)
        self.ts_path = os.path.join(self.trial_dir, 'timeseries.csv')
        self.mx_path = os.path.join(self.trial_dir, 'metrics.csv')

        # IO
        self.tf = tf2_ros.TransformListener(tf2_ros.Buffer(), self)
        self.tfbuf: tf2_ros.Buffer = self.tf.buffer
        self.sub_des  = self.create_subscription(Twist, '/wbc_eval/cmd_des',  self.on_des,  50)
        self.sub_out  = self.create_subscription(Twist, '/wbc_eval/cmd_out',  self.on_out,  50)
        self.sub_stab = self.create_subscription(Twist, '/cmd_vel_stab',      self.on_stab, 50)  # optional tap
        self.sub_imu  = self.create_subscription(Imu,   '/imu/data',          self.on_imu,  50)
        self.sub_hold = self.create_subscription(Bool,  '/approach/ready',    self.on_hold, 10)
        self.sub_mode = self.create_subscription(String,'/wbc_eval/mode',      self.on_mode, 10)
        # optional diag from stabilizer (if instrumented)
        self.sub_solver = self.create_subscription(String,  '/wbc/diag/solver',   self.on_solver,   10)
        self.sub_ms     = self.create_subscription(Float32, '/wbc/diag/solve_ms', self.on_solve_ms, 50)

        # state
        self.latest_des  = Twist(); self.latest_out = Twist(); self.latest_stab = Twist()
        self.imu_wz = 0.0; self.tilt_deg = 0.0; self.hold = False; self.router_mode = self.mode
        self.solve_ms = float('nan'); self.solver_tag = ''
        self.ms_samples = []; self.fallback_cnt = 0; self.solve_cnt = 0

        # buffers for metrics
        self.rho_min_list = []; self.poly_viols = 0; self.N = 0
        self.tilt_ok = 0
        self.jerk_acc = []  # use out cmd for smoothness proxy
        self.prev_out = None; self.prev_prev_out = None
        self.hold_energy = 0.0; self.hold_N = 0
        self.wz_des_win = []; self.wz_out_win = []

        # write header
        with open(self.ts_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','vx_des','vy_des','wz_des','vx_out','vy_out','wz_out',
                        'imu_wz','tilt_deg','rho_min','poly_violation','hold','solver','solve_ms'])
        self.t0 = time.time()
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"WBC eval logger started: mode={self.mode} dir={self.trial_dir}")

    # ---------- subs ----------
    def on_des(self, m):  self.latest_des  = m
    def on_out(self, m):  self.latest_out  = m
    def on_stab(self, m): self.latest_stab = m  # tap only
    def on_hold(self, b): self.hold = bool(b.data)
    def on_mode(self, s): self.router_mode = s.data
    def on_solver(self, s): 
        self.solver_tag = s.data
        # if stabilizer wanted OSQP but reported 'pocs', count as fallback
        if self.mode=='osqp' and s.data.lower().startswith('pocs'):
            self.fallback_cnt += 1
    def on_solve_ms(self, v):
        self.solve_ms = float(v.data)
        if not math.isnan(self.solve_ms):
            self.ms_samples.append(self.solve_ms); self.solve_cnt += 1

    def on_imu(self, m: Imu):
        self.imu_wz = float(m.angular_velocity.z)
        q = m.orientation
        sinr_cosp = 2*(q.w*q.x + q.y*q.z);  cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2*(q.w*q.y - q.z*q.x); pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.tilt_deg = math.degrees(math.hypot(roll, pitch))

    # ---------- helpers ----------
    def feet_xy(self):
        pts=[]
        for name in self.feet_names:
            try:
                tf = self.tfbuf.lookup_transform(self.base, name, Time())  # latest
                pts.append([tf.transform.translation.x, tf.transform.translation.y])
            except TransformException:
                pass
        return np.array(pts,float) if len(pts)>=3 else None

    def rho_min(self, feet):
        if feet is None: return float('nan'), False
        # shrink polygon inward by self.shrink when building halfspaces
        A=[]; b=[]
        for p0,p1 in edges_ccw(feet):
            n, bi = halfspace(p0,p1)
            A.append(n); b.append(bi - self.shrink)
        A=np.array(A); b=np.array(b)
        # CoM proxy at origin (base) → slack of base origin
        p = np.array([0.02, 0.0])  # same offset as stabilizer default
        s = b - (A@p)
        rmin = float(np.min(s))
        return rmin, (rmin < 0.0)

    # ---------- main ----------
    def tick(self):
        t = time.time() - self.t0

        # metrics that need feet
        feet = self.feet_xy()
        rmin, viol = self.rho_min(feet)

        # jerk proxy on issued command (second diff)
        out = self.latest_out
        v = np.array([out.linear.x, out.linear.y, out.angular.z], float)
        if self.prev_out is not None and self.prev_prev_out is not None:
            a = (v - 2*self.prev_out + self.prev_prev_out) / (self.dt**2)
            self.jerk_acc.append(float(np.linalg.norm(a)))
        self.prev_prev_out = self.prev_out
        self.prev_out = v.copy()

        # hold energy & yaw attenuation window
        if self.hold:
            self.hold_energy += float(np.dot(v, v)) * self.dt
            self.hold_N += 1
            self.wz_des_win.append(abs(self.latest_des.angular.z))
            self.wz_out_win.append(abs(out.angular.z))

        # tilt compliance
        if self.tilt_deg <= self.tilt_lim: self.tilt_ok += 1

        # write row
        with open(self.ts_path, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                f"{t:.3f}",
                f"{self.latest_des.linear.x:.5f}", f"{self.latest_des.linear.y:.5f}", f"{self.latest_des.angular.z:.5f}",
                f"{out.linear.x:.5f}", f"{out.linear.y:.5f}", f"{out.angular.z:.5f}",
                f"{self.imu_wz:.5f}", f"{self.tilt_deg:.3f}",
                f"{rmin:.5f}", int(viol), int(self.hold),
                self.solver_tag, f"{self.solve_ms:.3f}" if not math.isnan(self.solve_ms) else ""
            ])

        # accumulate
        self.N += 1
        if viol: self.poly_viols += 1

        # refresh metrics file occasionally
        if self.N % 50 == 0:
            self.write_metrics()

    def write_metrics(self):
        # basic stats
        rho_min_mean = float('nan'); rho_min_std = float('nan')
        # we didn’t store all rho but we can recompute approx: skip here; user can recompute from timeseries
        pr_violate = (self.poly_viols / max(1,self.N)) * 100.0
        gamma_tilt = (self.tilt_ok / max(1,self.N))
        # jerk ratio etc. are computed across conditions; here store per-trial jerk level
        jerk_mean = statistics.mean(self.jerk_acc) if self.jerk_acc else float('nan')
        # residual motion during hold
        E_hold = self.hold_energy
        # yaw attenuation during hold (RMS ratio)
        if self.wz_des_win and any(self.wz_des_win):
            A_omega = ( (statistics.mean([w**2 for w in self.wz_out_win]))**0.5 /
                        (statistics.mean([w**2 for w in self.wz_des_win]))**0.5 )
        else:
            A_omega = float('nan')
        # solver time
        solve_med = statistics.median(self.ms_samples) if self.ms_samples else float('nan')
        solve_p95 = (np.percentile(self.ms_samples,95) if self.ms_samples else float('nan'))
        fallback_rate = (self.fallback_cnt / max(1,self.solve_cnt)) * 100.0 if self.solve_cnt else 0.0

        rows = [
            ['mode', self.mode],
            ['samples', self.N],
            ['poly_violation_pct', f"{pr_violate:.3f}"],
            ['tilt_compliance', f"{gamma_tilt:.3f}"],
            ['jerk_proxy_mean', f"{jerk_mean:.6f}"],
            ['E_hold', f"{E_hold:.6f}"],
            ['A_omega', f"{A_omega:.6f}"],
            ['solve_ms_median', f"{solve_med:.3f}"],
            ['solve_ms_p95', f"{solve_p95:.3f}"],
            ['fallback_rate_pct', f"{fallback_rate:.2f}"],
        ]
        with open(self.mx_path, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['metric','value']); w.writerows(rows)

def main():
    rclpy.init(); n = WBCLogger()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.write_metrics()
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
