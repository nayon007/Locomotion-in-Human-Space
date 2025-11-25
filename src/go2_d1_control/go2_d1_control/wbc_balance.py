#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_ros import TransformException

# Optional QP solver
_HAS_OSQP = False
try:
    import osqp
    from scipy import sparse
    _HAS_OSQP = True
except Exception:
    _HAS_OSQP = False

# -------------------------------------------------------------------

FOOT_LINKS = ['FL_foot', 'FR_foot', 'RR_foot', 'RL_foot']  # CCW order preferred

QOS_BE1 = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

def clamp(x, lo, hi): return max(lo, min(hi, x))

def edges_ccw(pts):
    """Return CCW edges (p_i -> p_{i+1}) for polygon points pts (Nx2)."""
    return [(pts[i], pts[(i+1) % len(pts)]) for i in range(len(pts))]

def halfspace_from_edge(p0, p1):
    """
    For CCW edge from p0 to p1, inward normal n points to polygon interior.
    Using 2D normal n = (dy, -dx) normalized.
    Returns (n, b) such that n^T x <= b is interior.
    """
    dx, dy = (p1[0]-p0[0]), (p1[1]-p0[1])
    n = np.array([dy, -dx], dtype=float)
    n_norm = np.linalg.norm(n)
    if n_norm < 1e-9:
        return np.array([0.0, 0.0]), -1e9
    n = n / n_norm
    b = np.dot(n, p0)
    return n, b

def project_halfspace(v, a, b):
    """
    Project vector v (R^3) onto halfspace a^T v <= b (with a in R^3).
    """
    # violation = a^T v - b
    viol = float(np.dot(a, v) - b)
    if viol <= 0.0:
        return v
    denom = float(np.dot(a, a))
    if denom < 1e-12:
        return v
    return v - (viol / denom) * a

# -------------------------------------------------------------------

class WBCBalance(Node):
    """
    QP-based base-velocity stabilizer:
      - Input:  desired /cmd_vel  (Twist)
      - Output: stabilized /cmd_vel_stab (Twist)
      - Constraints: keep predicted CoM projection within a shrunken support polygon,
                     respect bounds and tilt-yaw damping constraints.

    Notes:
      * This is a quasi-static stabilizer at the velocity (twist) level.
      * It does not command leg torques — it filters the base motion for safety/comfort.
      * Uses OSQP if present; otherwise falls back to a projection loop.
    """
    def __init__(self):
        super().__init__('wbc_balance')

        # --------------- Parameters ---------------
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('foot_links', FOOT_LINKS)
        self.declare_parameter('publish_rate_hz', 50.0)

        # velocity limits
        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('max_vy', 0.20)
        self.declare_parameter('max_wz', 0.60)

        # cost weights (closer to desired)
        self.declare_parameter('w_vx', 1.0)
        self.declare_parameter('w_vy', 1.0)
        self.declare_parameter('w_wz', 0.5)
        self.declare_parameter('w_wz_damp', 0.2)  # penalize large |wz|

        # support polygon shrink (safety margin) in meters
        self.declare_parameter('poly_shrink_m', 0.02)

        # CoM proxy and prediction horizon
        self.declare_parameter('com_offset_x', 0.02)  # proxy: base_link origin slightly forward
        self.declare_parameter('com_offset_y', 0.00)
        self.declare_parameter('predict_dt', 0.10)    # predict CoM shift over dt

        # mapping CoM shift from twist (simple quasi-static)
        self.declare_parameter('beta_x', 1.0)  # dCoM_x ≈ beta_x * vx * dt
        self.declare_parameter('beta_y', 1.0)  # dCoM_y ≈ beta_y * vy * dt

        # IMU stabilization
        self.declare_parameter('use_imu', True)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('kd_wz', 0.2)   # yaw-rate damping
        self.declare_parameter('tilt_limit_deg', 15.0)  # hard limit (zero vx above)
        self.declare_parameter('tilt_soft_deg', 5.0)    # start scaling vx

        # RViz markers
        self.declare_parameter('publish_markers', True)

        # --------------- Resolve params ---------------
        gp = lambda n: self.get_parameter(n).value
        self.base_frame = str(gp('base_frame'))
        self.foot_links = list(gp('foot_links'))

        self.max_vx = float(gp('max_vx')); self.max_vy = float(gp('max_vy')); self.max_wz = float(gp('max_wz'))
        self.W = np.diag([float(gp('w_vx')), float(gp('w_vy')), float(gp('w_wz'))]).astype(float)
        self.w_wz_damp = float(gp('w_wz_damp'))

        self.shrink = float(gp('poly_shrink_m'))
        self.dt = float(gp('predict_dt'))
        self.beta_x = float(gp('beta_x')); self.beta_y = float(gp('beta_y'))
        self.com_off = np.array([float(gp('com_offset_x')), float(gp('com_offset_y'))], dtype=float)

        self.use_imu = bool(gp('use_imu'))
        self.imu_topic = str(gp('imu_topic'))
        self.kd_wz = float(gp('kd_wz'))
        self.tilt_limit = math.radians(float(gp('tilt_limit_deg')))
        self.tilt_soft  = math.radians(float(gp('tilt_soft_deg')))

        self.pub_markers = bool(gp('publish_markers'))
        self.Ts = 1.0 / float(gp('publish_rate_hz'))

        # --------------- IO ---------------
        self.tf_buf = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_buf, self)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        if self.use_imu:
            self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.on_imu, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_stab', 10)

        if self.pub_markers:
            self.pub_com = self.create_publisher(Marker, '/wbc/com_marker', 10)
            self.pub_poly = self.create_publisher(Marker, '/wbc/support_polygon', 10)

        self.timer = self.create_timer(self.Ts, self.tick)

        # --------------- State ---------------
        self.v_des = np.zeros(3)  # desired [vx, vy, wz]
        self.last_imu_wz = 0.0
        self.last_tilt = 0.0

        self.get_logger().info(
            f"WBCBalance (QP stabilizer) — OSQP={'ON' if _HAS_OSQP else 'OFF (fallback)'} | "
            f"base={self.base_frame} | feet={self.foot_links}"
        )

    # ---------- Subscribers ----------
    def on_cmd(self, msg: Twist):
        self.v_des = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)

    def on_imu(self, msg: Imu):
        self.last_imu_wz = float(msg.angular_velocity.z)
        q = msg.orientation
        # roll, pitch magnitude (no yaw) for tilt
        sinr_cosp = 2*(q.w*q.x + q.y*q.z)
        cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2*(q.w*q.y - q.z*q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.last_tilt = math.hypot(roll, pitch)

    # ---------- TF helpers ----------
    def get_feet_xy(self):
        pts = []
        for fl in self.foot_links:
            try:
                tf = self.tf_buf.lookup_transform(self.base_frame, fl, Time())  # latest
                pts.append([tf.transform.translation.x, tf.transform.translation.y])
            except TransformException:
                pass
        return np.array(pts, dtype=float) if len(pts) >= 3 else None

    # ---------- QP build ----------
    def build_polygon_halfspaces(self, feet_xy):
        """
        Given Nx2 polygon points in roughly CCW order,
        return (A, b) for n_i^T p <= b_i (interior) with shrink.
        """
        A = []; b = []
        for p0, p1 in edges_ccw(feet_xy):
            n, bi = halfspace_from_edge(p0, p1)
            # shrink inward by 'shrink' meters along inward normal
            bi_shrunk = bi - self.shrink
            A.append(n); b.append(bi_shrunk)
        return np.array(A, dtype=float), np.array(b, dtype=float)

    def com_predicted(self, com_xy, v):
        # Predict CoM shift in dt from base twist (quasi-static proxy)
        d = np.array([self.beta_x * v[0] * self.dt,
                      self.beta_y * v[1] * self.dt], dtype=float)
        return com_xy + d

    # ---------- Main ----------
    def tick(self):
        # Desired command
        v_des = self.v_des.copy()

        # IMU yaw damping and tilt gating
        if self.use_imu:
            v_des[2] -= self.kd_wz * self.last_imu_wz
            if self.last_tilt >= self.tilt_limit:
                v_des[0] = 0.0
            elif self.last_tilt > self.tilt_soft:
                s = 1.0 - (self.last_tilt - self.tilt_soft) / (self.tilt_limit - self.tilt_soft)
                v_des[0] *= clamp(s, 0.0, 1.0)

        # Velocity bounds (hard)
        vmin = np.array([-self.max_vx, -self.max_vy, -self.max_wz], dtype=float)
        vmax = np.array([ self.max_vx,  self.max_vy,  self.max_wz], dtype=float)

        # Support polygon from feet
        feet = self.get_feet_xy()
        if feet is None:
            # No TF → just clamp and publish
            v = np.clip(v_des, vmin, vmax)
            self.publish_cmd(v)
            if self.pub_markers:
                self.publish_markers(None, None)
            return

        # Build halfspaces for polygon (inward)
        A_poly, b_poly = self.build_polygon_halfspaces(feet)

        # Current CoM proxy (base_link origin + offset)
        com_now = self.com_off.copy()

        # We want predicted CoM (with v) to satisfy: n_i^T p_pred <= b_i
        # p_pred = com_now + [beta_x*vx*dt, beta_y*vy*dt]
        # => n_i^T ([beta_x*dt, beta_y*dt, 0] * v) <= b_i - n_i^T com_now
        A_v = []
        u_v = []
        for n, bi in zip(A_poly, b_poly):
            a_row = np.array([n[0]*self.beta_x*self.dt, n[1]*self.beta_y*self.dt, 0.0], dtype=float)
            rhs = float(bi - np.dot(n, com_now))
            A_v.append(a_row); u_v.append(rhs)
        A_v = np.array(A_v, dtype=float)   # shape Mx3
        u_v = np.array(u_v, dtype=float)

        # Build QP:
        #   minimize  (v - v_des)^T W (v - v_des) + w_wz_damp * wz^2
        #   subject to A_v v <= u_v
        #              vmin <= v <= vmax
        # If OSQP present, solve properly; else fallback.

        if _HAS_OSQP:
            P = self.W.copy()
            P[2,2] += self.w_wz_damp
            q = - (self.W @ v_des)

            # Inequalities:
            # 1) polygon: A_v v <= u_v
            # 2) box:     v <= vmax   ->  I v <= vmax
            #             -v <= -vmin -> -I v <= -vmin
            A_list = [A_v,  np.eye(3), -np.eye(3)]
            u_list = [u_v,  vmax,      -vmin]
            A = np.vstack(A_list)
            u = np.hstack(u_list)
            l = -np.inf * np.ones_like(u)

            P_sp = sparse.csc_matrix((P + P.T) * 0.5)  # ensure symmetric
            A_sp = sparse.csc_matrix(A)
            prob = osqp.OSQP()
            prob.setup(P=P_sp, q=q, A=A_sp, l=l, u=u, verbose=False, polish=True)
            res = prob.solve()
            if res.x is None:
                v = np.clip(v_des, vmin, vmax)
            else:
                v = np.clip(res.x.astype(float), vmin, vmax)
        else:
            # Fallback: unconstrained => clamp => project onto halfspaces
            v = np.clip(v_des, vmin, vmax)
            # Iterative projection onto halfspaces (POCS)
            a_rows = []
            b_rows = []
            for i in range(A_v.shape[0]):
                a = np.array([A_v[i,0], A_v[i,1], A_v[i,2]], dtype=float)
                b = float(u_v[i])
                a_rows.append(a); b_rows.append(b)
            # 2-3 passes are usually enough for these gentle constraints
            for _ in range(3):
                for a, b in zip(a_rows, b_rows):
                    v = project_halfspace(v, a, b)
                v = np.clip(v, vmin, vmax)

        # Publish stabilized command
        self.publish_cmd(v)

        # Markers
        if self.pub_markers:
            # support polygon
            self.publish_markers(feet, com_now)

    # ---------- Publishers ----------
    def publish_cmd(self, v):
        msg = Twist()
        msg.linear.x = float(v[0])
        msg.linear.y = float(v[1])
        msg.angular.z = float(v[2])
        self.pub_cmd.publish(msg)

    def publish_markers(self, feet, com_now):
        # Support polygon
        poly = Marker()
        poly.header.frame_id = self.base_frame
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.ns = "wbc"
        poly.id = 1
        poly.type = Marker.LINE_STRIP
        poly.action = Marker.ADD
        poly.scale.x = 0.01
        poly.color.a = 0.9
        poly.color.b = 1.0

        if feet is not None and len(feet) >= 3:
            order = [i for i in range(len(feet))]
            for i in order + [order[0]]:
                p = feet[i]
                pt = Point(); pt.x = float(p[0]); pt.y = float(p[1]); pt.z = 0.0
                poly.points.append(pt)
        self.pub_poly.publish(poly)

        # CoM proxy
        com = Marker()
        com.header.frame_id = self.base_frame
        com.header.stamp = poly.header.stamp
        com.ns = "wbc"
        com.id = 2
        com.type = Marker.SPHERE
        com.action = Marker.ADD
        com.scale.x = 0.05; com.scale.y = 0.05; com.scale.z = 0.05
        com.color.a = 0.9; com.color.g = 1.0
        if com_now is not None:
            com.pose.position.x = float(com_now[0]); com.pose.position.y = float(com_now[1]); com.pose.position.z = 0.0
        self.pub_com.publish(com)

# -------------------------------------------------------------------

def main():
    rclpy.init()
    node = WBCBalance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

