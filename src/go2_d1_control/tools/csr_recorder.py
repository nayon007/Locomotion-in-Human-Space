#!/usr/bin/env python3
import os, csv, math, time, argparse, datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def norm2(x,y): return math.hypot(x,y)

class CsrRecorder(Node):
    def __init__(self, args):
        super().__init__('csr_recorder')
        self.args = args
        self.out_dir = os.path.abspath(args.out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base = os.path.join(self.out_dir, f'csr_{args.label}_{ts}')
        self.csv_path = self.base + '.csv'
        self.meta_path= self.base + '.meta.txt'

        self.stop_band = float(args.stop_band)
        self.v_thresh  = float(args.v_thresh)
        self.settle_window = float(args.settle_window)

        self.have_odom = False
        self.last_pose = None
        self.last_t = None
        self.win = []   # sliding window for settle check (speed history)

        # subscribers
        self.sub_pose = self.create_subscription(PoseStamped, args.target_topic, self.on_pose, 30)
        if args.odom_topic:
            self.sub_odom = self.create_subscription(Odometry, args.odom_topic, self.on_odom, 30)
        else:
            self.sub_odom = None

        # odom cache
        self.vx = self.vy = 0.0

        # state markers
        self.t0 = None
        self.entry_t = None
        self.settle_t = None

        # csv
        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow([
            't','x','y','z','r','speed','jerk','band','label',
            'odom_vx','odom_vy'
        ])
        self.get_logger().info(f'CSR recorder: writing {self.csv_path}')

    def on_odom(self, msg: Odometry):
        self.have_odom = True
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

    def on_pose(self, msg: PoseStamped):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None: self.t0 = t
        t_rel = t - self.t0

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        r = math.sqrt(x*x + y*y)

        # derive speed from odom if available, else from relative range change
        if self.have_odom:
            speed = math.hypot(self.vx, self.vy)
        else:
            if self.last_pose is None:
                speed = 0.0
            else:
                dt = max(1e-3, t - self.last_t)
                # use change in (x,y) to estimate relative closing speed
                dx = x - self.last_pose[0]
                dy = y - self.last_pose[1]
                speed = math.hypot(dx, dy) / dt

        # jerk proxy: derivative of speed
        if self.last_pose is None:
            jerk = 0.0
        else:
            dt = max(1e-3, t - self.last_t)
            jerk = (speed - self.last_pose[3]) / dt

        # band entry time
        in_band = (r <= self.stop_band)
        if in_band and self.entry_t is None:
            self.entry_t = t_rel

        # settle time once in band and speed low for a window
        self.win.append((t_rel, speed))
        # keep only last settle_window seconds
        t_cut = t_rel - self.settle_window
        while self.win and self.win[0][0] < t_cut:
            self.win.pop(0)
        if self.entry_t is not None and self.settle_t is None:
            if self.win and max(s for _,s in self.win) < self.v_thresh:
                self.settle_t = t_rel

        self.w.writerow([f'{t_rel:.3f}', x, y, z, r, speed, jerk, int(in_band), self.args.label, self.vx, self.vy])
        self.last_pose = (x,y,z,t_rel,speed,jerk,in_band)
        self.last_t = t

    def close(self):
        try: self.f.flush(); self.f.close()
        except: pass
        # compute residual motion after settle
        resid_rms = float('nan')
        dwell = float('nan')
        if self.settle_t is not None and self.last_pose is not None:
            # read back speeds after settle
            import pandas as pd
            df = pd.read_csv(self.csv_path)
            df = df[df['t'] >= self.settle_t]
            if len(df) >= 3:
                resid_rms = float((df['speed']**2).mean()**0.5)
                dwell = float(df['t'].iloc[-1] - self.settle_t)
        with open(self.meta_path, 'w') as m:
            m.write(f'label={self.args.label}\n')
            m.write(f'stop_band={self.stop_band}\n')
            m.write(f'v_thresh={self.v_thresh}\n')
            m.write(f'settle_window={self.settle_window}\n')
            m.write(f'entry_t={self.entry_t}\n')
            m.write(f'settle_t={self.settle_t}\n')
            m.write(f'dwell={dwell}\n')
            m.write(f'residual_rms={resid_rms}\n')
        self.get_logger().info(f'Wrote meta: {self.meta_path}')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target_topic', default='/perception/bottle_pose_base')
    parser.add_argument('--odom_topic',   default='/odom')   # leave empty to disable
    parser.add_argument('--label',        default='csr_on', help='csr_on or baseline')
    parser.add_argument('--out_dir',      default=os.path.expanduser('~/go2_ws/src/go2_d1_control/results'))
    parser.add_argument('--stop_band',    type=float, default=0.70)
    parser.add_argument('--v_thresh',     type=float, default=0.06)
    parser.add_argument('--settle_window',type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = CsrRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
