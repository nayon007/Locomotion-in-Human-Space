#!/usr/bin/env python3
import numpy as np, cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_msgs.msg import String, Bool, Header
from tf2_ros import Buffer, TransformListener

# [ADD] publish pixel-centering error each frame
from geometry_msgs.msg import Point32  # <—

# Optional deps
try:
    from cv_bridge import CvBridge
    _HAS_BRIDGE = True
except Exception:
    _HAS_BRIDGE = False

try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except Exception:
    _HAS_YOLO = False

try:
    from tf2_geometry_msgs import do_transform_pose
    _HAS_TF2_GEOM = True
except Exception:
    _HAS_TF2_GEOM = False


def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


class BottleHumanDetector(Node):
    def __init__(self):
        super().__init__('bottle_human_detector')

        # Topics
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')

        # Detection
        self.declare_parameter('mode', 'yolo')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_imgsz', 512)
        self.declare_parameter('conf_bottle', 0.35)
        self.declare_parameter('conf_person', 0.35)
        self.declare_parameter('require_holding', False)
        self.declare_parameter('accept_labels', 'bottle,cup,can,vase')

        # Frames
        self.declare_parameter('optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        # Orientation to publish
        self.declare_parameter('publish_orientation_yaw', 0.0)

        # Light smoothing of 3D point
        self.declare_parameter('xyz_ema_alpha', 0.3)

        # Resolve params
        img_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.mode = self.get_parameter('mode').value
        self.accept_labels = {s.strip().lower() for s in self.get_parameter('accept_labels').value.split(',')}
        self.optical_frame = self.get_parameter('optical_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.alpha = float(self.get_parameter('xyz_ema_alpha').value)

        # State
        self.bridge = CvBridge() if _HAS_BRIDGE else None
        self.cam_info = None
        self.depth_frame = None
        self.last_xyz_optical = None  # EMA state

        # Subs
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, img_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data)

        # Pubs
        self.pose_cam_pub  = self.create_publisher(PoseStamped, '/perception/bottle_pose', 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, '/perception/bottle_pose_base', 10)
        self.header_pub    = self.create_publisher(Header, '/perception/bottle_pose/header', 10)
        self.debug_pub     = self.create_publisher(Image, '/perception/debug_image', 10)
        self.status_pub    = self.create_publisher(String, '/perception/status', 10)
        self.ready_pub     = self.create_publisher(Bool, '/perception/handover_ready', 10)
        self.center_px_pub = self.create_publisher(Point32, '/perception/centering_px', 10)  # [ADD]

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO
        self.yolo = None
        if self.mode == 'yolo' and _HAS_YOLO:
            try:
                self.yolo = YOLO(self.get_parameter('yolo_model').value)
                self.names = getattr(getattr(self.yolo, 'model', self.yolo), 'names', {})
                self.get_logger().info(f"Loaded YOLO model: {self.get_parameter('yolo_model').value}")
            except Exception as e:
                self.get_logger().error(f"YOLO load failed: {e}")
        elif self.mode == 'yolo':
            self.get_logger().warn("ultralytics not installed. `pip3 install --user ultralytics`")

        self.get_logger().info(
            f"Perception started. img:={img_topic} info:={info_topic} depth:={depth_topic} "
            f"| mode={self.mode} | frames: optical={self.optical_frame}, base={self.base_frame}"
        )

    # ---------- Callbacks ----------
    def on_info(self, msg: CameraInfo):
        self.cam_info = msg

    def on_depth(self, msg: Image):
        if not self.bridge:
            return
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            self.depth_frame = None

    def on_image(self, image_msg: Image):
        self.status_pub.publish(String(data="image_rx"))
        if not self.bridge:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        detections, holding = [], False

        if self.mode == 'yolo' and self.yolo is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            try:
                res = self.yolo.predict(rgb, imgsz=int(self.get_parameter('yolo_imgsz').value),
                                        conf=0.20, verbose=False)[0]
                if res.boxes is not None:
                    xyxy = res.boxes.xyxy.cpu().numpy()
                    cls  = res.boxes.cls.cpu().numpy().astype(int)
                    conf = res.boxes.conf.cpu().numpy()
                    conf_p = float(self.get_parameter('conf_person').value)
                    conf_b = float(self.get_parameter('conf_bottle').value)
                    ppl, obj = [], []
                    for bb, c, cf in zip(xyxy, cls, conf):
                        label = (self.names.get(int(c), str(c)) if isinstance(self.names, dict)
                                 else self.names[int(c)]).lower()
                        if label == 'person' and cf >= conf_p:
                            ppl.append((bb, cf))
                        elif (label in self.accept_labels) and cf >= conf_b:
                            obj.append((bb, cf, label))
                    for bb, cf, lbl in obj:
                        x0,y0,x1,y1 = bb
                        cx, cy = 0.5*(x0+x1), 0.5*(y0+y1)
                        is_hold = False
                        for pb,_ in ppl:
                            px0,py0,px1,py1 = pb
                            pw, ph = (px1-px0), (py1-py0)
                            ex, ey = 0.1*pw, 0.2*ph
                            if (px0-ex) <= cx <= (px1+ex) and (py0-ey) <= cy <= (py1+ey):
                                is_hold = True; holding = True; break
                        detections.append({'bb':bb, 'conf':float(cf), 'label':lbl, 'holding':is_hold})
                    # overlays
                    for pb,_ in ppl:
                        x0,y0,x1,y1 = pb.astype(int); cv2.rectangle(frame,(x0,y0),(x1,y1),(0,150,255),2)
                    for d in detections:
                        x0,y0,x1,y1 = d['bb'].astype(int)
                        color = (0,255,0) if d['holding'] else (0,0,255)
                        cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)
                        cv2.putText(frame,f"{d['label']}{'(hold)' if d['holding'] else ''}",
                                    (x0,max(0,y0-6)),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1,cv2.LINE_AA)
            except Exception as e:
                self.get_logger().warn(f"YOLO inference failed: {e}")

        # Debug image + ready flag
        try: self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception: pass
        self.ready_pub.publish(Bool(data=bool(holding)))

        # Choose a target
        target = None
        if detections:
            if self.get_parameter('require_holding').value:
                preferred = [d for d in detections if d['holding']]
                if not preferred: preferred = detections
            else:
                preferred = detections
            target = max(preferred, key=lambda d: d['conf'])
        if target is None or self.cam_info is None or self.depth_frame is None:
            return

        # Depth median near bbox center
        x0,y0,x1,y1 = target['bb']
        H, W = self.depth_frame.shape[:2]
        x0,y0,x1,y1 = map(int,[max(0,x0),max(0,y0),min(W-1,x1),min(H-1,y1)])
        w,h = max(1,x1-x0), max(1,y1-y0)
        cx, cy = x0+w//2, y0+h//2

        # [ADD] publish pixel centering error every frame for the logger
        try:
            cx0, cy0 = self.cam_info.k[2], self.cam_info.k[5]
            ex_px, ey_px = float(cx - cx0), float(cy - cy0)
            self.center_px_pub.publish(Point32(x=ex_px, y=ey_px, z=0.0))
        except Exception:
            pass

        rx, ry = max(0,cx-max(2,w//10)), max(0,cy-max(2,h//10))
        rx2,ry2= min(W,cx+max(3,w//10)), min(H,cy+max(3,h//10))
        roi = self.depth_frame[ry:ry2, rx:rx2]
        if roi.size == 0: return
        roi_f = roi[np.isfinite(roi)]; roi_f = roi_f[roi_f > 0.0]
        if roi_f.size == 0: return
        Zmm = float(np.median(roi_f))
        Z = Zmm*1e-3 if Zmm > 10.0 else Zmm
        if not (0.05 < Z < 4.0): return

        # Back-project in optical frame
        fx, fy = self.cam_info.k[0], self.cam_info.k[4]
        cx0, cy0 = self.cam_info.k[2], self.cam_info.k[5]
        X = (cx - cx0) * Z / fx
        Y = (cy - cy0) * Z / fy

        # EMA (optical frame)
        if self.last_xyz_optical is None:
            Xs, Ys, Zs = X, Y, Z
        else:
            a = float(self.alpha)
            Xs = a * X + (1 - a) * self.last_xyz_optical[0]
            Ys = a * Y + (1 - a) * self.last_xyz_optical[1]
            Zs = a * Z + (1 - a) * self.last_xyz_optical[2]
        self.last_xyz_optical = (Xs, Ys, Zs)

        # Pose in optical frame
        ps = PoseStamped()
        ps.header.frame_id = self.optical_frame
        ps.header.stamp = self.cam_info.header.stamp if self.cam_info.header.stamp.sec != 0 else image_msg.header.stamp
        ps.pose.position = Point(x=float(Xs), y=float(Ys), z=float(Zs))
        qx,qy,qz,qw = quaternion_from_euler(0.0, 0.0, float(self.get_parameter('publish_orientation_yaw').value))
        ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Publish in camera frame
        self.pose_cam_pub.publish(ps)

        # Publish in base frame (robust even without tf2_geometry_msgs)
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, ps.header.frame_id, rclpy.time.Time())
            if _HAS_TF2_GEOM:
                pose_in_base = do_transform_pose(ps.pose, tf)
                out = PoseStamped()
                out.header.stamp = ps.header.stamp
                out.header.frame_id = self.base_frame
                out.pose = pose_in_base
            else:
                # Manual point transform (orientation just copied)
                q = tf.transform.rotation; t = tf.transform.translation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([
                    [1-2*(y*y+z*z), 2*(x*y-w*z),     2*(x*z+w*y)],
                    [2*(x*y+w*z),   1-2*(x*x+z*z),   2*(y*z-w*x)],
                    [2*(x*z-w*y),   2*(y*z+w*x),     1-2*(x*x+y*y)]
                ])
                p = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                pb = R @ p + np.array([t.x, t.y, t.z])
                out = PoseStamped()
                out.header.stamp = ps.header.stamp
                out.header.frame_id = self.base_frame
                out.pose.position = Point(x=float(pb[0]), y=float(pb[1]), z=float(pb[2]))
                out.pose.orientation = ps.pose.orientation
            self.pose_base_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"TF {ps.header.frame_id} -> {self.base_frame} failed: {e}")

        # Freshness header (for the bridge)
        try:
            hdr = Header(); hdr.stamp = ps.header.stamp
            self.header_pub.publish(hdr)
        except Exception:
            pass

        # Log (camera-frame)
        self.get_logger().info(
            f"bottle @ {ps.header.frame_id}: x={ps.pose.position.x:.2f} y={ps.pose.position.y:.2f} z={ps.pose.position.z:.2f}"
        )


def main():
    rclpy.init()
    node = BottleHumanDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

