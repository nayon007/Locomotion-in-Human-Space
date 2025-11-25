# (same as your working file)
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import numpy as np, cv2

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

from tf2_ros import Buffer, TransformListener
try:
    from tf2_geometry_msgs import do_transform_pose
    _HAS_TF2_GEOM = True
except Exception:
    _HAS_TF2_GEOM = False

def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll*0.5),  np.sin(roll*0.5)
    cp, sp = np.cos(pitch*0.5), np.sin(pitch*0.5)
    cy, sy = np.cos(yaw*0.5),   np.sin(yaw*0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return (x, y, z, w)

class BottleHumanDetector(Node):
    def __init__(self):
        super().__init__('bottle_human_detector')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('mode', 'yolo')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('conf_bottle', 0.30)
        self.declare_parameter('conf_person', 0.30)
        self.declare_parameter('require_holding', False)
        self.declare_parameter('publish_orientation_yaw', 0.0)
        self.declare_parameter('accept_labels', 'bottle,cup,can,vase')

        img_topic  = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        depth_topic= self.get_parameter('depth_topic').value
        self.mode  = self.get_parameter('mode').value
        self.accept_labels = {s.strip().lower() for s in self.get_parameter('accept_labels').value.split(',')}

        self.bridge = CvBridge() if _HAS_BRIDGE else None
        self.cam_info = None
        self.depth_frame = None

        self.create_subscription(CameraInfo, info_topic,  self.on_info,  qos_profile_sensor_data)
        self.create_subscription(Image,      img_topic,   self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image,      depth_topic, self.on_depth, qos_profile_sensor_data)

        self.pose_pub   = self.create_publisher(PoseStamped, '/perception/bottle_pose', 10)
        self.debug_pub  = self.create_publisher(Image, '/perception/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/perception/status', 10)
        self.ready_pub  = self.create_publisher(Bool, '/perception/handover_ready', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.yolo = None
        if self.mode == 'yolo' and _HAS_YOLO:
            try:
                self.yolo = YOLO(self.get_parameter('yolo_model').value)
                self.names = self.yolo.model.names if hasattr(self.yolo, 'model') else self.yolo.names
                self.get_logger().info(f"Loaded YOLO model: {self.get_parameter('yolo_model').value}")
            except Exception as e:
                self.get_logger().error(f"YOLO load failed: {e}")
        elif self.mode == 'yolo':
            self.get_logger().warn("ultralytics not installed. pip3 install --user ultralytics")

        self.get_logger().info(f"Perception node started. image:= {img_topic} | camera_info:= {info_topic} | mode={self.mode}")

    def on_info(self, msg: CameraInfo): self.cam_info = msg

    def on_depth(self, msg: Image):
        if not self.bridge: return
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            self.depth_frame = None

    def on_image(self, msg: Image):
        self.status_pub.publish(String(data="image_rx"))
        if not self.bridge: return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        detections = []
        holding = False

        if self.mode == 'yolo' and self.yolo is not None:
            import cv2
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
                        detections.append({'bb':bb, 'conf':cf, 'label':lbl, 'holding':is_hold})
            except Exception as e:
                self.get_logger().warn(f"YOLO inference failed: {e}")

        try:
            import cv2
            for d in detections:
                x0,y0,x1,y1 = d['bb'].astype(int)
                color = (0,255,0) if d['holding'] else (0,0,255)
                cv2.rectangle(frame,(x0,y0),(x1,y1),color,2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception:
            pass

        self.ready_pub.publish(Bool(data=bool(holding)))

        target = None
        if detections:
            preferred = [d for d in detections if d['holding']] if self.get_parameter('require_holding').value else detections
            if not preferred: preferred = detections
            target = max(preferred, key=lambda d: d['conf'])
        if target is None or self.cam_info is None or self.depth_frame is None:
            return

        x0,y0,x1,y1 = target['bb']
        H, W = self.depth_frame.shape[:2]
        x0,y0,x1,y1 = map(int,[max(0,x0),max(0,y0),min(W-1,x1),min(H-1,y1)])
        w,h = max(1,x1-x0), max(1,y1-y0)
        cx, cy = x0+w//2, y0+h//2
        rx, ry = max(0,cx-max(2,w//10)), max(0,cy-max(2,h//10))
        rx2,ry2= min(W,cx+max(3,w//10)), min(H,cy+max(3,h//10))
        roi = self.depth_frame[ry:ry2, rx:rx2]
        if roi.size == 0: return
        Zmm = float(np.nanmedian(roi))
        Z = Zmm*0.001 if Zmm>10.0 else Zmm
        if not (0.05 < Z < 4.0): return

        fx, fy, cx0, cy0 = self.cam_info.k[0], self.cam_info.k[4], self.cam_info.k[2], self.cam_info.k[5]
        X = (cx - cx0) * Z / fx
        Y = (cy - cy0) * Z / fy

        ps = PoseStamped()
        ps.header = self.cam_info.header if self.cam_info.header.stamp.sec != 0 else msg.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(X), float(Y), float(Z)
        qx,qy,qz,qw = quaternion_from_euler(0.0, 0.0, float(self.get_parameter('publish_orientation_yaw').value))
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qx,qy,qz,qw

        out = ps
        if _HAS_TF2_GEOM:
            try:
                tf = self.tf_buffer.lookup_transform('base_link', ps.header.frame_id, rclpy.time.Time())
                out = do_transform_pose(ps, tf); out.header.frame_id = 'base_link'
            except Exception as e:
                self.get_logger().warn(f"TF to base_link failed: {e}")

        self.pose_pub.publish(out)
        self.get_logger().info(f"{target['label']} pose @ {out.header.frame_id}: "
                               f"x={{:.2f}} y={{:.2f}} z={{:.2f}}".format(
                                   out.pose.position.x,out.pose.position.y,out.pose.position.z))

def main():
    rclpy.init(); n = BottleHumanDetector()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
