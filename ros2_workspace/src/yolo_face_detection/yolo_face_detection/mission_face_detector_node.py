#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class MissionFaceDetectorNode(Node):
    def __init__(self):
        super().__init__('mission_face_detector_node')
        
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('min_detection_size', 30)
        
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        image_topic = self.get_parameter('image_topic').value
        self.min_detection_size = self.get_parameter('min_detection_size').value
        
        try:
            self.get_logger().info(f'Loading YOLO model: {model_path}')
            self.model = YOLO(model_path)
            self.model.to(device)
            self.get_logger().info(f'YOLO loaded on {device}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO: {e}')
            raise
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        
        self.detection_pub = self.create_publisher(Detection2DArray, '/face_detection/detections', 10)
        self.viz_pub = self.create_publisher(Image, '/face_detection/visualization', 10)
        self.count_pub = self.create_publisher(Int32, '/face_detection/count', 10)
        self.detected_pub = self.create_publisher(Bool, '/face_detection/detected', 10)
        self.position_pub = self.create_publisher(Point, '/face_detection/position', 10)
        
        self.get_logger().info(f'Mission Face Detector ready on {image_topic}')
        self.frame_count = 0
        self.start_time = self.get_clock().now()
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            img_height, img_width = cv_image.shape[:2]
            
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
            
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            detection_count = 0
            viz_image = cv_image.copy()
            
            best_detection = None
            best_area = 0
            
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    
                    if class_id == 0:
                        box_w = x2 - x1
                        box_h = y2 - y1
                        box_area = box_w * box_h
                        
                        if box_w < self.min_detection_size or box_h < self.min_detection_size:
                            continue
                        
                        detection_count += 1
                        
                        if box_area > best_area:
                            best_area = box_area
                            center_x = (x1 + x2) / 2.0
                            center_y = (y1 + y2) / 2.0
                            norm_x = (center_x / img_width) * 2.0 - 1.0
                            norm_y = (center_y / img_height) * 2.0 - 1.0
                            best_detection = (norm_x, norm_y, confidence)
                        
                        detection = Detection2D()
                        detection.header = msg.header
                        detection.bbox.center.position.x = float((x1 + x2) / 2)
                        detection.bbox.center.position.y = float((y1 + y2) / 2)
                        detection.bbox.size_x = float(box_w)
                        detection.bbox.size_y = float(box_h)
                        
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = 'person'
                        hypothesis.hypothesis.score = confidence
                        detection.results.append(hypothesis)
                        detections_msg.detections.append(detection)
                        
                        cv2.rectangle(viz_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(viz_image, f'Person: {confidence:.2f}', (int(x1), int(y1) - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.putText(viz_image, f'Detected: {detection_count}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                fps = self.frame_count / elapsed
                self.get_logger().info(f'FPS: {fps:.1f}, Detected: {detection_count}')
            
            self.detection_pub.publish(detections_msg)
            self.count_pub.publish(Int32(data=detection_count))
            
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, 'bgr8')
            viz_msg.header = msg.header
            self.viz_pub.publish(viz_msg)
            
            detected_msg = Bool()
            detected_msg.data = (detection_count > 0)
            self.detected_pub.publish(detected_msg)
            
            if best_detection is not None:
                pos_msg = Point()
                pos_msg.x = best_detection[0]
                pos_msg.y = best_detection[1]
                pos_msg.z = best_detection[2]
                self.position_pub.publish(pos_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionFaceDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
