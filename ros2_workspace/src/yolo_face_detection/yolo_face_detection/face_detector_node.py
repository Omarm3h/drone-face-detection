#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector_node')
        
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/camera/image_raw')
        
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        image_topic = self.get_parameter('image_topic').value
        
        try:
            self.get_logger().info(f'Loading YOLO model: {model_path}')
            self.model = YOLO(model_path)
            self.model.to(device)
            self.get_logger().info(f'Model loaded on {device}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.detection_pub = self.create_publisher(Detection2DArray, '/face_detection/detections', 10)
        self.viz_pub = self.create_publisher(Image, '/face_detection/visualization', 10)
        self.count_pub = self.create_publisher(Int32, '/face_detection/count', 10)
        
        self.get_logger().info(f'Face Detector ready, listening on {image_topic}')
        self.frame_count = 0
        self.start_time = self.get_clock().now()
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
            
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            face_count = 0
            viz_image = cv_image.copy()
            
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    
                    if class_id == 0:  # person class
                        face_count += 1
                        
                        detection = Detection2D()
                        detection.header = msg.header
                        detection.bbox.center.position.x = float((x1 + x2) / 2)
                        detection.bbox.center.position.y = float((y1 + y2) / 2)
                        detection.bbox.size_x = float(x2 - x1)
                        detection.bbox.size_y = float(y2 - y1)
                        
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = 'person'
                        hypothesis.hypothesis.score = confidence
                        detection.results.append(hypothesis)
                        detections_msg.detections.append(detection)
                        
                        cv2.rectangle(viz_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(viz_image, f'Person: {confidence:.2f}', (int(x1), int(y1)-10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.putText(viz_image, f'Count: {face_count}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                fps = self.frame_count / elapsed
                self.get_logger().info(f'FPS: {fps:.1f}, Detected: {face_count}')
            
            self.detection_pub.publish(detections_msg)
            self.count_pub.publish(Int32(data=face_count))
            
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, 'bgr8')
            viz_msg.header = msg.header
            self.viz_pub.publish(viz_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = FaceDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
