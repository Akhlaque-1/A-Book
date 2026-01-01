"""
Object Detection Service

This module implements object detection for the AI-Robot Brain,
using computer vision techniques to identify and locate objects in the environment.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from robot_system.msg import PerceptionResult, SensorDataStream
from robot_system.srv import DetectObjects
import cv2
from cv2 import aruco
import numpy as np
from typing import List, Dict, Optional, Tuple
import threading


class ObjectDetectionService(Node):
    """
    Object detection service that identifies and locates objects in the environment
    using computer vision techniques.
    """
    
    def __init__(self):
        super().__init__('object_detection_service')
        
        # Publishers
        self.detection_result_pub = self.create_publisher(PerceptionResult, '/detection_result', 10)
        self.detection_status_pub = self.create_publisher(String, '/object_detection_status', 10)
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # Services
        self.detect_objects_srv = self.create_service(
            DetectObjects, 'detect_objects', self.detect_objects_callback)
        
        # Internal state
        self.latest_image = None
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.detection_results = []
        
        # Detection parameters
        self.detection_threshold = 0.7
        self.enable_aruco_detection = True
        self.enable_color_detection = True
        self.enable_template_matching = True
        
        # Known objects for template matching
        self.known_objects = {}  # Would contain templates for known objects
        
        # Lock for thread safety
        self.image_lock = threading.Lock()
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Object Detection Service initialized')
    
    def camera_info_callback(self, msg):
        """Handle camera calibration information"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)
        self.get_logger().debug('Updated camera calibration parameters')
    
    def camera_callback(self, msg):
        """Handle incoming camera images"""
        # Convert ROS Image message to OpenCV format
        try:
            # Determine image encoding and convert to OpenCV format
            if msg.encoding == 'rgb8' or msg.encoding == 'bgr8':
                # Convert the image data to a numpy array
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img_data = img_data.reshape((msg.height, msg.width, 3))
                
                # If it's RGB, convert to BGR for OpenCV
                if msg.encoding == 'rgb8':
                    img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}')
                return
            
            with self.image_lock:
                self.latest_image = img_data
            
            # Perform detection on the new image
            self.perform_detection(img_data, msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')
    
    def perform_detection(self, image, timestamp):
        """Perform object detection on the provided image"""
        if image is None:
            return
        
        detection_results = []
        
        # Perform different types of detection
        if self.enable_aruco_detection:
            aruco_results = self._detect_aruco_markers(image)
            detection_results.extend(aruco_results)
        
        if self.enable_color_detection:
            color_results = self._detect_color_objects(image)
            detection_results.extend(color_results)
        
        if self.enable_template_matching:
            template_results = self._detect_template_objects(image)
            detection_results.extend(template_results)
        
        # Publish results if any objects were detected
        if detection_results:
            self._publish_detection_results(detection_results, timestamp)
    
    def _detect_aruco_markers(self, image) -> List[Dict]:
        """Detect ArUco markers in the image"""
        results = []
        
        try:
            # Define ArUco dictionary
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()
            
            # Detect markers
            corners, ids, rejected_img_points = aruco.detectMarkers(
                image, aruco_dict, parameters=parameters,
                cameraMatrix=self.camera_matrix,
                distCoeff=self.distortion_coeffs
            )
            
            if ids is not None:
                # Estimate pose of each marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.camera_matrix, self.distortion_coeffs
                )
                
                for i in range(len(ids)):
                    # Calculate position relative to camera
                    position = {
                        'x': float(tvecs[i][0][0]),
                        'y': float(tvecs[i][0][1]),
                        'z': float(tvecs[i][0][2])
                    }
                    
                    result = {
                        'class_name': f'aruco_marker_{ids[i][0]}',
                        'confidence': 0.95,
                        'position': position,
                        'size': {'length': 0.05, 'width': 0.05, 'height': 0.001},
                        'bbox': self._corners_to_bbox(corners[i][0])
                    }
                    results.append(result)
        
        except Exception as e:
            self.get_logger().warn(f'Error in ArUco detection: {e}')
        
        return results
    
    def _detect_color_objects(self, image) -> List[Dict]:
        """Detect objects based on color ranges"""
        results = []
        
        try:
            # Convert to HSV for color detection
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Define color ranges (these are example ranges)
            color_ranges = {
                'red': (np.array([0, 50, 50]), np.array([10, 255, 255])),
                'blue': (np.array([100, 50, 50]), np.array([130, 255, 255])),
                'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),
                'yellow': (np.array([20, 50, 50]), np.array([30, 255, 255]))
            }
            
            for color_name, (lower, upper) in color_ranges.items():
                # Create mask for the color range
                mask = cv2.inRange(hsv, lower, upper)
                
                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    # Filter by area to avoid tiny detections
                    area = cv2.contourArea(contour)
                    if area > 500:  # Minimum area threshold
                        # Get bounding box
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        # Calculate center position in 3D space (simplified)
                        center_x = x + w / 2
                        center_y = y + h / 2
                        # In a real system, we would use depth information to calculate Z
                        position = {
                            'x': float(center_x),
                            'y': float(center_y),
                            'z': 1.0  # Placeholder depth
                        }
                        
                        result = {
                            'class_name': color_name,
                            'confidence': min(0.9, area / 10000),  # Confidence based on size
                            'position': position,
                            'size': {'length': float(w), 'width': float(h), 'height': 0.1},
                            'bbox': [int(x), int(y), int(w), int(h)]
                        }
                        results.append(result)
        
        except Exception as e:
            self.get_logger().warn(f'Error in color detection: {e}')
        
        return results
    
    def _detect_template_objects(self, image) -> List[Dict]:
        """Detect known objects using template matching"""
        results = []
        
        # This is a simplified implementation
        # In a real system, we would have templates for known objects
        return results
    
    def _corners_to_bbox(self, corners) -> List[int]:
        """Convert ArUco marker corners to bounding box [x, y, w, h]"""
        x_coords = [int(corner[0]) for corner in corners]
        y_coords = [int(corner[1]) for corner in corners]
        
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        return [x_min, y_min, x_max - x_min, y_max - y_min]
    
    def _publish_detection_results(self, detection_results, timestamp):
        """Publish the detection results"""
        # Create a PerceptionResult message
        result_msg = PerceptionResult()
        result_msg.header.stamp = timestamp
        result_msg.header.frame_id = 'camera_frame'
        
        # For now, we'll just log the results
        # In a real system, we would populate the PerceptionResult message
        # with properly formatted detected objects
        
        for result in detection_results:
            self.get_logger().info(f'Detected {result["class_name"]} with confidence {result["confidence"]:.2f}')
        
        # Store results internally
        self.detection_results = detection_results
        
        # Publish the results
        self.detection_result_pub.publish(result_msg)
    
    def detect_objects_callback(self, request, response):
        """Service callback for object detection"""
        self.get_logger().info('Processing object detection request via service')
        
        try:
            with self.image_lock:
                if self.latest_image is not None:
                    # Perform detection on the latest image
                    self.perform_detection(self.latest_image, self.get_clock().now().to_msg())
                    
                    # Prepare response based on internal results
                    if self.detection_results:
                        response.success = True
                        # In a real system, we would format the detection results properly
                        # For now, we'll just indicate that objects were detected
                        response.error_message = ''
                        
                        self.get_logger().info(f'Detected {len(self.detection_results)} objects')
                    else:
                        response.success = False
                        response.error_message = 'No objects detected in the current image'
                else:
                    response.success = False
                    response.error_message = 'No image data available for detection'
        
        except Exception as e:
            response.success = False
            response.error_message = f'Error in object detection: {str(e)}'
            self.get_logger().error(f'Error in object detection service: {e}')
        
        return response
    
    def publish_status(self):
        """Publish the current object detection service status"""
        status_msg = String()
        status_msg.data = f'Detected objects: {len(self.detection_results)}, Camera matrix: {self.camera_matrix is not None}'
        self.detection_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    object_detection_service = ObjectDetectionService()
    
    try:
        rclpy.spin(object_detection_service)
    except KeyboardInterrupt:
        pass
    finally:
        object_detection_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()