"""
Sensor Fusion Service

This module implements sensor fusion for the AI-Robot Brain,
combining data from multiple sensors (LiDAR, cameras, IMU) to create
a coherent understanding of the environment.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image, Imu, JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from robot_system.msg import SensorDataStream, PerceptionResult
from robot_system.srv import DetectObjects
import numpy as np
import math
from typing import Dict, List, Optional, Tuple
from collections import deque
import threading


class SensorFusionService(Node):
    """
    Sensor fusion service that combines data from multiple sensors
    to create a coherent understanding of the environment.
    """
    
    def __init__(self):
        super().__init__('sensor_fusion_service')
        
        # Publishers
        self.fused_data_pub = self.create_publisher(SensorDataStream, '/fused_sensor_data', 50)
        self.perception_result_pub = self.create_publisher(PerceptionResult, '/perception_result', 10)
        self.sensor_fusion_status_pub = self.create_publisher(String, '/sensor_fusion_status', 10)
        
        # Subscribers for different sensor types
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar_points', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Additional sensor data stream subscriber
        self.sensor_stream_sub = self.create_subscription(
            SensorDataStream, '/sensor_data_stream', self.sensor_stream_callback, 50)
        
        # Services
        self.detect_objects_srv = self.create_service(
            DetectObjects, 'detect_objects', self.detect_objects_callback)
        
        # Internal state
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None
        self.joint_state_data = None
        
        # Time-synchronized buffers for sensor fusion
        self.sensor_buffers = {
            'lidar': deque(maxlen=10),
            'camera': deque(maxlen=10),
            'imu': deque(maxlen=10),
            'joint_state': deque(maxlen=10)
        }
        
        # Robot pose estimation
        self.robot_pose = Pose()
        self.robot_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Environment model
        self.environment_map = {}  # Would contain detected objects and their properties
        
        # Fusion parameters
        self.fusion_window = 0.1  # 100ms window for time synchronization
        self.confidence_threshold = 0.7  # Minimum confidence for fused data
        
        # Lock for thread safety
        self.data_lock = threading.Lock()
        
        # Timer for periodic fusion processing
        self.fusion_timer = self.create_timer(0.05, self.perform_sensor_fusion)  # 20 Hz
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Sensor Fusion Service initialized')
    
    def lidar_callback(self, msg):
        """Handle incoming LiDAR data"""
        with self.data_lock:
            self.lidar_data = msg
            self.sensor_buffers['lidar'].append({
                'timestamp': msg.header.stamp,
                'data': msg,
                'sensor_type': 'LIDAR'
            })
        self.get_logger().debug('Received LiDAR data')
    
    def camera_callback(self, msg):
        """Handle incoming camera data"""
        with self.data_lock:
            self.camera_data = msg
            self.sensor_buffers['camera'].append({
                'timestamp': msg.header.stamp,
                'data': msg,
                'sensor_type': 'CAMERA'
            })
        self.get_logger().debug('Received camera data')
    
    def imu_callback(self, msg):
        """Handle incoming IMU data"""
        with self.data_lock:
            self.imu_data = msg
            self.sensor_buffers['imu'].append({
                'timestamp': msg.header.stamp,
                'data': msg,
                'sensor_type': 'IMU'
            })
            
            # Update robot orientation from IMU
            self.robot_orientation = msg.orientation
        self.get_logger().debug('Received IMU data')
    
    def joint_state_callback(self, msg):
        """Handle incoming joint state data"""
        with self.data_lock:
            self.joint_state_data = msg
            self.sensor_buffers['joint_state'].append({
                'timestamp': msg.header.stamp,
                'data': msg,
                'sensor_type': 'JOINT_STATE'
            })
        self.get_logger().debug('Received joint state data')
    
    def sensor_stream_callback(self, msg):
        """Handle incoming sensor data stream messages"""
        with self.data_lock:
            # Add to appropriate buffer based on sensor type
            if msg.sensor_type == 'LIDAR':
                self.sensor_buffers['lidar'].append({
                    'timestamp': msg.header.stamp,
                    'data': msg,
                    'sensor_type': 'LIDAR'
                })
            elif msg.sensor_type == 'CAMERA':
                self.sensor_buffers['camera'].append({
                    'timestamp': msg.header.stamp,
                    'data': msg,
                    'sensor_type': 'CAMERA'
                })
            elif msg.sensor_type == 'IMU':
                self.sensor_buffers['imu'].append({
                    'timestamp': msg.header.stamp,
                    'data': msg,
                    'sensor_type': 'IMU'
                })
            elif msg.sensor_type == 'JOINT_ENCODER':
                self.sensor_buffers['joint_state'].append({
                    'timestamp': msg.header.stamp,
                    'data': msg,
                    'sensor_type': 'JOINT_ENCODER'
                })
        self.get_logger().debug(f'Received sensor stream data: {msg.sensor_type}')
    
    def perform_sensor_fusion(self):
        """Perform sensor fusion on synchronized data"""
        with self.data_lock:
            # Find the most recent common timestamp across all sensors
            common_timestamp = self._find_common_timestamp()
            
            if common_timestamp is None:
                return  # Not enough synchronized data yet
            
            # Get synchronized sensor data
            synchronized_data = self._get_synchronized_data(common_timestamp)
            
            if not synchronized_data:
                return  # Could not synchronize data
            
            # Perform fusion
            fused_result = self._fuse_sensor_data(synchronized_data)
            
            if fused_result:
                # Publish the fused result
                self._publish_fused_result(fused_result, common_timestamp)
                
                # Update environment model
                self._update_environment_model(fused_result)
    
    def _find_common_timestamp(self) -> Optional[float]:
        """Find a common timestamp across all sensor buffers"""
        if not all(self.sensor_buffers.values()):
            return None
        
        # Get the latest timestamp from each buffer
        latest_timestamps = []
        for sensor_type, buffer in self.sensor_buffers.items():
            if buffer:
                latest_ts = buffer[-1]['timestamp']
                latest_timestamps.append(latest_ts.sec + latest_ts.nanosec / 1e9)
        
        if not latest_timestamps:
            return None
        
        # Find the earliest of the latest timestamps (most conservative approach)
        common_time = min(latest_timestamps)
        
        # Check if all sensors have data within the fusion window
        for sensor_type, buffer in self.sensor_buffers.items():
            if not buffer:
                return None
            
            # Find the closest timestamp in the buffer
            closest_ts = None
            min_diff = float('inf')
            for item in buffer:
                ts = item['timestamp'].sec + item['timestamp'].nanosec / 1e9
                diff = abs(ts - common_time)
                if diff < min_diff:
                    min_diff = diff
                    closest_ts = ts
            
            if min_diff > self.fusion_window:
                return None  # Not within fusion window
        
        return common_time
    
    def _get_synchronized_data(self, target_time: float) -> Dict:
        """Get sensor data synchronized to the target time"""
        synchronized = {}
        
        for sensor_type, buffer in self.sensor_buffers.items():
            if not buffer:
                continue
            
            # Find the closest data to the target time
            closest_item = None
            min_diff = float('inf')
            
            for item in buffer:
                ts = item['timestamp'].sec + item['timestamp'].nanosec / 1e9
                diff = abs(ts - target_time)
                
                if diff < min_diff:
                    min_diff = diff
                    closest_item = item
            
            if closest_item and min_diff <= self.fusion_window:
                synchronized[sensor_type] = closest_item
        
        return synchronized
    
    def _fuse_sensor_data(self, synchronized_data: Dict) -> Optional[Dict]:
        """Fuse synchronized sensor data into a coherent representation"""
        if not synchronized_data:
            return None
        
        fused_result = {
            'timestamp': None,
            'position_estimate': None,
            'orientation_estimate': None,
            'detected_objects': [],
            'environment_map': {},
            'confidence': 0.0
        }
        
        # Process each sensor type
        for sensor_type, data_item in synchronized_data.items():
            if fused_result['timestamp'] is None:
                fused_result['timestamp'] = data_item['timestamp']
            
            if sensor_type == 'IMU':
                # Use IMU for orientation
                imu_data = data_item['data']
                fused_result['orientation_estimate'] = imu_data.orientation
            elif sensor_type == 'LIDAR':
                # Process LiDAR data for object detection and mapping
                lidar_data = data_item['data']
                objects = self._process_lidar_data(lidar_data)
                fused_result['detected_objects'].extend(objects)
            elif sensor_type == 'CAMERA':
                # Process camera data for object detection
                camera_data = data_item['data']
                camera_objects = self._process_camera_data(camera_data)
                fused_result['detected_objects'].extend(camera_objects)
            elif sensor_type in ['JOINT_STATE', 'JOINT_ENCODER']:
                # Use joint data for position estimation
                joint_data = data_item['data']
                pos_estimate = self._estimate_position_from_joints(joint_data)
                fused_result['position_estimate'] = pos_estimate
        
        # Calculate overall confidence based on number of sensors contributing
        num_sensors = len(synchronized_data)
        fused_result['confidence'] = min(1.0, num_sensors * 0.3)  # Each sensor contributes 0.3 to confidence
        
        # Perform cross-validation between sensors
        validated_objects = self._validate_objects_with_multiple_sensors(
            fused_result['detected_objects'], synchronized_data
        )
        fused_result['detected_objects'] = validated_objects
        
        return fused_result
    
    def _process_lidar_data(self, lidar_msg) -> List[Dict]:
        """Process LiDAR point cloud data to detect objects"""
        # This is a simplified implementation
        # In a real system, this would use point cloud processing algorithms
        objects = []
        
        # For demonstration, we'll create some dummy objects
        # In a real system, this would perform actual point cloud segmentation
        if lidar_msg.height * lidar_msg.width > 0:
            # Create a dummy object representing a detected obstacle
            object_info = {
                'class_name': 'obstacle',
                'confidence': 0.9,
                'position': {'x': 1.0, 'y': 0.5, 'z': 0.0},
                'size': {'length': 0.5, 'width': 0.3, 'height': 0.8}
            }
            objects.append(object_info)
        
        return objects
    
    def _process_camera_data(self, camera_msg) -> List[Dict]:
        """Process camera image data to detect objects"""
        # This is a simplified implementation
        # In a real system, this would use computer vision algorithms
        objects = []
        
        # For demonstration, we'll create some dummy objects
        # In a real system, this would perform actual image processing
        if camera_msg.height > 0 and camera_msg.width > 0:
            # Create a dummy object representing a detected person
            object_info = {
                'class_name': 'person',
                'confidence': 0.85,
                'position': {'x': 2.0, 'y': 0.0, 'z': 0.0},
                'size': {'length': 0.6, 'width': 0.3, 'height': 1.7}
            }
            objects.append(object_info)
        
        return objects
    
    def _estimate_position_from_joints(self, joint_msg) -> Optional[Point]:
        """Estimate robot position based on joint angles"""
        # This is a simplified implementation
        # In a real system, this would use forward kinematics
        # For now, return the current position estimate
        return Point(x=self.robot_pose.position.x, y=self.robot_pose.position.y, z=self.robot_pose.position.z)
    
    def _validate_objects_with_multiple_sensors(self, objects: List[Dict], sensor_data: Dict) -> List[Dict]:
        """Validate detected objects using multiple sensors"""
        validated_objects = []
        
        for obj in objects:
            # Increase confidence if detected by multiple sensors
            # For this simplified version, we'll just return the objects as-is
            validated_objects.append(obj)
        
        return validated_objects
    
    def _publish_fused_result(self, fused_result: Dict, timestamp):
        """Publish the fused sensor result"""
        # Create a SensorDataStream message with fused data
        fused_msg = SensorDataStream()
        fused_msg.header.stamp = timestamp
        fused_msg.header.frame_id = 'robot_base'
        fused_msg.sensor_type = 'FUSED_SENSOR_DATA'
        fused_msg.sensor_id = 'multi_sensor_fusion'
        fused_msg.quality = fused_result['confidence']
        
        # Publish the fused data
        self.fused_data_pub.publish(fused_msg)
        
        # Create and publish a PerceptionResult if there are detected objects
        if fused_result['detected_objects']:
            perception_msg = PerceptionResult()
            perception_msg.header.stamp = timestamp
            perception_msg.header.frame_id = 'robot_base'
            
            # Convert detected objects to the appropriate format
            for obj in fused_result['detected_objects']:
                # This would require creating custom message types in a real system
                pass
            
            self.perception_result_pub.publish(perception_msg)
    
    def _update_environment_model(self, fused_result: Dict):
        """Update the internal environment model with fused data"""
        # Update the environment map with new information
        for obj in fused_result['detected_objects']:
            # Add or update object in environment map
            obj_key = f"{obj['class_name']}_{len(self.environment_map)}"
            self.environment_map[obj_key] = obj
    
    def detect_objects_callback(self, request, response):
        """Service callback for object detection"""
        self.get_logger().info('Processing object detection request')
        
        try:
            # Process the sensor data for object detection
            # For this implementation, we'll use the latest fused data
            with self.data_lock:
                # In a real system, we would process the specific sensor data provided in the request
                # For now, we'll return the latest detected objects from our fusion process
                detected_objects = self._get_latest_detected_objects()
            
            if detected_objects:
                response.success = True
                response.objects = detected_objects  # This would need to be properly formatted
                response.error_message = ''
                
                self.get_logger().info(f'Detected {len(detected_objects)} objects')
            else:
                response.success = False
                response.objects = []
                response.error_message = 'No objects detected'
                
        except Exception as e:
            response.success = False
            response.objects = []
            response.error_message = f'Error in object detection: {str(e)}'
            self.get_logger().error(f'Error in object detection: {e}')
        
        return response
    
    def _get_latest_detected_objects(self):
        """Get the latest detected objects from the environment model"""
        # This is a simplified implementation
        # In a real system, this would return properly formatted objects
        return []
    
    def publish_status(self):
        """Publish the current sensor fusion service status"""
        status_msg = String()
        status_msg.data = f'Detected objects: {len(self.environment_map)}, Buffers: {[len(buf) for buf in self.sensor_buffers.values()]}'
        self.sensor_fusion_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    sensor_fusion_service = SensorFusionService()
    
    try:
        rclpy.spin(sensor_fusion_service)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()