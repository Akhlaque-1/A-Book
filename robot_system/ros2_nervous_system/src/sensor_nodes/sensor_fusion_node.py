"""
Integration with ROS 2 Nervous System for Sensor Data

This module provides integration between the perception system and the ROS 2 nervous system,
enabling proper handling of sensor data throughout the robot system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image, Imu, JointState, LaserScan
from geometry_msgs.msg import Twist, Pose
from robot_system.msg import SensorDataStream, RobotState, PerceptionResult
from robot_system.srv import DetectObjects
import threading
from typing import Dict, List, Optional
import time


class SensorFusionNode(Node):
    """
    Integration node that connects the perception system with the ROS 2 nervous system,
    handling sensor data flow and coordination between different system components.
    """
    
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Publishers for fused sensor data
        self.fused_sensor_pub = self.create_publisher(SensorDataStream, '/fused_sensor_data', 50)
        self.perception_result_pub = self.create_publisher(PerceptionResult, '/perception_result', 10)
        self.robot_state_pub = self.create_publisher(RobotState, '/robot_state', 10)
        self.sensor_integration_status_pub = self.create_publisher(String, '/sensor_integration_status', 10)
        
        # Subscribers for raw sensor data
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar_points', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.laser_scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_scan_callback, 10)
        
        # Subscribers for system state
        self.robot_state_sub = self.create_subscription(
            RobotState, '/robot_state', self.robot_state_internal_callback, 10)
        
        # Services
        self.detect_objects_client = self.create_client(
            DetectObjects, 'detect_objects')
        
        # Internal state
        self.sensor_data_buffer = {}  # Buffer for different sensor types
        self.robot_state = RobotState()
        self.last_sensor_update = {}
        self.synchronization_window = 0.1  # 100ms window for sensor synchronization
        
        # Sensor availability tracking
        self.available_sensors = {
            'lidar': False,
            'camera': False,
            'imu': False,
            'joint_state': False,
            'laser_scan': False
        }
        
        # Lock for thread safety
        self.data_lock = threading.Lock()
        
        # Timer for periodic sensor data processing
        self.processing_timer = self.create_timer(0.05, self.process_sensor_data)  # 20 Hz
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        self.detect_objects_client.wait_for_service()
        self.get_logger().info('Required services are available')
        
        self.get_logger().info('Sensor Fusion Node initialized')
    
    def lidar_callback(self, msg):
        """Handle incoming LiDAR data"""
        with self.data_lock:
            self.sensor_data_buffer['lidar'] = {
                'data': msg,
                'timestamp': time.time(),
                'type': 'LIDAR'
            }
            self.last_sensor_update['lidar'] = time.time()
            self.available_sensors['lidar'] = True
        self.get_logger().debug('Received LiDAR data')
    
    def camera_callback(self, msg):
        """Handle incoming camera data"""
        with self.data_lock:
            self.sensor_data_buffer['camera'] = {
                'data': msg,
                'timestamp': time.time(),
                'type': 'CAMERA'
            }
            self.last_sensor_update['camera'] = time.time()
            self.available_sensors['camera'] = True
        self.get_logger().debug('Received camera data')
    
    def imu_callback(self, msg):
        """Handle incoming IMU data"""
        with self.data_lock:
            self.sensor_data_buffer['imu'] = {
                'data': msg,
                'timestamp': time.time(),
                'type': 'IMU'
            }
            self.last_sensor_update['imu'] = time.time()
            self.available_sensors['imu'] = True
        self.get_logger().debug('Received IMU data')
    
    def joint_state_callback(self, msg):
        """Handle incoming joint state data"""
        with self.data_lock:
            self.sensor_data_buffer['joint_state'] = {
                'data': msg,
                'timestamp': time.time(),
                'type': 'JOINT_STATE'
            }
            self.last_sensor_update['joint_state'] = time.time()
            self.available_sensors['joint_state'] = True
        self.get_logger().debug('Received joint state data')
    
    def laser_scan_callback(self, msg):
        """Handle incoming laser scan data"""
        with self.data_lock:
            self.sensor_data_buffer['laser_scan'] = {
                'data': msg,
                'timestamp': time.time(),
                'type': 'LASER_SCAN'
            }
            self.last_sensor_update['laser_scan'] = time.time()
            self.available_sensors['laser_scan'] = True
        self.get_logger().debug('Received laser scan data')
    
    def robot_state_internal_callback(self, msg):
        """Update internal robot state"""
        with self.data_lock:
            self.robot_state = msg
        self.get_logger().debug('Updated internal robot state')
    
    def process_sensor_data(self):
        """Process and fuse sensor data"""
        with self.data_lock:
            # Check if we have synchronized data from multiple sensors
            synchronized_data = self._get_synchronized_sensor_data()
            
            if synchronized_data:
                # Perform sensor fusion
                fused_data = self._fuse_sensor_data(synchronized_data)
                
                if fused_data:
                    # Publish fused data
                    self._publish_fused_data(fused_data)
                    
                    # Trigger perception processing if needed
                    self._trigger_perception_processing(fused_data)
    
    def _get_synchronized_sensor_data(self) -> Dict:
        """Get sensor data that is synchronized within the time window"""
        current_time = time.time()
        synchronized = {}
        
        for sensor_type, data_info in self.sensor_data_buffer.items():
            if current_time - data_info['timestamp'] <= self.synchronization_window:
                synchronized[sensor_type] = data_info
        
        return synchronized if len(synchronized) > 1 else {}  # Need at least 2 sensors for fusion
    
    def _fuse_sensor_data(self, synchronized_data: Dict) -> Optional[Dict]:
        """Fuse synchronized sensor data into a coherent representation"""
        if not synchronized_data:
            return None
        
        fused_result = {
            'timestamp': time.time(),
            'data_sources': list(synchronized_data.keys()),
            'fused_data': {},
            'confidence': 0.0
        }
        
        # Process each sensor type
        for sensor_type, data_info in synchronized_data.items():
            # In a real system, this would perform actual sensor fusion
            # For now, we'll just pass through the data
            fused_result['fused_data'][sensor_type] = data_info['data']
        
        # Calculate confidence based on number of sensors
        num_sensors = len(synchronized_data)
        fused_result['confidence'] = min(1.0, num_sensors * 0.3)  # Each sensor contributes 0.3 to confidence
        
        return fused_result
    
    def _publish_fused_data(self, fused_data: Dict):
        """Publish the fused sensor data"""
        # Create a SensorDataStream message with fused data
        fused_msg = SensorDataStream()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = 'robot_base'
        fused_msg.sensor_type = 'FUSED_SENSOR_DATA'
        fused_msg.sensor_id = 'multi_sensor_fusion'
        fused_msg.quality = fused_data['confidence']
        
        # Publish the fused data
        self.fused_sensor_pub.publish(fused_msg)
    
    def _trigger_perception_processing(self, fused_data: Dict):
        """Trigger perception processing with fused data"""
        # In a real system, this would call perception algorithms
        # For now, we'll just log that perception processing would occur
        self.get_logger().info(f'Perception processing triggered with {len(fused_data["data_sources"])} sensor sources')
        
        # Create a perception result based on the fused data
        perception_result = PerceptionResult()
        perception_result.header.stamp = self.get_clock().now().to_msg()
        perception_result.header.frame_id = 'robot_base'
        
        # For now, we'll just publish an empty perception result
        # In a real system, this would contain actual perception results
        self.perception_result_pub.publish(perception_result)
    
    def get_available_sensors(self) -> Dict[str, bool]:
        """Get the status of available sensors"""
        with self.data_lock:
            return self.available_sensors.copy()
    
    def get_last_sensor_update_times(self) -> Dict[str, float]:
        """Get the last update time for each sensor"""
        with self.data_lock:
            return self.last_sensor_update.copy()
    
    def publish_status(self):
        """Publish the current sensor integration status"""
        available_sensors = [sensor for sensor, available in self.available_sensors.items() if available]
        status_msg = String()
        status_msg.data = f'Available sensors: {len(available_sensors)}/{len(self.available_sensors)}, Last updates: {self.last_sensor_update}'
        self.sensor_integration_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    sensor_fusion_node = SensorFusionNode()
    
    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()