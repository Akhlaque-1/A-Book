"""
Safety Monitoring System

This module provides safety monitoring for the humanoid robot system,
ensuring safe operation by preventing dangerous actions and monitoring
system states.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Pose
from robot_system.msg import RobotState, SensorDataStream, UserCommand
from robot_system.srv import ProcessVoiceCommand
import threading
import time
import math


class SafetyMonitor(Node):
    """
    Safety monitoring system for the humanoid robot.
    Monitors robot state, sensor data, and commands to ensure safe operation.
    """
    
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Parameters for safety thresholds
        self.declare_parameter('max_joint_velocity', 2.0)  # rad/s
        self.declare_parameter('max_joint_effort', 100.0)  # N*m
        self.declare_parameter('max_linear_velocity', 1.0)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('max_battery_low', 0.2)  # 20% battery
        self.declare_parameter('max_tilt_angle', 0.5)  # rad (about 28 degrees)
        self.declare_parameter('min_distance_obstacle', 0.5)  # meters
        
        # Get parameters
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.max_joint_effort = self.get_parameter('max_joint_effort').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.max_battery_low = self.get_parameter('max_battery_low').value
        self.max_tilt_angle = self.get_parameter('max_tilt_angle').value
        self.min_distance_obstacle = self.get_parameter('min_distance_obstacle').value
        
        # Publishers
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Subscribers
        self.robot_state_sub = self.create_subscription(
            RobotState, '/robot_state', self.robot_state_callback, 10)
        self.sensor_data_sub = self.create_subscription(
            SensorDataStream, '/sensor_data_stream', self.sensor_data_callback, 50)
        self.user_command_sub = self.create_subscription(
            UserCommand, '/user_command', self.user_command_callback, 10)
        
        # Service client for voice command processing (to intercept dangerous commands)
        self.voice_command_client = self.create_client(
            ProcessVoiceCommand, 'process_voice_command')
        
        # Internal state
        self.current_robot_state = RobotState()
        self.safety_status = 'SAFE'  # SAFE, WARNING, DANGER
        self.emergency_stop_active = False
        self.last_safety_check = time.time()
        
        # Timer for periodic safety checks
        self.safety_check_timer = self.create_timer(0.1, self.periodic_safety_check)  # 10 Hz
        
        self.get_logger().info('Safety Monitor initialized')
    
    def robot_state_callback(self, msg):
        """Handle incoming robot state messages"""
        self.current_robot_state = msg
        
        # Check for immediate safety violations
        self.check_robot_state_safety(msg)
    
    def sensor_data_callback(self, msg):
        """Handle incoming sensor data"""
        # Check sensor-specific safety conditions
        if msg.sensor_type == 'IMU':
            self.check_imu_safety(msg)
        elif msg.sensor_type == 'LIDAR':
            self.check_lidar_safety(msg)
        elif msg.sensor_type == 'CAMERA':
            self.check_camera_safety(msg)
    
    def user_command_callback(self, msg):
        """Handle incoming user commands"""
        # Check if the command is safe to execute
        is_safe = self.is_command_safe(msg)
        
        if not is_safe:
            self.get_logger().warn(f'Unsafe command detected: {msg.command_text}')
            self.trigger_safety_response('DANGEROUS_COMMAND')
    
    def check_robot_state_safety(self, robot_state):
        """Check the robot state for safety violations"""
        violations = []
        
        # Check battery level
        if robot_state.battery_level < self.max_battery_low:
            violations.append(f'Battery level too low: {robot_state.battery_level:.2f}')
        
        # Check joint states for safety
        for i, name in enumerate(robot_state.joint_states.name):
            if i < len(robot_state.joint_states.velocity):
                velocity = abs(robot_state.joint_states.velocity[i])
                if velocity > self.max_joint_velocity:
                    violations.append(f'Joint {name} velocity too high: {velocity:.2f}')
            
            if i < len(robot_state.joint_states.effort):
                effort = abs(robot_state.joint_states.effort[i])
                if effort > self.max_joint_effort:
                    violations.append(f'Joint {name} effort too high: {effort:.2f}')
        
        # Check operational status
        if robot_state.operational_status == 'ERROR':
            violations.append('Robot in error state')
        
        # Update safety status based on violations
        if violations:
            if self.safety_status != 'DANGER':
                self.safety_status = 'DANGER'
                self.get_logger().error(f'Safety violations detected: {violations}')
                self.publish_safety_status()
                self.trigger_safety_response('SAFETY_VIOLATIONS')
        elif self.safety_status == 'DANGER':
            # If we were in danger and now there are no violations, return to safe state
            self.safety_status = 'SAFE'
            self.get_logger().info('Safety violations resolved, returning to SAFE state')
            self.publish_safety_status()
    
    def check_imu_safety(self, sensor_data):
        """Check IMU data for safety violations (e.g., excessive tilt)"""
        # This would parse the IMU data from sensor_data and check for excessive tilt
        # For now, we'll simulate checking the orientation
        try:
            # Extract orientation from sensor data (this would be specific to the IMU message format)
            # For simulation purposes, we'll check a hypothetical tilt
            orientation = sensor_data.imu_data.orientation
            tilt_angle = math.sqrt(orientation.x**2 + orientation.y**2)
            
            if tilt_angle > self.max_tilt_angle:
                self.get_logger().warn(f'Excessive tilt detected: {tilt_angle:.2f} rad')
                self.trigger_safety_response('EXCESSIVE_TILT')
        except:
            # If we can't parse the IMU data, log a warning
            self.get_logger().warn('Could not parse IMU data for safety check')
    
    def check_lidar_safety(self, sensor_data):
        """Check LIDAR data for safety violations (e.g., obstacles too close)"""
        # This would parse the LIDAR data from sensor_data and check for obstacles
        # For now, we'll simulate checking for nearby obstacles
        try:
            # Extract distance data from sensor_data (this would be specific to the LIDAR message format)
            # For simulation purposes, we'll assume there's a minimum distance check
            min_distance = self.min_distance_obstacle  # This would come from actual LIDAR processing
            
            if min_distance < self.min_distance_obstacle:
                self.get_logger().warn(f'Obstacle too close: {min_distance:.2f} m')
                self.trigger_safety_response('OBSTACLE_TOO_CLOSE')
        except:
            # If we can't parse the LIDAR data, log a warning
            self.get_logger().warn('Could not parse LIDAR data for safety check')
    
    def check_camera_safety(self, sensor_data):
        """Check camera data for safety violations"""
        # This would analyze camera data for safety issues
        # For example, detecting humans too close, unsafe environments, etc.
        pass  # Implementation would depend on computer vision algorithms
    
    def is_command_safe(self, command):
        """Check if a user command is safe to execute"""
        # Check for dangerous keywords or commands
        dangerous_keywords = [
            'self-destruct', 'harm', 'damage', 'destroy', 'break', 
            'overheat', 'max speed', 'full power', 'crash'
        ]
        
        command_text = command.command_text.lower()
        for keyword in dangerous_keywords:
            if keyword in command_text:
                self.get_logger().warn(f'Dangerous keyword detected in command: {keyword}')
                return False
        
        # Additional safety checks could go here
        # For example, checking if navigation command is to a dangerous location
        
        return True
    
    def periodic_safety_check(self):
        """Perform periodic safety checks"""
        current_time = time.time()
        
        # Check if robot state is being updated regularly
        if current_time - self.last_safety_check > 1.0:  # More than 1 second since last check
            if self.safety_status != 'DANGER':
                self.safety_status = 'DANGER'
                self.get_logger().error('Robot state not updating - possible communication failure')
                self.publish_safety_status()
                self.trigger_safety_response('COMMUNICATION_FAILURE')
        
        self.last_safety_check = current_time
        
        # Publish current safety status
        self.publish_safety_status()
    
    def trigger_safety_response(self, violation_type):
        """Trigger appropriate safety response based on violation type"""
        self.get_logger().info(f'Triggering safety response for: {violation_type}')
        
        # Update safety status
        self.safety_status = 'DANGER'
        
        # Publish emergency stop command
        emergency_stop_msg = Bool()
        emergency_stop_msg.data = True
        self.emergency_stop_pub.publish(emergency_stop_msg)
        
        # Log the violation
        self.get_logger().error(f'SAFETY VIOLATION: {violation_type}')
        
        # Additional safety responses could be implemented here
        # For example:
        # - Stop all robot motion
        # - Activate hazard lights
        # - Send alert to operators
        # - Log detailed information for analysis
    
    def publish_safety_status(self):
        """Publish the current safety status"""
        status_msg = String()
        status_msg.data = self.safety_status
        self.safety_status_pub.publish(status_msg)
    
    def reset_safety_system(self):
        """Reset the safety system after a safety event"""
        self.get_logger().info('Resetting safety system')
        self.safety_status = 'SAFE'
        self.emergency_stop_active = False
        
        # Publish safety reset
        emergency_stop_msg = Bool()
        emergency_stop_msg.data = False
        self.emergency_stop_pub.publish(emergency_stop_msg)
        
        self.publish_safety_status()
    
    def is_safe_to_operate(self):
        """Check if it's safe to operate the robot"""
        return self.safety_status == 'SAFE' and not self.emergency_stop_active


def main(args=None):
    rclpy.init(args=args)
    
    safety_monitor = SafetyMonitor()
    
    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()