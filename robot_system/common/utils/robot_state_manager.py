"""
Robot State Manager

This module provides functionality for managing the state of the humanoid robot,
including position, orientation, joint angles, and operational status.
"""

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from robot_system.msg import RobotState  # This would be generated from robot_state.msg
import time


class RobotStateManager:
    """
    Manages the current state of the humanoid robot including position, 
    orientation, joint angles, and operational status.
    """
    
    def __init__(self):
        self.header = Header()
        self.position = Point()
        self.orientation = Quaternion()
        self.joint_angles = {}
        self.sensor_readings = {}
        self.battery_level = 1.0  # Start with full battery
        self.operational_status = "IDLE"
        self.safety_status = "SAFE"
        self.timestamp = rospy.Time.now()
        
        # Initialize joint angles with default values
        self._initialize_joint_angles()
        
        # Setup ROS publisher for robot state
        self.state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)
        
    def _initialize_joint_angles(self):
        """Initialize joint angles with default positions"""
        # Default joint positions for humanoid robot
        self.joint_angles = {
            'head_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0
        }
    
    def update_joint_angle(self, joint_name, angle):
        """Update a specific joint angle"""
        if joint_name in self.joint_angles:
            self.joint_angles[joint_name] = angle
        else:
            rospy.logwarn(f"Joint {joint_name} not found in robot model")
    
    def update_sensor_reading(self, sensor_type, value):
        """Update a sensor reading"""
        self.sensor_readings[sensor_type] = value
    
    def update_battery_level(self, level):
        """Update battery level (0.0 to 1.0)"""
        if 0.0 <= level <= 1.0:
            self.battery_level = level
        else:
            rospy.logwarn("Battery level must be between 0.0 and 1.0")
    
    def update_operational_status(self, status):
        """Update operational status"""
        valid_statuses = ["IDLE", "MOVING", "PROCESSING", "ERROR"]
        if status in valid_statuses:
            self.operational_status = status
        else:
            rospy.logwarn(f"Invalid operational status: {status}")
    
    def update_safety_status(self, status):
        """Update safety status"""
        valid_statuses = ["SAFE", "WARNING", "DANGER"]
        if status in valid_statuses:
            self.safety_status = status
        else:
            rospy.logwarn(f"Invalid safety status: {status}")
    
    def get_robot_state_msg(self):
        """Create and return a RobotState message with current state"""
        state_msg = RobotState()
        
        # Update header
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = "robot_base"
        
        # Update pose
        state_msg.pose.position = self.position
        state_msg.pose.orientation = self.orientation
        
        # Update joint states
        joint_state = JointState()
        joint_state.header = state_msg.header
        joint_state.name = list(self.joint_angles.keys())
        joint_state.position = list(self.joint_angles.values())
        # For simplicity, we're not setting velocities and efforts
        state_msg.joint_states = joint_state
        
        # Update battery level
        state_msg.battery_level = self.battery_level
        
        # Update operational and safety status
        state_msg.operational_status = self.operational_status
        state_msg.safety_status = self.safety_status
        
        # Update timestamp
        state_msg.timestamp.sec = int(time.time())
        state_msg.timestamp.nanosec = int((time.time() % 1) * 1e9)
        
        return state_msg
    
    def publish_state(self):
        """Publish the current robot state to the /robot_state topic"""
        state_msg = self.get_robot_state_msg()
        self.state_pub.publish(state_msg)
    
    def get_joint_angles(self):
        """Get all joint angles"""
        return self.joint_angles.copy()
    
    def get_position(self):
        """Get current position"""
        return self.position
    
    def get_orientation(self):
        """Get current orientation"""
        return self.orientation
    
    def get_battery_level(self):
        """Get current battery level"""
        return self.battery_level
    
    def get_operational_status(self):
        """Get current operational status"""
        return self.operational_status
    
    def get_safety_status(self):
        """Get current safety status"""
        return self.safety_status


# Example usage
if __name__ == '__main__':
    rospy.init_node('robot_state_manager')
    state_manager = RobotStateManager()
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        state_manager.publish_state()
        rate.sleep()