"""
Obstacle Detection and Avoidance Service

This module implements obstacle detection and avoidance for the AI-Robot Brain,
enabling the robot to navigate safely around obstacles in its environment.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Path
from robot_system.msg import SensorDataStream, RobotState
from robot_system.srv import PlanNavigation
from robot_system.action import MoveHumanoid
from rclpy.action import ActionClient
import numpy as np
import math
from typing import List, Dict, Optional, Tuple
import threading


class ObstacleAvoidanceService(Node):
    """
    Obstacle detection and avoidance service that enables the robot
    to navigate safely around obstacles in its environment.
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance_service')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.avoidance_status_pub = self.create_publisher(String, '/obstacle_avoidance_status', 10)
        self.collision_warning_pub = self.create_publisher(Bool, '/collision_warning', 10)
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar_points', self.lidar_callback, 10)
        self.laser_scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_scan_callback, 10)
        self.robot_state_sub = self.create_subscription(
            RobotState, '/robot_state', self.robot_state_callback, 10)
        self.sensor_stream_sub = self.create_subscription(
            SensorDataStream, '/sensor_data_stream', self.sensor_stream_callback, 50)
        
        # Services
        self.plan_navigation_client = self.create_client(
            PlanNavigation, 'plan_navigation')
        
        # Action clients
        self.move_humanoid_client = ActionClient(
            self, MoveHumanoid, 'move_humanoid')
        
        # Internal state
        self.robot_pose = Pose()
        self.robot_velocity = Point(x=0.0, y=0.0, z=0.0)
        self.obstacles = []  # List of detected obstacles
        self.safety_margin = 0.5  # 50cm safety margin
        self.avoidance_active = False
        self.collision_imminent = False
        self.navigation_goal = None
        
        # Obstacle detection parameters
        self.min_obstacle_distance = 1.0  # Minimum distance to consider obstacle
        self.max_detection_range = 5.0  # Maximum range for obstacle detection
        self.avoidance_threshold = 0.8  # Distance threshold for avoidance action
        
        # Lock for thread safety
        self.state_lock = threading.Lock()
        
        # Timer for periodic obstacle detection and avoidance
        self.avoidance_timer = self.create_timer(0.1, self.check_obstacles_and_avoid)  # 10 Hz
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Obstacle Avoidance Service initialized')
    
    def robot_state_callback(self, msg):
        """Update robot state information"""
        with self.state_lock:
            self.robot_pose = msg.pose
            # Update velocity from joint states if available
            if len(msg.joint_states.velocity) >= 2:
                # Simplified velocity estimation from joint velocities
                pass
    
    def lidar_callback(self, msg):
        """Handle incoming LiDAR data for obstacle detection"""
        try:
            # Process LiDAR point cloud to detect obstacles
            obstacles = self._process_lidar_data(msg)
            
            with self.state_lock:
                self.obstacles = obstacles
        
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
    
    def laser_scan_callback(self, msg):
        """Handle incoming laser scan data for obstacle detection"""
        try:
            # Process laser scan to detect obstacles
            obstacles = self._process_laser_scan(msg)
            
            with self.state_lock:
                self.obstacles = obstacles  # Could merge with LiDAR obstacles in a real system
        
        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {e}')
    
    def sensor_stream_callback(self, msg):
        """Handle incoming sensor data stream"""
        if msg.sensor_type == 'LIDAR':
            # Process as LiDAR data
            pass
        elif msg.sensor_type == 'LASER':
            # Process as laser data
            pass
    
    def _process_lidar_data(self, msg) -> List[Dict]:
        """Process LiDAR point cloud data to detect obstacles"""
        obstacles = []
        
        # This is a simplified implementation
        # In a real system, we would properly parse the PointCloud2 message
        # For now, we'll simulate obstacle detection
        
        # Simulate some obstacles in front of the robot
        for i in range(5):
            obstacle = {
                'position': Point(
                    x=self.robot_pose.position.x + 1.0 + i * 0.5,
                    y=self.robot_pose.position.y + (i % 2) * 0.3,
                    z=0.0
                ),
                'size': 0.3,  # Approximate size
                'distance': 1.0 + i * 0.5  # Distance from robot
            }
            obstacles.append(obstacle)
        
        return obstacles
    
    def _process_laser_scan(self, msg) -> List[Dict]:
        """Process laser scan data to detect obstacles"""
        obstacles = []
        
        # Process laser scan ranges to detect obstacles
        for i, range_val in enumerate(msg.ranges):
            if not math.isnan(range_val) and range_val < self.max_detection_range:
                # Calculate angle of this measurement
                angle = msg.angle_min + i * msg.angle_increment
                
                # Calculate position relative to robot
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                
                obstacle = {
                    'position': Point(
                        x=self.robot_pose.position.x + x,
                        y=self.robot_pose.position.y + y,
                        z=0.0
                    ),
                    'size': 0.2,  # Approximate size
                    'distance': range_val
                }
                obstacles.append(obstacle)
        
        return obstacles
    
    def check_obstacles_and_avoid(self):
        """Check for obstacles and perform avoidance maneuvers"""
        with self.state_lock:
            if not self.obstacles:
                # No obstacles detected, continue normal operation
                self.avoidance_active = False
                self.collision_imminent = False
                return
            
            # Check if any obstacles are too close
            closest_obstacle = min(self.obstacles, key=lambda o: o['distance'])
            
            if closest_obstacle['distance'] < self.avoidance_threshold:
                self.collision_imminent = True
                self.avoidance_active = True
                
                # Determine avoidance action
                avoidance_cmd = self._compute_avoidance_maneuver(closest_obstacle)
                
                # Publish avoidance command
                self.cmd_vel_pub.publish(avoidance_cmd)
                
                # Publish collision warning
                warning_msg = Bool()
                warning_msg.data = True
                self.collision_warning_pub.publish(warning_msg)
                
                self.get_logger().warn(f'Obstacle detected at {closest_obstacle["distance"]:.2f}m, performing avoidance maneuver')
            else:
                self.collision_imminent = False
                self.avoidance_active = False
                
                # Publish collision warning reset
                warning_msg = Bool()
                warning_msg.data = False
                self.collision_warning_pub.publish(warning_msg)
    
    def _compute_avoidance_maneuver(self, obstacle) -> Twist:
        """Compute an avoidance maneuver based on obstacle position"""
        cmd_vel = Twist()
        
        # Calculate relative position of obstacle to robot
        rel_x = obstacle['position'].x - self.robot_pose.position.x
        rel_y = obstacle['position'].y - self.robot_pose.position.y
        
        # Calculate angle to obstacle
        angle_to_obstacle = math.atan2(rel_y, rel_x)
        
        # Calculate robot's current heading (simplified)
        robot_yaw = self._get_robot_yaw()
        
        # Calculate relative angle
        rel_angle = angle_to_obstacle - robot_yaw
        
        # Normalize angle to [-pi, pi]
        while rel_angle > math.pi:
            rel_angle -= 2 * math.pi
        while rel_angle < -math.pi:
            rel_angle += 2 * math.pi
        
        # If obstacle is in front, turn away
        if abs(rel_angle) < math.pi / 3:  # Within 60 degrees of front
            # Turn away from obstacle
            if rel_angle > 0:
                # Obstacle on the left, turn right
                cmd_vel.angular.z = -0.5  # Turn right
            else:
                # Obstacle on the right, turn left
                cmd_vel.angular.z = 0.5   # Turn left
            
            # Slow down or stop linear motion
            cmd_vel.linear.x = 0.1  # Move slowly forward
        else:
            # Obstacle not directly in front, continue with caution
            cmd_vel.linear.x = 0.2  # Move forward slowly
            cmd_vel.angular.z = 0.0  # No turning
        
        return cmd_vel
    
    def _get_robot_yaw(self) -> float:
        """Get the robot's current yaw angle from orientation"""
        # Extract yaw from quaternion (simplified)
        # In a real system, we would properly convert quaternion to Euler angles
        return 0.0  # Placeholder
    
    def set_navigation_goal(self, goal_pose: Pose):
        """Set a navigation goal and handle obstacle avoidance during navigation"""
        with self.state_lock:
            self.navigation_goal = goal_pose
    
    def get_closest_obstacle(self) -> Optional[Dict]:
        """Get the closest obstacle to the robot"""
        with self.state_lock:
            if not self.obstacles:
                return None
            return min(self.obstacles, key=lambda o: o['distance'])
    
    def is_path_clear(self, target_pose: Pose) -> bool:
        """Check if the path to the target pose is clear of obstacles"""
        with self.state_lock:
            # Calculate the path from current position to target
            dx = target_pose.position.x - self.robot_pose.position.x
            dy = target_pose.position.y - self.robot_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if any obstacles are on this path
            for obstacle in self.obstacles:
                # Calculate distance from path to obstacle
                # This is a simplified check
                obstacle_dx = obstacle['position'].x - self.robot_pose.position.x
                obstacle_dy = obstacle['position'].y - self.robot_pose.position.y
                obstacle_distance = math.sqrt(obstacle_dx*obstacle_dx + obstacle_dy*obstacle_dy)
                
                # Check if obstacle is within path corridor
                if obstacle_distance < distance and obstacle['distance'] < self.safety_margin:
                    return False  # Path is blocked
            
            return True  # Path appears clear
    
    def publish_status(self):
        """Publish the current obstacle avoidance service status"""
        status_msg = String()
        status_msg.data = f'Obstacles: {len(self.obstacles)}, Avoidance: {self.avoidance_active}, Collision Imminent: {self.collision_imminent}'
        self.avoidance_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    obstacle_avoidance_service = ObstacleAvoidanceService()
    
    try:
        rclpy.spin(obstacle_avoidance_service)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_avoidance_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()