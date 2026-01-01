"""
Integration with Navigation Module

This module provides integration between the VLA module and the navigation system,
enabling the robot to plan and execute navigation tasks based on voice commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from robot_system.msg import UserCommand, ActionPlan, RobotState
from robot_system.srv import PlanNavigation
from robot_system.action import MoveHumanoid
from rclpy.action import ActionClient
import time
import math
from typing import Dict, List, Optional


class NavigationIntegration(Node):
    """
    Integration between the VLA module and the navigation system,
    enabling the robot to plan and execute navigation tasks based on voice commands.
    """
    
    def __init__(self):
        super().__init__('navigation_integration')
        
        # Publishers
        self.nav_status_pub = self.create_publisher(String, '/navigation_integration_status', 10)
        
        # Subscribers
        self.user_command_sub = self.create_subscription(
            UserCommand, '/processed_user_command', self.processed_command_callback, 10)
        self.action_plan_sub = self.create_subscription(
            ActionPlan, '/action_plan', self.action_plan_callback, 10)
        self.robot_state_sub = self.create_subscription(
            RobotState, '/robot_state', self.robot_state_callback, 10)
        
        # Services
        self.plan_navigation_client = self.create_client(
            PlanNavigation, 'plan_navigation')
        
        # Action clients
        self.move_humanoid_client = ActionClient(
            self, MoveHumanoid, 'move_humanoid')
        
        # Internal state
        self.robot_pose = Pose()
        self.known_locations = {
            'kitchen': self._create_pose(3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'living room': self._create_pose(-1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'bedroom': self._create_pose(2.0, -2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'office': self._create_pose(-2.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'entrance': self._create_pose(0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        }
        
        # Track active navigation tasks
        self.active_navigations = {}
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Wait for services and actions to be available
        self.get_logger().info('Waiting for navigation services and actions...')
        self.plan_navigation_client.wait_for_service()
        self.get_logger().info('Navigation services are available')
        
        self.get_logger().info('Navigation Integration initialized')
    
    def _create_pose(self, x, y, z, ox, oy, oz, ow):
        """Helper function to create a Pose object"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = ox
        pose.orientation.y = oy
        pose.orientation.z = oz
        pose.orientation.w = ow
        return pose
    
    def robot_state_callback(self, msg):
        """Update robot pose from robot state"""
        self.robot_pose = msg.pose
        self.get_logger().debug('Updated robot pose from robot state')
    
    def processed_command_callback(self, msg):
        """Handle processed user commands that might require navigation"""
        if msg.intent == 'NAVIGATE':
            self.get_logger().info(f'Processing navigation command: {msg.command_text}')
            self.handle_navigation_command(msg)
    
    def action_plan_callback(self, msg):
        """Handle action plans that contain navigation actions"""
        # Check if this action plan contains navigation actions
        for action in msg.action_sequence:
            if action.get('action_type') == 'NAVIGATE_TO_POSE':
                self.get_logger().info(f'Executing navigation action plan: {msg.plan_id}')
                self.execute_navigation_action_plan(msg)
                break
    
    def handle_navigation_command(self, user_command: UserCommand):
        """Handle a navigation command from the user"""
        try:
            # Extract destination from command
            destination = self._extract_destination(user_command.command_text)
            
            if destination.lower() in self.known_locations:
                target_pose = self.known_locations[destination.lower()]
                
                # Plan navigation using the navigation service
                self.plan_and_execute_navigation(user_command.command_id, self.robot_pose, target_pose)
            else:
                self.get_logger().warn(f'Unknown destination: {destination}')
                # Try to extract coordinates if available
                coords = self._extract_coordinates(user_command.command_text)
                if coords:
                    target_pose = self._create_pose(coords[0], coords[1], 0.0, 0.0, 0.0, 0.0, 1.0)
                    self.plan_and_execute_navigation(user_command.command_id, self.robot_pose, target_pose)
                else:
                    self.get_logger().error(f'Could not determine destination for navigation command: {user_command.command_text}')
                    # Publish feedback about unknown destination
                    self.publish_navigation_feedback(user_command.command_id, f"Sorry, I don't know where '{destination}' is located.")
        
        except Exception as e:
            self.get_logger().error(f'Error handling navigation command: {e}')
            self.publish_navigation_feedback(user_command.command_id, f"Error processing navigation command: {str(e)}")
    
    def plan_and_execute_navigation(self, command_id: str, start_pose: Pose, goal_pose: Pose):
        """Plan and execute navigation from start to goal"""
        try:
            # Create navigation plan request
            request = PlanNavigation.Request()
            request.start_pose = start_pose
            request.goal_pose = goal_pose
            
            # Send navigation plan request
            future = self.plan_navigation_client.call_async(request)
            future.add_done_callback(lambda future: self._navigation_plan_callback(future, command_id, goal_pose))
            
            self.get_logger().info(f'Navigation plan requested for command {command_id}')
            
        except Exception as e:
            self.get_logger().error(f'Error requesting navigation plan: {e}')
            self.publish_navigation_feedback(command_id, f"Error planning navigation: {str(e)}")
    
    def _navigation_plan_callback(self, future, command_id: str, goal_pose: Pose):
        """Callback for navigation plan response"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'Navigation plan successful for command {command_id}')
                
                # Execute the navigation using the action client
                self.execute_navigation_action(command_id, goal_pose)
            else:
                self.get_logger().error(f'Navigation plan failed for command {command_id}: {response.error_message}')
                self.publish_navigation_feedback(command_id, f"Could not plan navigation: {response.error_message}")
        
        except Exception as e:
            self.get_logger().error(f'Error in navigation plan callback: {e}')
            self.publish_navigation_feedback(command_id, f"Error in navigation planning: {str(e)}")
    
    def execute_navigation_action(self, command_id: str, goal_pose: Pose):
        """Execute navigation using the MoveHumanoid action"""
        try:
            # Wait for action server
            self.move_humanoid_client.wait_for_server()
            
            # Create goal message
            goal_msg = MoveHumanoid.Goal()
            goal_msg.target_pose = goal_pose
            goal_msg.movement_type = 'walk'  # Default to walking
            
            # Send goal
            goal_future = self.move_humanoid_client.send_goal_async(goal_msg)
            goal_future.add_done_callback(lambda future: self._navigation_goal_callback(future, command_id))
            
            self.get_logger().info(f'Navigation goal sent for command {command_id}')
            
            # Track this navigation
            self.active_navigations[command_id] = {
                'goal_pose': goal_pose,
                'start_time': time.time(),
                'status': 'EXECUTING'
            }
            
        except Exception as e:
            self.get_logger().error(f'Error executing navigation action: {e}')
            self.publish_navigation_feedback(command_id, f"Error executing navigation: {str(e)}")
    
    def _navigation_goal_callback(self, future, command_id: str):
        """Callback for navigation goal result"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info(f'Navigation goal was rejected for command {command_id}')
                self.publish_navigation_feedback(command_id, "Navigation goal was rejected by the system")
                return
            
            self.get_logger().info(f'Navigation goal accepted for command {command_id}')
            
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda future: self._navigation_result_callback(future, command_id))
            
        except Exception as e:
            self.get_logger().error(f'Error in navigation goal callback: {e}')
            self.publish_navigation_feedback(command_id, f"Error in navigation execution: {str(e)}")
    
    def _navigation_result_callback(self, future, command_id: str):
        """Callback for navigation result"""
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info(f'Navigation completed successfully for command {command_id}')
                self.publish_navigation_feedback(command_id, f"Successfully reached destination!")
                
                # Update navigation status
                if command_id in self.active_navigations:
                    self.active_navigations[command_id]['status'] = 'COMPLETED'
            else:
                self.get_logger().error(f'Navigation failed for command {command_id}: {result.error_message}')
                self.publish_navigation_feedback(command_id, f"Navigation failed: {result.error_message}")
                
                # Update navigation status
                if command_id in self.active_navigations:
                    self.active_navigations[command_id]['status'] = 'FAILED'
        
        except Exception as e:
            self.get_logger().error(f'Error in navigation result callback: {e}')
            self.publish_navigation_feedback(command_id, f"Error getting navigation result: {str(e)}")
        
        # Remove from active navigations
        if command_id in self.active_navigations:
            del self.active_navigations[command_id]
    
    def execute_navigation_action_plan(self, action_plan: ActionPlan):
        """Execute a navigation action plan"""
        for action in action_plan.action_sequence:
            if action.get('action_type') == 'NAVIGATE_TO_POSE':
                # Extract target pose from action parameters
                params = action.get('parameters', [])
                if len(params) >= 7:  # x, y, z, ox, oy, oz, ow
                    target_pose = self._create_pose(*params)
                    self.execute_navigation_action(action_plan.plan_id, target_pose)
                else:
                    self.get_logger().error(f'Invalid parameters for navigation action in plan {action_plan.plan_id}')
    
    def _extract_destination(self, command_text: str) -> str:
        """Extract destination from navigation command"""
        command_lower = command_text.lower()
        
        for location in self.known_locations.keys():
            if location in command_lower:
                return location
        
        # If no known location found, return the last word as a potential location
        words = command_text.split()
        if words:
            return words[-1]
        
        return ""
    
    def _extract_coordinates(self, command_text: str) -> Optional[List[float]]:
        """Extract coordinates from command if available"""
        import re
        
        # Pattern for "X meters forward/backward/left/right"
        pattern = r'(\d+(?:\.\d+)?)\s*meters?\s*(forward|backward|left|right)'
        matches = re.findall(pattern, command_text.lower())
        
        if matches:
            value, direction = matches[0]
            value = float(value)
            
            # Calculate relative coordinates based on current pose
            current_x = self.robot_pose.position.x
            current_y = self.robot_pose.position.y
            
            if direction == 'forward':
                return [current_x + value, current_y]
            elif direction == 'backward':
                return [current_x - value, current_y]
            elif direction == 'right':
                return [current_x, current_y - value]
            elif direction == 'left':
                return [current_x, current_y + value]
        
        return None
    
    def publish_navigation_feedback(self, command_id: str, message: str):
        """Publish feedback about navigation status"""
        feedback_msg = String()
        feedback_msg.data = f"Navigation ({command_id}): {message}"
        
        # Publish to the user feedback topic
        user_feedback_pub = self.create_publisher(String, '/user_feedback', 10)
        user_feedback_pub.publish(feedback_msg)
        
        self.get_logger().info(f'Navigation feedback: {message}')
    
    def publish_status(self):
        """Publish the current navigation integration status"""
        status_msg = String()
        status_msg.data = f'Active navigations: {len(self.active_navigations)}, Known locations: {len(self.known_locations)}'
        self.nav_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    nav_integration = NavigationIntegration()
    
    try:
        rclpy.spin(nav_integration)
    except KeyboardInterrupt:
        pass
    finally:
        nav_integration.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()