"""
Action Planning Service

This module implements the action planning service for the VLA module,
converting high-level commands into executable action sequences for the robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from robot_system.msg import UserCommand, ActionPlan
from robot_system.srv import PlanNavigation
import time
import math
from typing import List, Dict, Any, Optional


class ActionPlanner(Node):
    """
    Action planning service that converts high-level commands into
    executable action sequences for the robot.
    """
    
    def __init__(self):
        super().__init__('action_planner')
        
        # Publishers
        self.action_plan_pub = self.create_publisher(ActionPlan, '/action_plan', 10)
        self.planner_status_pub = self.create_publisher(String, '/action_planner_status', 10)
        
        # Subscribers
        self.user_command_sub = self.create_subscription(
            UserCommand, '/processed_user_command', self.processed_command_callback, 10)
        
        # Services
        self.plan_navigation_srv = self.create_service(
            PlanNavigation, 'plan_navigation', self.plan_navigation_callback)
        
        # Internal state
        self.action_plan_history = []
        self.robot_pose = Pose()
        self.robot_pose.position.x = 0.0
        self.robot_pose.position.y = 0.0
        self.robot_pose.position.z = 0.0
        self.robot_pose.orientation.x = 0.0
        self.robot_pose.orientation.y = 0.0
        self.robot_pose.orientation.z = 0.0
        self.robot_pose.orientation.w = 1.0
        
        # Known locations in the environment
        self.known_locations = {
            'kitchen': self._create_pose(3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'living room': self._create_pose(-1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'bedroom': self._create_pose(2.0, -2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'office': self._create_pose(-2.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'entrance': self._create_pose(0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        }
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Action Planner initialized')
    
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
    
    def processed_command_callback(self, msg):
        """Process incoming processed user commands"""
        self.get_logger().info(f'Planning action for command: {msg.command_text} (intent: {msg.intent})')
        
        try:
            # Generate an action plan based on the command intent
            action_plan = self.generate_action_plan(msg)
            
            if action_plan:
                # Publish the action plan
                self.action_plan_pub.publish(action_plan)
                
                # Add to history
                self.action_plan_history.append({
                    'id': action_plan.plan_id,
                    'command': msg.command_text,
                    'intent': msg.intent,
                    'timestamp': time.time()
                })
                
                self.get_logger().info(f'Action plan generated with ID: {action_plan.plan_id}')
            else:
                self.get_logger().error(f'Could not generate action plan for command: {msg.command_text}')
        
        except Exception as e:
            self.get_logger().error(f'Error generating action plan: {e}')
    
    def generate_action_plan(self, user_command: UserCommand) -> Optional[ActionPlan]:
        """Generate an action plan based on the user command"""
        action_plan = ActionPlan()
        action_plan.plan_id = f'plan_{int(time.time() * 1000)}_{user_command.command_id}'
        action_plan.goal_description = user_command.command_text
        action_plan.status = 'PENDING'
        
        # Set timestamp
        current_time = self.get_clock().now()
        action_plan.timestamp.sec = current_time.seconds_nanoseconds()[0]
        action_plan.timestamp.nanosec = current_time.seconds_nanoseconds()[1]
        
        # Generate action sequence based on intent
        if user_command.intent == 'NAVIGATE':
            return self._generate_navigation_plan(user_command, action_plan)
        elif user_command.intent == 'PERFORM_ACTION':
            return self._generate_action_plan(user_command, action_plan)
        elif user_command.intent == 'STOP':
            return self._generate_stop_plan(user_command, action_plan)
        elif user_command.intent == 'REPORT_STATUS':
            return self._generate_status_report_plan(user_command, action_plan)
        elif user_command.intent == 'PERCEPTION_TASK':
            return self._generate_perception_plan(user_command, action_plan)
        else:
            # For unknown intents, create a simple acknowledgment plan
            return self._generate_acknowledgment_plan(user_command, action_plan)
    
    def _generate_navigation_plan(self, user_command: UserCommand, base_plan: ActionPlan) -> ActionPlan:
        """Generate a navigation action plan"""
        # Extract destination from command
        destination = self._extract_destination(user_command.command_text)
        
        if destination.lower() in self.known_locations:
            target_pose = self.known_locations[destination.lower()]
            
            # Create navigation action
            nav_action = self._create_navigation_action(target_pose)
            base_plan.action_sequence = [nav_action]
            
            # Estimate duration based on distance
            distance = self._calculate_distance(self.robot_pose, target_pose)
            base_plan.estimated_duration = distance * 2.0  # 2 seconds per meter as estimate
            
            return base_plan
        else:
            self.get_logger().warn(f'Unknown destination: {destination}')
            # Try to extract coordinates if available
            coords = self._extract_coordinates(user_command.command_text)
            if coords:
                target_pose = self._create_pose(coords[0], coords[1], 0.0, 0.0, 0.0, 0.0, 1.0)
                nav_action = self._create_navigation_action(target_pose)
                base_plan.action_sequence = [nav_action]
                
                distance = self._calculate_distance(self.robot_pose, target_pose)
                base_plan.estimated_duration = distance * 2.0
                
                return base_plan
            else:
                self.get_logger().error(f'Could not determine destination for navigation command: {user_command.command_text}')
                return None
    
    def _generate_action_plan(self, user_command: UserCommand, base_plan: ActionPlan) -> ActionPlan:
        """Generate an action (manipulation) plan"""
        # For now, create a simple action plan
        # In a real system, this would involve complex manipulation planning
        
        # Extract object from command
        obj = self._extract_object(user_command.command_text)
        
        if obj:
            # Create a sequence of actions: navigate to object, grasp it
            actions = []
            
            # For demonstration, we'll just create a simple action
            action = self._create_simple_action('GRASP', [obj])
            actions.append(action)
            
            base_plan.action_sequence = actions
            base_plan.estimated_duration = 5.0  # 5 seconds for simple action
            
            return base_plan
        else:
            self.get_logger().warn(f'Could not identify object for action: {user_command.command_text}')
            # Create a simple acknowledgment action
            action = self._create_simple_action('ACKNOWLEDGE', [user_command.command_text])
            base_plan.action_sequence = [action]
            base_plan.estimated_duration = 1.0
            
            return base_plan
    
    def _generate_stop_plan(self, user_command: UserCommand, base_plan: ActionPlan) -> ActionPlan:
        """Generate a stop action plan"""
        # Create a stop action
        stop_action = self._create_simple_action('STOP', [])
        base_plan.action_sequence = [stop_action]
        base_plan.estimated_duration = 0.5  # Very quick
        
        return base_plan
    
    def _generate_status_report_plan(self, user_command: UserCommand, base_plan: ActionPlan) -> ActionPlan:
        """Generate a status report action plan"""
        # Create a status report action
        status_action = self._create_simple_action('REPORT_STATUS', [])
        base_plan.action_sequence = [status_action]
        base_plan.estimated_duration = 2.0  # 2 seconds to gather and report status
        
        return base_plan
    
    def _generate_perception_plan(self, user_command: UserCommand, base_plan: ActionPlan) -> ActionPlan:
        """Generate a perception action plan"""
        # Extract what to perceive
        target = self._extract_object(user_command.command_text) or self._extract_location(user_command.command_text)
        
        if target:
            # Create a perception action
            perception_action = self._create_simple_action('PERCEIVE', [target])
            base_plan.action_sequence = [perception_action]
            base_plan.estimated_duration = 3.0  # 3 seconds for perception
            
            return base_plan
        else:
            # Create a general perception action
            perception_action = self._create_simple_action('PERCEIVE', ['environment'])
            base_plan.action_sequence = [perception_action]
            base_plan.estimated_duration = 3.0
            
            return base_plan
    
    def _generate_acknowledgment_plan(self, user_command: UserCommand, base_plan: ActionPlan) -> ActionPlan:
        """Generate an acknowledgment plan for unknown commands"""
        # Create an acknowledgment action
        ack_action = self._create_simple_action('ACKNOWLEDGE', [user_command.command_text])
        base_plan.action_sequence = [ack_action]
        base_plan.estimated_duration = 1.0  # 1 second to acknowledge
        
        return base_plan
    
    def _create_navigation_action(self, target_pose: Pose) -> Any:
        """Create a navigation action"""
        # This would create a navigation action in a real system
        # For now, we'll return a simple dictionary representation
        return {
            'action_type': 'NAVIGATE_TO_POSE',
            'parameters': [
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w
            ],
            'target_pose': target_pose,
            'execution_priority': 1
        }
    
    def _create_simple_action(self, action_type: str, parameters: List) -> Any:
        """Create a simple action with the given type and parameters"""
        return {
            'action_type': action_type,
            'parameters': parameters,
            'execution_priority': 1
        }
    
    def _extract_destination(self, command_text: str) -> str:
        """Extract destination from navigation command"""
        # Simple extraction - look for location names
        command_lower = command_text.lower()
        
        for location in self.known_locations.keys():
            if location in command_lower:
                return location
        
        # If no known location found, return the last word as a potential location
        words = command_text.split()
        if words:
            return words[-1]
        
        return ""
    
    def _extract_object(self, command_text: str) -> str:
        """Extract object from command"""
        # Simple extraction - look for common object names
        command_lower = command_text.lower()
        
        common_objects = [
            'ball', 'cup', 'book', 'phone', 'keys', 'bottle', 
            'toy', 'box', 'chair', 'table', 'door', 'window'
        ]
        
        for obj in common_objects:
            if obj in command_lower:
                return obj
        
        # If no common object found, return relevant words
        words = command_text.split()
        # Look for words that might be objects (not commands)
        for word in words:
            if word not in ['pick', 'up', 'grasp', 'grab', 'take', 'lift', 'put', 'down', 'place', 'set']:
                return word
        
        return ""
    
    def _extract_location(self, command_text: str) -> str:
        """Extract location from command"""
        command_lower = command_text.lower()
        
        for location in self.known_locations.keys():
            if location in command_lower:
                return location
        
        return ""
    
    def _extract_coordinates(self, command_text: str) -> Optional[List[float]]:
        """Extract coordinates from command if available"""
        # Look for coordinate patterns like "x meters forward" or "y meters right"
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
    
    def _calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        """Calculate Euclidean distance between two poses"""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def plan_navigation_callback(self, request, response):
        """Service callback for planning navigation directly"""
        self.get_logger().info(f'Planning navigation from {request.start_pose} to {request.goal_pose}')
        
        try:
            # Create an action plan for navigation
            action_plan = ActionPlan()
            action_plan.plan_id = f'nav_plan_{int(time.time() * 1000)}'
            action_plan.goal_description = f'Navigate from ({request.start_pose.position.x}, {request.start_pose.position.y}) to ({request.goal_pose.position.x}, {request.goal_pose.position.y})'
            action_plan.status = 'PENDING'
            
            # Set timestamp
            current_time = self.get_clock().now()
            action_plan.timestamp.sec = current_time.seconds_nanoseconds()[0]
            action_plan.timestamp.nanosec = current_time.seconds_nanoseconds()[1]
            
            # Create navigation action
            nav_action = self._create_navigation_action(request.goal_pose)
            action_plan.action_sequence = [nav_action]
            
            # Calculate estimated duration
            distance = self._calculate_distance(request.start_pose, request.goal_pose)
            action_plan.estimated_duration = distance * 2.0  # 2 seconds per meter as estimate
            
            # Publish the action plan
            self.action_plan_pub.publish(action_plan)
            
            # Add to history
            self.action_plan_history.append({
                'id': action_plan.plan_id,
                'command': action_plan.goal_description,
                'intent': 'NAVIGATE',
                'timestamp': time.time()
            })
            
            # Prepare response
            response.success = True
            response.path.poses = [request.start_pose, request.goal_pose]  # Simplified path
            response.estimated_time = action_plan.estimated_duration
            response.error_message = ''
            
            self.get_logger().info(f'Navigation plan created with ID: {action_plan.plan_id}')
            
        except Exception as e:
            response.success = False
            response.path.poses = []
            response.estimated_time = 0.0
            response.error_message = f'Error planning navigation: {str(e)}'
            self.get_logger().error(f'Error in navigation planning: {e}')
        
        return response
    
    def publish_status(self):
        """Publish the current action planner status"""
        status_msg = String()
        status_msg.data = f'Action plans generated: {len(self.action_plan_history)}'
        self.planner_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    action_planner = ActionPlanner()
    
    try:
        rclpy.spin(action_planner)
    except KeyboardInterrupt:
        pass
    finally:
        action_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()