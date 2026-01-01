"""
VLA Interface for ROS 2 Nervous System

This module provides the interface between the Vision-Language-Action (VLA) module
and the ROS 2 nervous system, facilitating communication between high-level
cognitive planning and low-level robot control.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from robot_system.msg import UserCommand, ActionPlan, RobotState
from robot_system.srv import ProcessVoiceCommand, PlanNavigation
from robot_system.action import MoveHumanoid
from rclpy.action import ActionClient
import time
import threading
from typing import Dict, List, Optional


class VLAInterface(Node):
    """
    Interface between the VLA module and the ROS 2 nervous system.
    Handles communication between high-level cognitive planning and low-level robot control.
    """
    
    def __init__(self):
        super().__init__('vla_interface')
        
        # Publishers for sending commands to the nervous system
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.vla_status_pub = self.create_publisher(String, '/vla_interface_status', 10)
        
        # Subscribers for receiving state from the nervous system
        self.robot_state_sub = self.create_subscription(
            RobotState, '/robot_state', self.robot_state_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Services for communicating with other modules
        self.voice_command_client = self.create_client(
            ProcessVoiceCommand, 'process_voice_command')
        self.navigation_client = self.create_client(
            PlanNavigation, 'plan_navigation')
        
        # Action clients for complex behaviors
        self.move_humanoid_client = ActionClient(
            self, MoveHumanoid, 'move_humanoid')
        
        # Internal state
        self.current_robot_state = RobotState()
        self.current_joint_state = JointState()
        self.active_action_plans = {}
        self.command_queue = []
        self.is_operational = True
        
        # Timer for periodic status updates and command processing
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.command_timer = self.create_timer(0.1, self.process_command_queue)
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        self.voice_command_client.wait_for_service()
        self.navigation_client.wait_for_service()
        self.get_logger().info('Required services are available')
        
        self.get_logger().info('VLA Interface initialized and ready')
    
    def robot_state_callback(self, msg):
        """Update internal robot state from nervous system"""
        self.current_robot_state = msg
        self.get_logger().debug('Updated robot state from nervous system')
    
    def joint_state_callback(self, msg):
        """Update internal joint state from nervous system"""
        self.current_joint_state = msg
        self.get_logger().debug('Updated joint state from nervous system')
    
    def send_command_to_robot(self, action_plan: ActionPlan):
        """Send an action plan to the robot for execution"""
        self.get_logger().info(f'Sending action plan to robot: {action_plan.plan_id}')
        
        # Process each action in the sequence
        for i, action in enumerate(action_plan.action_sequence):
            action_type = action.get('action_type', 'UNKNOWN')
            
            if action_type == 'NAVIGATE_TO_POSE':
                self._execute_navigation_action(action)
            elif action_type == 'GRASP':
                self._execute_grasp_action(action)
            elif action_type == 'STOP':
                self._execute_stop_action()
            elif action_type == 'REPORT_STATUS':
                self._execute_status_action()
            elif action_type == 'PERCEIVE':
                self._execute_perceive_action(action)
            elif action_type == 'ACKNOWLEDGE':
                self._execute_acknowledge_action(action)
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
        
        # Update action plan status
        action_plan.status = 'EXECUTING'
        self.active_action_plans[action_plan.plan_id] = action_plan
    
    def _execute_navigation_action(self, action):
        """Execute a navigation action"""
        try:
            target_pose = action.get('target_pose')
            if target_pose:
                # Send navigation goal via action client
                goal_msg = MoveHumanoid.Goal()
                goal_msg.target_pose = target_pose
                goal_msg.movement_type = 'walk'
                
                # Wait for action server
                self.move_humanoid_client.wait_for_server()
                
                # Send goal
                future = self.move_humanoid_client.send_goal_async(goal_msg)
                future.add_done_callback(self._navigation_goal_callback)
                
                self.get_logger().info(f'Navigation goal sent to action server')
            else:
                self.get_logger().error('No target pose in navigation action')
        except Exception as e:
            self.get_logger().error(f'Error executing navigation action: {e}')
    
    def _navigation_goal_callback(self, future):
        """Callback for navigation goal result"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Navigation goal was rejected')
                return
            
            self.get_logger().info('Navigation goal accepted, waiting for result...')
            
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._navigation_result_callback)
        except Exception as e:
            self.get_logger().error(f'Error in navigation goal callback: {e}')
    
    def _navigation_result_callback(self, future):
        """Callback for navigation result"""
        try:
            result = future.result().result
            self.get_logger().info(f'Navigation result: {result.success}')
        except Exception as e:
            self.get_logger().error(f'Error in navigation result callback: {e}')
    
    def _execute_grasp_action(self, action):
        """Execute a grasp action"""
        try:
            # For now, just log the action
            # In a real system, this would send commands to the manipulator
            obj_to_grasp = action.get('parameters', ['unknown'])[0] if action.get('parameters') else 'unknown'
            self.get_logger().info(f'Attempting to grasp object: {obj_to_grasp}')
            
            # Send joint commands to perform grasping
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = ['left_gripper_joint', 'right_gripper_joint']  # Example joint names
            joint_cmd.position = [0.5, 0.5]  # Example positions
            
            self.joint_cmd_pub.publish(joint_cmd)
        except Exception as e:
            self.get_logger().error(f'Error executing grasp action: {e}')
    
    def _execute_stop_action(self):
        """Execute a stop action"""
        try:
            # Send zero velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info('Stop command sent to robot')
        except Exception as e:
            self.get_logger().error(f'Error executing stop action: {e}')
    
    def _execute_status_action(self):
        """Execute a status report action"""
        try:
            # For now, just log the action
            # In a real system, this might trigger a status report from various subsystems
            self.get_logger().info('Status report action executed')
            
            # Publish current robot state as status
            status_msg = String()
            status_msg.data = f'Robot status: {self.current_robot_state.operational_status}, Battery: {self.current_robot_state.battery_level:.2f}'
            self.vla_status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Error executing status action: {e}')
    
    def _execute_perceive_action(self, action):
        """Execute a perception action"""
        try:
            target = action.get('parameters', ['environment'])[0] if action.get('parameters') else 'environment'
            self.get_logger().info(f'Perception action: observing {target}')
            
            # In a real system, this would trigger perception modules
            # For now, just log the action
        except Exception as e:
            self.get_logger().error(f'Error executing perception action: {e}')
    
    def _execute_acknowledge_action(self, action):
        """Execute an acknowledgment action"""
        try:
            command_text = action.get('parameters', ['unknown'])[0] if action.get('parameters') else 'unknown'
            self.get_logger().info(f'Acknowledging command: {command_text}')
            
            # In a real system, this might trigger a verbal or visual acknowledgment
            # For now, just log the action
        except Exception as e:
            self.get_logger().error(f'Error executing acknowledge action: {e}')
    
    def process_command_queue(self):
        """Process commands in the queue"""
        if self.command_queue:
            # Process the next command in the queue
            action_plan = self.command_queue.pop(0)
            self.send_command_to_robot(action_plan)
    
    def add_command_to_queue(self, action_plan: ActionPlan):
        """Add an action plan to the execution queue"""
        self.command_queue.append(action_plan)
        self.get_logger().info(f'Added action plan {action_plan.plan_id} to execution queue')
    
    def publish_status(self):
        """Publish the current VLA interface status"""
        status_msg = String()
        status_msg.data = f'Operational: {self.is_operational}, Queued commands: {len(self.command_queue)}, Active plans: {len(self.active_action_plans)}'
        self.vla_status_pub.publish(status_msg)
    
    def is_robot_ready(self) -> bool:
        """Check if the robot is ready to receive commands"""
        return (self.is_operational and 
                self.current_robot_state.operational_status in ['IDLE', 'PROCESSING'] and
                self.current_robot_state.safety_status == 'SAFE' and
                self.current_robot_state.battery_level > 0.1)  # At least 10% battery
    
    def handle_action_plan(self, action_plan: ActionPlan):
        """Handle an incoming action plan"""
        if self.is_robot_ready():
            self.add_command_to_queue(action_plan)
        else:
            self.get_logger().warn(f'Robot not ready, adding plan {action_plan.plan_id} to queue')
            self.add_command_to_queue(action_plan)  # Still add to queue, but log the issue


def main(args=None):
    rclpy.init(args=args)
    
    vla_interface = VLAInterface()
    
    try:
        rclpy.spin(vla_interface)
    except KeyboardInterrupt:
        pass
    finally:
        vla_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()