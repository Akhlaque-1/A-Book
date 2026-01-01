"""
Feedback System for User Commands

This module provides feedback to users on the status of their commands,
including execution progress, success/failure, and system state.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from robot_system.msg import UserCommand, ActionPlan, RobotState
from robot_system.srv import ProcessVoiceCommand
import time
from enum import Enum
from typing import Dict, Optional


class FeedbackStatus(Enum):
    """Enumeration for feedback statuses"""
    RECEIVED = "RECEIVED"
    PROCESSING = "PROCESSING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class FeedbackService(Node):
    """
    Feedback system that provides users with information about the status
    of their commands, including execution progress and results.
    """
    
    def __init__(self):
        super().__init__('feedback_service')
        
        # Publishers
        self.user_feedback_pub = self.create_publisher(String, '/user_feedback', 10)
        self.voice_feedback_pub = self.create_publisher(String, '/voice_feedback', 10)
        self.visual_feedback_pub = self.create_publisher(String, '/visual_feedback', 10)
        self.feedback_status_pub = self.create_publisher(String, '/feedback_status', 10)
        
        # Subscribers
        self.user_command_sub = self.create_subscription(
            UserCommand, '/user_command', self.user_command_received_callback, 10)
        self.processed_command_sub = self.create_subscription(
            UserCommand, '/processed_user_command', self.processed_command_callback, 10)
        self.action_plan_sub = self.create_subscription(
            ActionPlan, '/action_plan', self.action_plan_callback, 10)
        self.robot_state_sub = self.create_subscription(
            RobotState, '/robot_state', self.robot_state_callback, 10)
        
        # Internal state
        self.command_status = {}  # Maps command_id to status
        self.command_feedback = {}  # Maps command_id to feedback messages
        self.active_commands = set()  # Set of currently executing command IDs
        self.command_start_times = {}  # Maps command_id to start time
        self.feedback_history = []  # History of all feedback messages
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(0.5, self.publish_feedback_status)
        
        self.get_logger().info('Feedback Service initialized')
    
    def user_command_received_callback(self, msg):
        """Handle when a user command is first received"""
        command_id = msg.command_id
        self.get_logger().info(f'Command received: {msg.command_text} (ID: {command_id})')
        
        # Update command status
        self.command_status[command_id] = FeedbackStatus.RECEIVED
        self.active_commands.add(command_id)
        self.command_start_times[command_id] = time.time()
        
        # Generate feedback message
        feedback_msg = f"I received your command: '{msg.command_text}'. Processing now."
        self._publish_feedback(feedback_msg, command_id, FeedbackStatus.RECEIVED)
    
    def processed_command_callback(self, msg):
        """Handle when a command has been processed by NLP"""
        command_id = msg.command_id
        self.get_logger().info(f'Command processed: {msg.command_text} (ID: {command_id})')
        
        # Update command status
        self.command_status[command_id] = FeedbackStatus.PROCESSING
        
        # Generate feedback message based on intent
        intent = msg.intent
        if intent == 'NAVIGATE':
            feedback_msg = f"I understand you want me to navigate. Planning route now."
        elif intent == 'PERFORM_ACTION':
            feedback_msg = f"I understand you want me to perform an action. Preparing now."
        elif intent == 'REPORT_STATUS':
            feedback_msg = f"I'm checking my status and will report back."
        elif intent == 'PERCEPTION_TASK':
            feedback_msg = f"I'm analyzing the environment as requested."
        else:
            feedback_msg = f"I'm processing your request: '{msg.command_text}'."
        
        self._publish_feedback(feedback_msg, command_id, FeedbackStatus.PROCESSING)
    
    def action_plan_callback(self, msg):
        """Handle when an action plan is created"""
        # Find associated command (this is a simplification - in a real system, 
        # there would be a clearer connection between commands and plans)
        command_id = self._find_command_for_plan(msg)
        
        if command_id:
            self.get_logger().info(f'Action plan created for command {command_id}')
            
            # Update command status
            self.command_status[command_id] = FeedbackStatus.IN_PROGRESS
            
            # Generate feedback message
            feedback_msg = f"Starting to execute your command. This may take a moment."
            self._publish_feedback(feedback_msg, command_id, FeedbackStatus.IN_PROGRESS)
    
    def robot_state_callback(self, msg):
        """Handle updates to robot state that might affect command feedback"""
        # Check if any active commands should be updated based on robot state
        for command_id in self.active_commands.copy():
            # Check if the command is still being executed based on robot state
            if msg.operational_status == 'IDLE' and self.command_status[command_id] == FeedbackStatus.IN_PROGRESS:
                # Command may have completed
                self._mark_command_completed(command_id)
            elif msg.operational_status == 'ERROR':
                # Robot error may have affected command
                self._mark_command_failed(command_id, "Robot encountered an error")
    
    def _find_command_for_plan(self, action_plan) -> Optional[str]:
        """Find the command associated with an action plan (simplified implementation)"""
        # In a real system, there would be a clearer connection between commands and plans
        # For now, return the most recent active command
        if self.active_commands:
            return list(self.active_commands)[0]
        return None
    
    def _mark_command_completed(self, command_id: str):
        """Mark a command as completed"""
        self.command_status[command_id] = FeedbackStatus.COMPLETED
        self.active_commands.discard(command_id)
        
        # Calculate execution time
        start_time = self.command_start_times.get(command_id, time.time())
        execution_time = time.time() - start_time
        
        feedback_msg = f"Command completed successfully in {execution_time:.2f} seconds."
        self._publish_feedback(feedback_msg, command_id, FeedbackStatus.COMPLETED)
    
    def _mark_command_failed(self, command_id: str, reason: str = ""):
        """Mark a command as failed"""
        self.command_status[command_id] = FeedbackStatus.FAILED
        self.active_commands.discard(command_id)
        
        feedback_msg = f"Command failed to execute. Reason: {reason}" if reason else "Command failed to execute."
        self._publish_feedback(feedback_msg, command_id, FeedbackStatus.FAILED)
    
    def _publish_feedback(self, message: str, command_id: str, status: FeedbackStatus):
        """Publish feedback through all available channels"""
        # Add to command feedback history
        if command_id not in self.command_feedback:
            self.command_feedback[command_id] = []
        self.command_feedback[command_id].append({
            'timestamp': time.time(),
            'message': message,
            'status': status.value
        })
        
        # Add to overall feedback history
        self.feedback_history.append({
            'command_id': command_id,
            'timestamp': time.time(),
            'message': message,
            'status': status.value
        })
        
        # Publish to different feedback channels
        feedback_msg = String()
        
        # Voice feedback
        feedback_msg.data = message
        self.voice_feedback_pub.publish(feedback_msg)
        
        # Visual feedback (could be displayed on a screen or LED indicators)
        feedback_msg.data = f"[{status.value}] {message}"
        self.visual_feedback_pub.publish(feedback_msg)
        
        # General user feedback
        feedback_msg.data = f"Command {command_id}: {message}"
        self.user_feedback_pub.publish(feedback_msg)
        
        self.get_logger().info(f'Feedback published: {message}')
    
    def publish_feedback_status(self):
        """Publish the current feedback service status"""
        status_msg = String()
        status_msg.data = f'Active commands: {len(self.active_commands)}, Total feedback messages: {len(self.feedback_history)}'
        self.feedback_status_pub.publish(status_msg)
    
    def get_command_status(self, command_id: str) -> Optional[FeedbackStatus]:
        """Get the current status of a command"""
        return self.command_status.get(command_id)
    
    def get_command_feedback_history(self, command_id: str) -> list:
        """Get the feedback history for a specific command"""
        return self.command_feedback.get(command_id, [])
    
    def get_all_feedback_history(self) -> list:
        """Get all feedback history"""
        return self.feedback_history.copy()


def main(args=None):
    rclpy.init(args=args)
    
    feedback_service = FeedbackService()
    
    try:
        rclpy.spin(feedback_service)
    except KeyboardInterrupt:
        pass
    finally:
        feedback_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()