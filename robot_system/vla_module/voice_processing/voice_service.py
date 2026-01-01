"""
Voice Processing Service

This module implements the voice processing service for the VLA module,
handling speech recognition and initial processing of voice commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from robot_system.msg import UserCommand
from robot_system.srv import ProcessVoiceCommand
import speech_recognition as sr
import threading
import time
import os


class VoiceService(Node):
    """
    Voice processing service that handles speech recognition and
    initial processing of voice commands for the VLA module.
    """
    
    def __init__(self):
        super().__init__('voice_service')
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Set up audio parameters
        self.recognizer.energy_threshold = 4000  # Adjust based on environment
        self.recognizer.dynamic_energy_threshold = True
        
        # Publishers
        self.user_command_pub = self.create_publisher(UserCommand, '/user_command', 10)
        self.voice_status_pub = self.create_publisher(String, '/voice_status', 10)
        
        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData, '/audio_input', self.audio_callback, 10)
        
        # Services
        self.process_voice_srv = self.create_service(
            ProcessVoiceCommand, 'process_voice_command', self.process_voice_command_callback)
        
        # Internal state
        self.is_listening = False
        self.activation_keyword = "Hey Robot"
        self.confidence_threshold = 0.7
        self.command_history = []
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Start listening thread
        self.listening_thread = threading.Thread(target=self.listen_continuously, daemon=True)
        self.listening_thread.start()
        
        self.get_logger().info('Voice Service initialized and listening for commands')
    
    def listen_continuously(self):
        """Continuously listen for voice commands"""
        self.is_listening = True
        
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)  # Adjust for ambient noise
            
        while self.is_listening:
            try:
                with self.microphone as source:
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)
                
                # Process the audio
                self.process_audio(audio)
                
            except sr.WaitTimeoutError:
                # This is expected when no audio is detected, continue listening
                continue
            except Exception as e:
                self.get_logger().error(f'Error in voice listening: {e}')
                time.sleep(0.1)  # Brief pause before continuing
    
    def audio_callback(self, msg):
        """Handle incoming audio data from the audio input topic"""
        # Convert the AudioData message to audio data that the recognizer can process
        # Note: This is a simplified implementation; actual implementation would need
        # to properly convert the ROS AudioData message to the format expected by
        # the speech recognition library
        pass
    
    def process_audio(self, audio):
        """Process audio data to extract voice commands"""
        try:
            # Try to recognize speech using Google Web Speech API
            # Note: For offline recognition, you could use pocketsphinx or Vosk
            text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Recognized: {text}')
            
            # Check if the command contains the activation keyword
            if self.activation_keyword.lower() in text.lower():
                # Extract the actual command after the activation keyword
                command_text = text.lower().split(self.activation_keyword.lower(), 1)[1].strip()
                
                if command_text:  # If there's actual command text
                    self.create_and_publish_user_command(command_text)
            else:
                # Check if this looks like a direct command (no activation keyword needed in some modes)
                if len(text.split()) > 2:  # If it's more than 2 words, treat as command
                    self.create_and_publish_user_command(text)
        
        except sr.UnknownValueError:
            self.get_logger().debug('Speech recognition could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Could not request results from speech recognition service; {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')
    
    def create_and_publish_user_command(self, command_text):
        """Create a UserCommand message and publish it"""
        user_command = UserCommand()
        user_command.command_id = f'cmd_{int(time.time() * 1000)}'  # Unique ID based on timestamp
        user_command.command_text = command_text
        user_command.processed_text = command_text  # For now, same as original
        user_command.intent = self.classify_intent(command_text)  # Classify intent
        user_command.confidence = 1.0  # For now, assume high confidence
        user_command.status = 'RECEIVED'
        
        # Set timestamp
        current_time = self.get_clock().now()
        user_command.timestamp.sec = current_time.seconds_nanoseconds()[0]
        user_command.timestamp.nanosec = current_time.seconds_nanoseconds()[1]
        
        # Publish the command
        self.user_command_pub.publish(user_command)
        
        self.get_logger().info(f'Published user command: {command_text}')
        
        # Add to command history
        self.command_history.append({
            'id': user_command.command_id,
            'text': command_text,
            'timestamp': time.time()
        })
    
    def classify_intent(self, command_text):
        """Classify the intent of a command"""
        command_lower = command_text.lower()
        
        # Simple keyword-based intent classification
        if any(word in command_lower for word in ['go to', 'navigate', 'move to', 'walk to', 'go']):
            return 'NAVIGATE'
        elif any(word in command_lower for word in ['pick up', 'grasp', 'grab', 'take', 'lift']):
            return 'PERFORM_ACTION'
        elif any(word in command_lower for word in ['stop', 'halt', 'freeze', 'pause']):
            return 'STOP'
        elif any(word in command_lower for word in ['status', 'how are you', 'report', 'what are you doing']):
            return 'REPORT_STATUS'
        elif any(word in command_lower for word in ['look', 'see', 'find', 'detect']):
            return 'PERCEPTION_TASK'
        else:
            return 'UNKNOWN'
    
    def process_voice_command_callback(self, request, response):
        """Service callback for processing voice commands directly"""
        self.get_logger().info(f'Processing direct voice command: {request.command_text}')
        
        try:
            # Classify the intent of the command
            intent = self.classify_intent(request.command_text)
            
            # Create and publish a user command
            user_command = UserCommand()
            user_command.command_id = f'direct_cmd_{int(time.time() * 1000)}'
            user_command.command_text = request.command_text
            user_command.processed_text = request.command_text
            user_command.intent = intent
            user_command.confidence = request.confidence_threshold if request.confidence_threshold > 0 else 0.9
            user_command.status = 'RECEIVED'
            
            # Set timestamp
            current_time = self.get_clock().now()
            user_command.timestamp.sec = current_time.seconds_nanoseconds()[0]
            user_command.timestamp.nanosec = current_time.seconds_nanoseconds()[1]
            
            # Publish the command
            self.user_command_pub.publish(user_command)
            
            # Prepare response
            response.success = True
            response.action_plan_id = user_command.command_id  # In a real system, this would be the actual plan ID
            response.error_message = ''
            
            self.get_logger().info(f'Direct command processed: {request.command_text}')
            
        except Exception as e:
            response.success = False
            response.action_plan_id = ''
            response.error_message = f'Error processing voice command: {str(e)}'
            self.get_logger().error(f'Error in direct voice command processing: {e}')
        
        return response
    
    def publish_status(self):
        """Publish the current voice service status"""
        status_msg = String()
        status_msg.data = f'Listening: {self.is_listening}, Commands processed: {len(self.command_history)}'
        self.voice_status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.is_listening = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    voice_service = VoiceService()
    
    try:
        rclpy.spin(voice_service)
    except KeyboardInterrupt:
        pass
    finally:
        voice_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()