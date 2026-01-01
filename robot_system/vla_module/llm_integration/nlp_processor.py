"""
NLP Processing for Command Understanding

This module implements natural language processing for understanding
voice commands and translating them into actionable intents for the robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_system.msg import UserCommand
from robot_system.srv import ProcessVoiceCommand
import openai
import time
import json
import re
from typing import Dict, List, Tuple, Optional


class NLPProcessor(Node):
    """
    Natural Language Processing module for understanding voice commands
    and translating them into actionable intents for the robot.
    """
    
    def __init__(self):
        super().__init__('nlp_processor')
        
        # Publishers
        self.processed_command_pub = self.create_publisher(UserCommand, '/processed_user_command', 10)
        self.nlp_status_pub = self.create_publisher(String, '/nlp_status', 10)
        
        # Subscribers
        self.user_command_sub = self.create_subscription(
            UserCommand, '/user_command', self.user_command_callback, 10)
        
        # Services
        self.process_text_srv = self.create_service(
            ProcessVoiceCommand, 'process_text_command', self.process_text_command_callback)
        
        # Internal state
        self.command_history = []
        self.intent_definitions = self._load_intent_definitions()
        self.entity_extractors = self._initialize_entity_extractors()
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('NLP Processor initialized')
    
    def _load_intent_definitions(self) -> Dict:
        """Load intent definitions for command classification"""
        # Define common intents and their patterns
        return {
            'NAVIGATE': {
                'patterns': [
                    r'go to (.+)',
                    r'move to (.+)',
                    r'walk to (.+)',
                    r'navigate to (.+)',
                    r'travel to (.+)',
                    r'go (.+)',
                    r'move (.+)',
                    r'head to (.+)'
                ],
                'keywords': ['go', 'move', 'walk', 'navigate', 'travel', 'head', 'to', 'toward']
            },
            'PERFORM_ACTION': {
                'patterns': [
                    r'pick up (.+)',
                    r'grasp (.+)',
                    r'grab (.+)',
                    r'take (.+)',
                    r'lift (.+)',
                    r'put (.+) (.+)',
                    r'place (.+) (.+)',
                    r'bring (.+)'
                ],
                'keywords': ['pick', 'grasp', 'grab', 'take', 'lift', 'put', 'place', 'bring', 'drop', 'release']
            },
            'STOP': {
                'patterns': [
                    r'stop',
                    r'halt',
                    r'freeze',
                    r'pause',
                    r'wait',
                    r'cease'
                ],
                'keywords': ['stop', 'halt', 'freeze', 'pause', 'wait', 'cease']
            },
            'REPORT_STATUS': {
                'patterns': [
                    r'how are you',
                    r'what are you doing',
                    r'report status',
                    r'what is your status',
                    r'tell me your status',
                    r'where are you'
                ],
                'keywords': ['how', 'what', 'report', 'status', 'where', 'doing', 'are', 'you']
            },
            'PERCEPTION_TASK': {
                'patterns': [
                    r'look at (.+)',
                    r'see (.+)',
                    r'find (.+)',
                    r'detect (.+)',
                    r'identify (.+)',
                    r'locate (.+)',
                    r'observe (.+)'
                ],
                'keywords': ['look', 'see', 'find', 'detect', 'identify', 'locate', 'observe', 'scan', 'search']
            }
        }
    
    def _initialize_entity_extractors(self) -> Dict:
        """Initialize entity extraction patterns"""
        return {
            'location': [
                r'(kitchen|living room|bedroom|bathroom|office|garage|garden|dining room|hallway|entrance)',
                r'(\d+\s*(meters|meter|feet|foot|steps?)\s*(forward|backward|left|right|ahead|up|down))'
            ],
            'object': [
                r'(ball|cup|book|phone|keys|bottle|toy|box|chair|table|door|window)',
                r'(red|blue|green|yellow|large|small|big|tiny|heavy|light)\s+\w+'
            ],
            'action': [
                r'(pick up|grasp|grab|take|lift|put down|place|set down|move|carry)'
            ]
        }
    
    def user_command_callback(self, msg):
        """Process incoming user commands"""
        self.get_logger().info(f'Processing user command: {msg.command_text}')
        
        try:
            # Process the command text
            processed_command = self.process_command(msg.command_text)
            
            # Update the original message with processed information
            msg.processed_text = processed_command['processed_text']
            msg.intent = processed_command['intent']
            msg.parameters = processed_command['parameters']
            msg.confidence = processed_command['confidence']
            msg.status = 'PROCESSED'
            
            # Publish the processed command
            self.processed_command_pub.publish(msg)
            
            # Add to command history
            self.command_history.append({
                'id': msg.command_id,
                'original': msg.command_text,
                'processed': msg.processed_text,
                'intent': msg.intent,
                'timestamp': time.time()
            })
            
            self.get_logger().info(f'Command processed with intent: {msg.intent}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
            # Update status but keep the original message
            msg.status = 'ERROR'
            msg.confidence = 0.0
            self.processed_command_pub.publish(msg)
    
    def process_command(self, command_text: str) -> Dict:
        """Process a command and return structured information"""
        # Normalize the command text
        normalized_text = self._normalize_text(command_text)
        
        # Classify intent
        intent, confidence = self._classify_intent(normalized_text)
        
        # Extract entities
        entities = self._extract_entities(normalized_text)
        
        # Create parameters from entities
        parameters = self._create_parameters(entities)
        
        return {
            'processed_text': normalized_text,
            'intent': intent,
            'parameters': parameters,
            'confidence': confidence,
            'entities': entities
        }
    
    def _normalize_text(self, text: str) -> str:
        """Normalize text for processing"""
        # Convert to lowercase
        text = text.lower()
        
        # Remove extra whitespace
        text = ' '.join(text.split())
        
        # Remove punctuation (except for specific cases)
        text = re.sub(r'[^\w\s]', ' ', text)
        
        # Remove extra spaces again after punctuation removal
        text = ' '.join(text.split())
        
        return text
    
    def _classify_intent(self, text: str) -> Tuple[str, float]:
        """Classify the intent of a command using pattern matching and keyword analysis"""
        best_intent = 'UNKNOWN'
        best_confidence = 0.0
        
        # Pattern-based classification
        for intent, definition in self.intent_definitions.items():
            # Check patterns
            for pattern in definition['patterns']:
                if re.search(pattern, text):
                    return intent, 1.0  # High confidence for pattern match
            
            # Check keywords
            text_words = set(text.split())
            intent_keywords = set(definition['keywords'])
            common_keywords = text_words.intersection(intent_keywords)
            
            if common_keywords:
                # Calculate confidence based on keyword overlap
                confidence = len(common_keywords) / len(intent_keywords)
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_intent = intent
        
        # If no high-confidence match, use a default threshold
        if best_confidence < 0.3:
            best_intent = 'UNKNOWN'
            best_confidence = 0.0
        
        return best_intent, best_confidence
    
    def _extract_entities(self, text: str) -> Dict[str, List[str]]:
        """Extract entities from the command text"""
        entities = {}
        
        for entity_type, patterns in self.entity_extractors.items():
            found_entities = []
            for pattern in patterns:
                matches = re.findall(pattern, text, re.IGNORECASE)
                for match in matches:
                    if isinstance(match, tuple):
                        found_entities.extend([m.strip() for m in match if m.strip()])
                    else:
                        found_entities.append(match.strip())
            
            entities[entity_type] = list(set(found_entities))  # Remove duplicates
        
        return entities
    
    def _create_parameters(self, entities: Dict[str, List[str]]) -> List:
        """Create parameters from extracted entities"""
        parameters = []
        
        # Convert entities to parameters format
        for entity_type, entity_values in entities.items():
            for value in entity_values:
                # Create a simple key-value parameter
                param = {
                    'key': entity_type,
                    'value': value
                }
                parameters.append(param)
        
        return parameters
    
    def process_text_command_callback(self, request, response):
        """Service callback for processing text commands directly"""
        self.get_logger().info(f'Processing direct text command: {request.command_text}')
        
        try:
            # Process the command
            processed = self.process_command(request.command_text)
            
            # Prepare response
            response.success = True
            response.action_plan_id = f'plan_{int(time.time())}'  # Generate a plan ID
            response.error_message = ''
            
            # Create and publish a processed command
            user_command = UserCommand()
            user_command.command_id = f'text_cmd_{int(time.time() * 1000)}'
            user_command.command_text = request.command_text
            user_command.processed_text = processed['processed_text']
            user_command.intent = processed['intent']
            user_command.parameters = processed['parameters']
            user_command.confidence = processed['confidence']
            user_command.status = 'PROCESSED'
            
            # Set timestamp
            current_time = self.get_clock().now()
            user_command.timestamp.sec = current_time.seconds_nanoseconds()[0]
            user_command.timestamp.nanosec = current_time.seconds_nanoseconds()[1]
            
            # Publish the processed command
            self.processed_command_pub.publish(user_command)
            
            self.get_logger().info(f'Text command processed with intent: {processed["intent"]}')
            
        except Exception as e:
            response.success = False
            response.action_plan_id = ''
            response.error_message = f'Error processing text command: {str(e)}'
            self.get_logger().error(f'Error in text command processing: {e}')
        
        return response
    
    def publish_status(self):
        """Publish the current NLP processor status"""
        status_msg = String()
        status_msg.data = f'Commands processed: {len(self.command_history)}'
        self.nlp_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    nlp_processor = NLPProcessor()
    
    try:
        rclpy.spin(nlp_processor)
    except KeyboardInterrupt:
        pass
    finally:
        nlp_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()