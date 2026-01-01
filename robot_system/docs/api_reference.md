# API Reference

This document provides a reference for the main APIs and interfaces in the Autonomous Humanoid Robot System.

## ROS 2 Messages

### RobotState

Represents the current state of the humanoid robot including position, orientation, joint angles, and operational status.

- `header`: std_msgs/Header
- `pose`: geometry_msgs/Pose
- `joint_states`: sensor_msgs/JointState
- `battery_level`: float64
- `operational_status`: string (IDLE, MOVING, PROCESSING, ERROR)
- `safety_status`: string (SAFE, WARNING, DANGER)
- `timestamp`: builtin_interfaces/Time

### UserCommand

Represents a high-level instruction from human operator in natural language or text.

- `header`: std_msgs/Header
- `command_id`: string
- `command_text`: string
- `processed_text`: string
- `intent`: string (NAVIGATE, PERFORM_ACTION, REPORT_STATUS, etc.)
- `parameters`: KeyValue[] (key-value pairs of extracted parameters)
- `confidence`: float64 (0.0-1.0)
- `status`: string (RECEIVED, PROCESSED, EXECUTING, COMPLETED, REJECTED)
- `timestamp`: builtin_interfaces/Time

### ActionPlan

Represents a sequence of low-level motor commands to achieve a high-level goal.

- `header`: std_msgs/Header
- `plan_id`: string
- `goal_description`: string
- `action_sequence`: ActionStep[] (list of action steps)
- `estimated_duration`: float64
- `safety_constraints`: string[] (list of safety constraints)
- `status`: string (PENDING, EXECUTING, COMPLETED, FAILED, CANCELLED)
- `timestamp`: builtin_interfaces/Time

### SensorDataStream

Represents real-time data from various sensors (LiDAR, cameras, IMU, etc.).

- `header`: std_msgs/Header
- `timestamp`: builtin_interfaces/Time
- `sensor_type`: string (LIDAR, CAMERA_RGB, CAMERA_DEPTH, IMU, JOINT_ENCODER)
- `sensor_id`: string
- `point_cloud`: sensor_msgs/PointCloud2 (for LiDAR data)
- `image`: sensor_msgs/Image (for camera data)
- `imu_data`: sensor_msgs/Imu (for IMU data)
- `joint_data`: sensor_msgs/JointState (for joint encoder data)
- `frame_id`: string
- `quality`: float64 (0.0-1.0)

### PerceptionResult

Represents the output of perception processing including detected objects and classifications.

- `header`: std_msgs/Header
- `timestamp`: builtin_interfaces/Time
- `detected_objects`: DetectedObject[] (list of detected objects)
- `object_classes`: string[] (list of class labels)
- `confidence_scores`: float64[] (list of confidence scores)
- `semantic_segmentation`: sensor_msgs/Image (optional)
- `confidence_map`: sensor_msgs/Image (optional)

## ROS 2 Services

### ProcessVoiceCommand

Service for processing voice commands and translating them into executable action sequences.

Request:
- `command_text`: string
- `confidence_threshold`: float

Response:
- `success`: bool
- `action_plan_id`: string
- `error_message`: string

### PlanNavigation

Service for planning navigation paths between start and goal poses.

Request:
- `start_pose`: geometry_msgs/Pose
- `goal_pose`: geometry_msgs/Pose
- `map_name`: string

Response:
- `success`: bool
- `path`: nav_msgs/Path
- `estimated_time`: float
- `error_message`: string

### DetectObjects

Service for detecting objects in sensor data.

Request:
- `sensor_data`: sensor_msgs/Image
- `detection_threshold`: float

Response:
- `success`: bool
- `objects`: array of objects with class_name, confidence, and bounding box
- `error_message`: string

## ROS 2 Actions

### MoveHumanoid

Action for moving the humanoid robot to a target pose.

Goal:
- `target_pose`: geometry_msgs/Pose
- `movement_type`: string ("walk", "step", "crawl")

Feedback:
- `current_pose`: geometry_msgs/Pose
- `progress`: float (0.0 to 1.0)
- `status`: string

Result:
- `success`: bool
- `final_pose`: geometry_msgs/Pose
- `error_message`: string

## Main Modules

### VLA Module

The Vision-Language-Action module handles voice processing, natural language understanding, and action planning.

#### VoiceService
- Handles speech recognition and initial processing of voice commands
- Publishes UserCommand messages to `/user_command` topic
- Provides ProcessVoiceCommand service

#### NLPProcessor
- Processes user commands using natural language processing
- Classifies command intent and extracts entities
- Publishes processed commands to `/processed_user_command` topic

#### ActionPlanner
- Converts high-level commands into executable action sequences
- Generates ActionPlan messages based on command intent
- Publishes action plans to `/action_plan` topic

### AI-Robot Brain

The AI-Robot Brain module handles perception, navigation, and learning.

#### SensorFusionService
- Combines data from multiple sensors to create a coherent understanding of the environment
- Publishes fused sensor data to `/fused_sensor_data` topic
- Provides object detection capabilities

#### ObjectDetectionService
- Identifies and locates objects in the environment using computer vision
- Publishes detection results to `/detection_result` topic
- Provides DetectObjects service

#### EnvironmentModelingService
- Creates a representation of the environment based on sensor data
- Maintains environment maps and occupancy grids
- Provides semantic mapping capabilities

#### ObstacleAvoidanceService
- Detects obstacles and performs avoidance maneuvers
- Publishes velocity commands to `/cmd_vel` topic
- Provides collision warnings

### ROS 2 Nervous System

The ROS 2 Nervous System provides communication between different modules.

#### VLAInterface
- Interfaces between the VLA module and the ROS 2 nervous system
- Handles communication between high-level cognitive planning and low-level robot control
- Sends commands to the robot for execution

#### SensorFusionNode
- Integrates the perception system with the ROS 2 nervous system
- Handles sensor data flow and coordination between different system components
- Provides fused sensor data to other modules