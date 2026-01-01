# Data Model: Autonomous Humanoid Robot System

**Date**: 2025-01-01
**Feature**: Autonomous Humanoid Robot System
**Branch**: 001-autonomous-humanoid-system

## Overview

This document defines the key data entities and their relationships for the autonomous humanoid robot system. It covers the four core modules: Robotic Nervous System (ROS 2), Digital Twin (Gazebo/Unity), AI-Robot Brain (NVIDIA Isaac), and Vision-Language-Action (VLA).

## 1. Core Entities

### 1.1 Robot State

**Description**: Represents the current state of the humanoid robot including position, orientation, joint angles, and operational status.

**Fields**:
- `timestamp`: Unix timestamp of the state measurement
- `position`: 3D position vector (x, y, z) in world coordinates
- `orientation`: Quaternion (x, y, z, w) representing robot orientation
- `joint_angles`: Map of joint names to angle values (in radians)
- `sensor_readings`: Map of sensor types to current readings
- `battery_level`: Float percentage (0.0-1.0) of battery remaining
- `operational_status`: Enum (IDLE, MOVING, PROCESSING, ERROR)
- `safety_status`: Enum (SAFE, WARNING, DANGER)

**Relationships**:
- One-to-many with Sensor Data Stream (robot has multiple sensor readings over time)
- One-to-many with Action Plan (robot executes multiple action plans)

### 1.2 Environment Model

**Description**: 3D representation of the physical space including static and dynamic objects.

**Fields**:
- `timestamp`: Unix timestamp of the environment model
- `static_objects`: List of static objects with position, size, and type
- `dynamic_objects`: List of dynamic objects with position, velocity, and type
- `occupancy_grid`: 2D grid representing free space and obstacles
- `semantic_map`: Semantic labeling of environment regions
- `navigation_waypoints`: Predefined waypoints for navigation

**Relationships**:
- One-to-many with Robot State (environment model is valid for multiple robot states)
- One-to-many with Sensor Data Stream (environment model is updated from sensor data)

### 1.3 Action Plan

**Description**: Sequence of low-level motor commands to achieve a high-level goal.

**Fields**:
- `plan_id`: Unique identifier for the action plan
- `timestamp`: Unix timestamp when plan was created
- `goal_description`: Natural language description of the goal
- `action_sequence`: List of action steps with parameters
- `estimated_duration`: Estimated time to complete the plan (in seconds)
- `safety_constraints`: List of safety constraints for plan execution
- `status`: Enum (PENDING, EXECUTING, COMPLETED, FAILED, CANCELLED)

**Relationships**:
- Many-to-one with Robot State (multiple action plans can be associated with robot state)
- Many-to-one with User Command (action plan is generated from user command)

### 1.4 Sensor Data Stream

**Description**: Real-time data from various sensors (LiDAR, cameras, IMU, etc.).

**Fields**:
- `timestamp`: Unix timestamp of the sensor reading
- `sensor_type`: Enum (LIDAR, CAMERA_RGB, CAMERA_DEPTH, IMU, JOINT_ENCODER)
- `sensor_id`: Unique identifier for the specific sensor
- `data`: Sensor-specific data payload (point cloud, image, orientation, etc.)
- `frame_id`: Coordinate frame in which the data is expressed
- `quality`: Float percentage (0.0-1.0) representing data quality

**Relationships**:
- Many-to-one with Robot State (multiple sensor readings contribute to robot state)
- Many-to-one with Environment Model (sensor data updates environment model)

### 1.5 User Command

**Description**: High-level instruction from human operator in natural language or text.

**Fields**:
- `command_id`: Unique identifier for the command
- `timestamp`: Unix timestamp when command was received
- `command_text`: Original natural language command
- `processed_text`: Processed and normalized command text
- `intent`: Classified intent (NAVIGATE, PERFORM_ACTION, REPORT_STATUS, etc.)
- `parameters`: Map of extracted parameters for the command
- `confidence`: Float percentage (0.0-1.0) representing NLP confidence
- `status`: Enum (RECEIVED, PROCESSED, EXECUTING, COMPLETED, REJECTED)

**Relationships**:
- One-to-many with Action Plan (one command can generate multiple action plans)
- One-to-many with Robot State (command may affect robot state over time)

## 2. Module-Specific Entities

### 2.1 ROS 2 Message Types

**Description**: Custom message types for communication between ROS 2 nodes.

**Fields**:
- `HumanoidState.msg`:
  - `header`: Standard ROS header
  - `joint_states`: Array of joint state messages
  - `base_pose`: Pose of the robot's base
  - `center_of_mass`: 3D position of center of mass
  - `stability_index`: Float representing stability

- `ActionCommand.msg`:
  - `header`: Standard ROS header
  - `command_type`: Enum for action type
  - `parameters`: Array of float parameters
  - `target_pose`: Optional target pose for navigation
  - `execution_priority`: Integer priority level

- `PerceptionData.msg`:
  - `header`: Standard ROS header
  - `object_list`: Array of detected objects
  - `semantic_map`: Semantic segmentation data
  - `confidence_map`: Confidence values for each pixel

### 2.2 Simulation Entities

**Description**: Entities specific to the Digital Twin simulation environment.

**Fields**:
- `SimulationState`:
  - `sim_time`: Simulation time in seconds
  - `real_time_factor`: Ratio of sim time to real time
  - `physics_accuracy`: Enum (FAST, BALANCED, ACCURATE)
  - `environment_settings`: Map of environment parameters

- `SimulationObject`:
  - `object_id`: Unique identifier
  - `object_type`: Enum (ROBOT, OBSTACLE, FURNITURE, DYNAMIC_OBJECT)
  - `physical_properties`: Mass, friction, restitution, etc.
  - `visual_properties`: Color, texture, mesh, etc.

### 2.3 AI/ML Entities

**Description**: Entities related to the AI-Robot Brain and VLA modules.

**Fields**:
- `PerceptionResult`:
  - `timestamp`: Unix timestamp of perception result
  - `detected_objects`: List of detected objects with bounding boxes
  - `object_classes`: List of class labels for detected objects
  - `confidence_scores`: List of confidence scores for detections
  - `semantic_segmentation`: Segmentation mask

- `NavigationPlan`:
  - `plan_id`: Unique identifier for the navigation plan
  - `start_pose`: Starting pose for navigation
  - `goal_pose`: Goal pose for navigation
  - `path`: List of poses forming the path
  - `path_cost`: Estimated cost of the path
  - `safety_rating`: Safety assessment of the path

- `VLACommand`:
  - `command_id`: Unique identifier
  - `original_text`: Original voice/text command
  - `parsed_intent`: Parsed intent from NLP
  - `action_sequence`: Sequence of actions to execute
  - `context`: Context information for command execution
  - `execution_feedback`: Feedback from command execution

## 3. Relationships and Constraints

### 3.1 Primary Relationships

1. **Robot State ↔ Sensor Data Stream**: Robot state is updated based on sensor data streams
2. **Environment Model ↔ Sensor Data Stream**: Environment model is updated from sensor data
3. **User Command ↔ Action Plan**: User commands generate action plans
4. **Action Plan ↔ Robot State**: Action plans modify robot state during execution

### 3.2 Validation Rules

1. **Robot State Validation**:
   - Joint angles must be within physical limits
   - Position and orientation must be valid numbers
   - Battery level must be between 0.0 and 1.0

2. **Environment Model Validation**:
   - Occupancy grid values must be between 0 and 100 (percentage)
   - Object positions must be within environment bounds
   - Navigation waypoints must be reachable

3. **Action Plan Validation**:
   - Action sequences must be executable by the robot
   - Safety constraints must be satisfied
   - Estimated duration must be reasonable

4. **Sensor Data Validation**:
   - Data quality must meet minimum thresholds
   - Timestamps must be recent (within tolerance)
   - Sensor readings must be within expected ranges

### 3.3 State Transitions

1. **Robot State Transitions**:
   - IDLE → MOVING (when action plan starts executing)
   - MOVING → IDLE (when action plan completes)
   - IDLE/MOVING → ERROR (when safety violation occurs)
   - ERROR → IDLE (after error recovery)

2. **Action Plan Transitions**:
   - PENDING → EXECUTING (when execution starts)
   - EXECUTING → COMPLETED (when successfully finished)
   - EXECUTING → FAILED (when execution fails)
   - EXECUTING → CANCELLED (when cancelled by user)

## 4. Data Flow Patterns

### 4.1 Sensor Data Flow
```
Physical Sensors → Sensor Nodes → Sensor Data Stream → Robot State → Environment Model
```

### 4.2 Command Execution Flow
```
User Command → VLA Module → Action Plan → ROS 2 Nodes → Robot Actuators
```

### 4.3 Perception Flow
```
Sensor Data → AI-Robot Brain → Perception Result → Environment Model → Navigation Plan
```

This data model provides the foundation for the autonomous humanoid robot system, ensuring consistent representation of information across all modules.