# Robotic Nervous System (ROS 2) Module

This document provides information about the Robotic Nervous System module, which serves as the core middleware for humanoid control and manages communication between sensors, actuators, and AI modules.

## Overview

The Robotic Nervous System is the central communication layer of the Autonomous Humanoid Robot System. It implements ROS 2 architecture with nodes, topics, services, and actions to facilitate communication between different system components.

## Architecture

The module follows ROS 2 architecture patterns:

- **Nodes**: Individual processes that perform specific functions
- **Topics**: Asynchronous message passing between nodes
- **Services**: Synchronous request/response communication
- **Actions**: Goal-oriented communication with feedback for long-running tasks

## Components

### Sensor Nodes

Handles communication with various sensors including LiDAR, cameras, and IMU units.

- Publishes sensor data to `/sensor_data_stream` topic
- Subscribes to sensor-specific topics like `/lidar_points`, `/camera/image_raw`, `/imu/data`

### Actuator Nodes

Controls the robot's actuators and motors.

- Subscribes to command topics like `/cmd_vel`, `/joint_commands`
- Publishes feedback about actuator status

### VLA Interface

Provides the interface between the Vision-Language-Action module and the ROS 2 nervous system.

- Handles communication between high-level cognitive planning and low-level robot control
- Translates high-level commands into low-level actuator commands

## Communication Patterns

### Topics

- `/robot_state` - Current state of the robot
- `/sensor_data_stream` - Stream of sensor data from various sources
- `/user_command` - User commands from the VLA module
- `/action_plan` - Action plans from the VLA module
- `/cmd_vel` - Velocity commands for navigation
- `/joint_commands` - Joint position/velocity commands
- `/joint_states` - Current joint states

### Services

- `process_voice_command` - Process voice commands
- `plan_navigation` - Plan navigation routes
- `detect_objects` - Detect objects in sensor data

### Actions

- `move_humanoid` - Move the humanoid robot to a target pose

## Configuration

The module can be configured through ROS 2 parameters and launch files:

- `config/` directory contains configuration files
- `launch/` directory contains launch files for different scenarios
- Parameters can be set at runtime using ROS 2 parameter system

## Integration

The Robotic Nervous System integrates with other modules:

- Receives user commands from the VLA module
- Provides sensor data to the AI-Robot Brain module
- Sends actuator commands based on plans from the VLA module
- Interfaces with the Digital Twin for simulation data