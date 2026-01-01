# Quickstart Guide: Autonomous Humanoid Robot System

**Date**: 2025-01-01
**Feature**: Autonomous Humanoid Robot System
**Branch**: 001-autonomous-humanoid-system

## Overview

This guide provides instructions for setting up and running the Autonomous Humanoid Robot System. The system consists of four core modules: Robotic Nervous System (ROS 2), Digital Twin (Gazebo/Unity), AI-Robot Brain (NVIDIA Isaac), and Vision-Language-Action (VLA).

## Prerequisites

### Hardware Requirements
- NVIDIA Jetson Orin (for edge deployment) or equivalent development machine
- Humanoid robot platform with ROS 2 compatibility
- Sensors: LiDAR, RGB-D camera, IMU
- Microphone and speakers for voice interaction

### Software Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA Isaac ROS packages
- Gazebo Garden
- Unity 2022.3 LTS (optional, for high-fidelity visualization)
- Python 3.8+
- CUDA 11.8+ (for NVIDIA hardware acceleration)

## Installation

### 1. System Setup

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. NVIDIA Isaac Setup

```bash
# Install NVIDIA Isaac ROS dependencies
sudo apt install ros-humble-isaac-ros-dev

# Install Isaac ROS navigation packages
sudo apt install ros-humble-isaac-ros-nav2 ros-humble-isaac-ros-buffers ros-humble-isaac-ros-common
```

### 3. Clone and Build the Project

```bash
# Create workspace
mkdir -p ~/robot_system_ws/src
cd ~/robot_system_ws/src

# Clone the project
git clone <repository-url> robot_system

# Install dependencies
cd ~/robot_system_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select ros2_nervous_system digital_twin ai_robot_brain vla_module
```

### 4. Environment Setup

```bash
# Source the workspace
source ~/robot_system_ws/install/setup.bash

# Add to bashrc for persistent sourcing
echo "source ~/robot_system_ws/install/setup.bash" >> ~/.bashrc
```

## Running the System

### 1. Launch the Robotic Nervous System

```bash
# Source the workspace
source ~/robot_system_ws/install/setup.bash

# Launch the core ROS 2 system
ros2 launch ros2_nervous_system core_system.launch.py
```

### 2. Launch the Digital Twin (Simulation)

```bash
# In a new terminal, source the workspace
source ~/robot_system_ws/install/setup.bash

# Launch Gazebo simulation
ros2 launch digital_twin simulation.launch.py
```

### 3. Launch the AI-Robot Brain

```bash
# In a new terminal, source the workspace
source ~/robot_system_ws/install/setup.bash

# Launch perception and navigation
ros2 launch ai_robot_brain perception_nav.launch.py
```

### 4. Launch the VLA Module

```bash
# In a new terminal, source the workspace
source ~/robot_system_ws/install/setup.bash

# Launch voice processing and action planning
ros2 launch vla_module vla_system.launch.py
```

## Basic Commands

### Voice Command Execution
1. Ensure all modules are running
2. Speak a command like "Move forward 2 meters" or "Go to the kitchen"
3. Observe the robot's response in the terminal and through the robot's actions

### Navigation Command
```bash
# Send a navigation goal programmatically
ros2 action send_goal /move_humanoid humanoid_robot_msgs/action/MoveHumanoid "{target_pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, movement_type: 'walk'}"
```

### Sensor Data Monitoring
```bash
# Monitor sensor data streams
ros2 topic echo /sensor_data_stream
```

## Validation

### 1. Check System Status
```bash
# List active ROS 2 nodes
ros2 node list

# List active topics
ros2 topic list

# Check the robot state
ros2 topic echo /robot_state
```

### 2. Run Basic Tests
```bash
# Navigate to the workspace
cd ~/robot_system_ws

# Run unit tests
colcon test --packages-select ros2_nervous_system digital_twin ai_robot_brain vla_module

# View test results
colcon test-result --all
```

### 3. Execute Test Scenarios
```bash
# Run a simple navigation test in simulation
source ~/robot_system_ws/install/setup.bash
ros2 launch robot_system_tests navigation_test.launch.py
```

## Troubleshooting

### Common Issues

1. **CUDA/GPU Issues**:
   - Ensure NVIDIA drivers are properly installed
   - Verify CUDA version compatibility with Isaac packages
   - Check that the system recognizes the GPU: `nvidia-smi`

2. **ROS 2 Communication Issues**:
   - Verify all nodes are on the same ROS domain: `echo $ROS_DOMAIN_ID`
   - Check network configuration if running across multiple machines
   - Ensure firewall settings allow ROS 2 communication

3. **Simulation Performance**:
   - Reduce physics accuracy settings in Gazebo if performance is poor
   - Close unnecessary applications to free up system resources
   - Consider using a more powerful machine for complex simulations

### Debugging Tips

- Use `rqt` for visual debugging of ROS 2 topics and services
- Monitor system resources with `htop` or `nvidia-smi`
- Check ROS 2 logs in `~/.ros/log/`
- Use `ros2 doctor` to diagnose common system issues

## Next Steps

1. Review the detailed architecture documentation in the `docs/architecture/` directory
2. Explore the tutorials in the `docs/tutorials/` directory
3. Run the complete system integration tests
4. Begin developing custom behaviors for your specific use case