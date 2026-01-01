# Installation Guide

This guide provides instructions for setting up and installing the Autonomous Humanoid Robot System.

## Prerequisites

Before installing the system, ensure you have the following prerequisites:

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA Isaac ROS packages
- Gazebo Garden
- Unity 2022.3 LTS (optional, for high-fidelity visualization)
- Python 3.8+
- CUDA 11.8+ (for NVIDIA hardware acceleration)

## System Setup

### 1. Install ROS 2 Humble

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

### 2. Install NVIDIA Isaac Setup

```bash
# Install NVIDIA Isaac ROS dependencies
sudo apt install ros-humble-isaac-ros-dev

# Install Isaac ROS navigation packages
sudo apt install ros-humble-isaac-ros-nav2 ros-humble-isaac-ros-buffers ros-humble-isaac-ros-common
```

## Clone and Build the Project

### 1. Create workspace

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

### 2. Environment Setup

```bash
# Source the workspace
source ~/robot_system_ws/install/setup.bash

# Add to bashrc for persistent sourcing
echo "source ~/robot_system_ws/install/setup.bash" >> ~/.bashrc
```

## Verification

After installation, verify that the system is properly set up:

```bash
# Check that all nodes are available
ros2 node list

# Check that all topics are available
ros2 topic list

# Run basic tests
cd ~/robot_system_ws
colcon test --packages-select ros2_nervous_system digital_twin ai_robot_brain vla_module
colcon test-result --all
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