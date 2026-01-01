# Usage Guide

This guide provides instructions for using the Autonomous Humanoid Robot System.

## Getting Started

Once you have installed the system, you can start using it by launching the different modules.

## Launching the System

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

## Voice Command Execution

The system supports voice command execution. To use this feature:

1. Ensure all modules are running
2. Speak a command like "Move forward 2 meters" or "Go to the kitchen"
3. Observe the robot's response in the terminal and through the robot's actions

### Supported Voice Commands

- Navigation: "Go to the kitchen", "Move forward 2 meters", "Navigate to the living room"
- Actions: "Pick up the red ball", "Grasp the cup", "Take the book"
- Status: "What is your status?", "How are you?", "Report status"
- Perception: "Look at the table", "Find the person", "Detect objects"

## Simulation to Real-World Transfer

The system supports sim-to-real transfer of behaviors. To use this feature:

1. Develop and test behaviors in the simulation environment
2. Deploy the same algorithm to the physical robot
3. The system will adapt to real-world conditions with minimal retraining

## Multi-Modal Perception and Response

The robot can perceive its environment through multiple sensors (vision, LiDAR, IMU) and respond appropriately:

1. The robot will continuously process sensor data
2. Objects and obstacles will be detected and mapped
3. The robot will respond appropriately to dynamic situations

### Sensor Data Monitoring

You can monitor sensor data streams:

```bash
# Monitor sensor data streams
ros2 topic echo /sensor_data_stream
```

## Validation and Testing

### Check System Status

```bash
# List active ROS 2 nodes
ros2 node list

# List active topics
ros2 topic list

# Check the robot state
ros2 topic echo /robot_state
```

### Run Basic Tests

```bash
# Navigate to the workspace
cd ~/robot_system_ws

# Run unit tests
colcon test --packages-select ros2_nervous_system digital_twin ai_robot_brain vla_module

# View test results
colcon test-result --all
```

### Execute Test Scenarios

```bash
# Run a simple navigation test in simulation
source ~/robot_system_ws/install/setup.bash
ros2 launch robot_system_tests navigation_test.launch.py
```

## Troubleshooting

### Common Issues

1. **No Response to Voice Commands**:
   - Check that the VLA module is running
   - Verify that the microphone is properly connected and configured
   - Ensure the activation keyword is being used ("Hey Robot" by default)

2. **Navigation Issues**:
   - Verify that the perception module is running
   - Check that sensor data is being received
   - Ensure the navigation goal is reachable and safe

3. **Simulation Performance**:
   - Reduce physics accuracy settings in Gazebo if performance is poor
   - Close unnecessary applications to free up system resources

### Debugging Tips

- Use `rqt` for visual debugging of ROS 2 topics and services
- Monitor system resources with `htop` or `nvidia-smi`
- Check ROS 2 logs in `~/.ros/log/`
- Use `ros2 doctor` to diagnose common system issues

## Next Steps

1. Explore the tutorials in the `docs/tutorials/` directory
2. Run the complete system integration tests
3. Begin developing custom behaviors for your specific use case
4. Experiment with different environments and scenarios