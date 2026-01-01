# Troubleshooting Guide

This guide provides solutions for common issues you may encounter when using the Autonomous Humanoid Robot System.

## Common Issues

### 1. ROS 2 Communication Issues

**Problem**: Nodes are not communicating properly.

**Solutions**:
- Check that all nodes are on the same ROS domain: `echo $ROS_DOMAIN_ID`
- Verify network configuration if running across multiple machines
- Ensure firewall settings allow ROS 2 communication
- Check that the ROS master URI is properly set: `echo $ROS_MASTER_URI`
- Verify that the ROS IP is correctly configured: `echo $ROS_IP`

### 2. Voice Command Processing Issues

**Problem**: The system is not responding to voice commands.

**Solutions**:
- Verify that the VLA module is running: `ros2 run vla_module voice_service`
- Check that the microphone is properly connected and configured
- Ensure the activation keyword is being used ("Hey Robot" by default)
- Check audio input permissions and settings
- Verify that the speech recognition service is working properly

### 3. Navigation Issues

**Problem**: The robot is not navigating properly or is unable to plan paths.

**Solutions**:
- Verify that the perception module is running
- Check that sensor data is being received and processed
- Ensure the navigation goal is reachable and safe
- Verify that the environment map is properly updated
- Check for obstacles in the navigation path

### 4. Simulation Performance Issues

**Problem**: The simulation is running slowly or with poor performance.

**Solutions**:
- Reduce physics accuracy settings in Gazebo if performance is poor
- Close unnecessary applications to free up system resources
- Consider using a more powerful machine for complex simulations
- Check that GPU acceleration is properly configured
- Reduce the complexity of the simulation environment

### 5. Sensor Data Issues

**Problem**: Sensor data is not being received or processed correctly.

**Solutions**:
- Verify that sensor nodes are running
- Check sensor connections and configurations
- Verify that sensor topics are being published: `ros2 topic list | grep sensor`
- Check sensor calibration parameters
- Ensure proper lighting conditions for camera sensors

### 6. Safety System Activation

**Problem**: The safety system is frequently activating and stopping the robot.

**Solutions**:
- Check safety parameters in the safety monitor configuration
- Verify that sensor data is accurate and not triggering false safety alerts
- Adjust safety thresholds if appropriate for your use case
- Check for environmental factors that might trigger safety responses

## Debugging Tips

### Using ROS 2 Tools

- Use `rqt` for visual debugging of ROS 2 topics and services:
  ```bash
  rqt
  ```

- Monitor system resources with `htop` or `nvidia-smi`:
  ```bash
  htop
  nvidia-smi
  ```

- Check ROS 2 logs in `~/.ros/log/`

- Use `ros2 doctor` to diagnose common system issues:
  ```bash
  ros2 doctor
  ```

### Checking System Status

- List active ROS 2 nodes:
  ```bash
  ros2 node list
  ```

- List active topics:
  ```bash
  ros2 topic list
  ```

- Check the robot state:
  ```bash
  ros2 topic echo /robot_state
  ```

- Check service availability:
  ```bash
  ros2 service list
  ```

### Verbose Logging

To enable more detailed logging for debugging:

1. Set the logging level to DEBUG:
   ```bash
   export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
   ```

2. Or set the logging level for a specific node:
   ```bash
   ros2 run package_name node_name --ros-args --log-level DEBUG
   ```

## Performance Optimization

### CPU Usage

- Monitor CPU usage with `htop` to identify high-usage processes
- Consider reducing the update rate of high-frequency processes
- Optimize perception algorithms for performance

### Memory Usage

- Monitor memory usage with `free -h`
- Check for memory leaks in long-running processes
- Consider using more efficient data structures

### Network Usage

- Monitor network usage with `nethogs` or `iftop`
- Reduce the frequency of large data transmissions
- Compress sensor data when possible

## Getting Help

If you encounter an issue not covered in this guide:

1. Check the project's GitHub issues page for similar problems
2. Review the documentation for configuration options
3. Consider creating a minimal example that reproduces the issue
4. When asking for help, provide:
   - Your system configuration (OS, ROS version, hardware)
   - The exact steps to reproduce the issue
   - Any relevant error messages or logs
   - What you expected to happen vs. what actually happened