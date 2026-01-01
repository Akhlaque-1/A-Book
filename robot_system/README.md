# Autonomous Humanoid Robot System

This repository contains the implementation of an autonomous humanoid robot system with four core modules: (1) Robotic Nervous System (ROS 2) for communication between sensors, actuators, and AI modules, (2) Digital Twin (Gazebo/Unity) for physics simulation and environment modeling, (3) AI-Robot Brain (NVIDIA Isaac) for perception and navigation, and (4) Vision-Language-Action (VLA) for cognitive planning and multi-modal interaction. The system enables voice commands to be translated into robot actions with sim-to-real transfer capabilities.

## Features

- Voice command processing and natural language understanding
- Multi-modal perception and sensor fusion
- Safe navigation and obstacle avoidance
- Sim-to-real transfer capabilities
- Modular architecture for easy extension

## Architecture

The system consists of four main modules:

1. **Robotic Nervous System (ROS 2)**: Core middleware for humanoid control; manages communication between sensors, actuators, and AI modules
2. **Digital Twin (Gazebo/Unity)**: Physics simulation and environment modeling; virtual testing for humanoid robots
3. **AI-Robot Brain (NVIDIA Isaac)**: Advanced perception, navigation, and learning for humanoid robots
4. **Vision-Language-Action (VLA)**: Cognitive planning and multi-modal interaction; converts human instructions to robot actions

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA Isaac ROS packages
- Gazebo Garden
- Unity 2022.3 LTS (optional, for high-fidelity visualization)
- Python 3.8+
- CUDA 11.8+ (for NVIDIA hardware acceleration)

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd robot_system
   ```

2. Install ROS 2 dependencies:
   ```bash
   cd ~/robot_system_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build --packages-select ros2_nervous_system digital_twin ai_robot_brain vla_module
   ```

4. Source the workspace:
   ```bash
   source ~/robot_system_ws/install/setup.bash
   ```

## Usage

1. Launch the Robotic Nervous System:
   ```bash
   ros2 launch ros2_nervous_system core_system.launch.py
   ```

2. Launch the Digital Twin (Simulation):
   ```bash
   ros2 launch digital_twin simulation.launch.py
   ```

3. Launch the AI-Robot Brain:
   ```bash
   ros2 launch ai_robot_brain perception_nav.launch.py
   ```

4. Launch the VLA Module:
   ```bash
   ros2 launch vla_module vla_system.launch.py
   ```

## Contributing

We welcome contributions to this project. Please read our contributing guidelines before submitting a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- This project was developed using the Spec-Driven Development methodology
- Special thanks to the ROS 2 and NVIDIA Isaac communities for their excellent tools and documentation