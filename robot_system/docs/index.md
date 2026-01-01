# Autonomous Humanoid Robot System - Documentation

Welcome to the documentation for the Autonomous Humanoid Robot System. This documentation provides information about the system architecture, modules, and how to use the system.

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Modules](#modules)
4. [Installation](#installation)
5. [Usage](#usage)
6. [API Reference](#api-reference)
7. [Troubleshooting](#troubleshooting)

## Overview

The Autonomous Humanoid Robot System is a comprehensive robotics platform that combines multiple technologies to create an intelligent, autonomous humanoid robot. The system consists of four core modules that work together to enable voice commands to be translated into robot actions with sim-to-real transfer capabilities.

## Architecture

The system follows a modular architecture with clear separation of concerns:

- **Robotic Nervous System (ROS 2)**: Core middleware for humanoid control
- **Digital Twin (Gazebo/Unity)**: Physics simulation and environment modeling
- **AI-Robot Brain (NVIDIA Isaac)**: Advanced perception, navigation, and learning
- **Vision-Language-Action (VLA)**: Cognitive planning and multi-modal interaction

## Modules

Each module is documented separately in the following sections:

- [Robotic Nervous System](./modules/ros2_nervous_system.md)
- [Digital Twin](./modules/digital_twin.md)
- [AI-Robot Brain](./modules/ai_robot_brain.md)
- [Vision-Language-Action](./modules/vla_module.md)

## Installation

See the [Installation Guide](./installation.md) for detailed instructions on setting up the system.

## Usage

See the [Usage Guide](./usage.md) for instructions on how to use the system.

## API Reference

See the [API Reference](./api_reference.md) for detailed information about the system's interfaces.

## Troubleshooting

See the [Troubleshooting Guide](./troubleshooting.md) for common issues and solutions.