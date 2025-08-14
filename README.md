# Isaac ROS Visual SLAM with RealSense D455

This project implements visual SLAM capabilities on NVIDIA Jetson Xavier using Isaac ROS 2.1 and RealSense D455 camera for localization and navigation applications.

## Platform Requirements

- **Hardware**: NVIDIA Jetson Xavier AGX
- **JetPack**: 5.1
- **Isaac ROS**: 2.1 (local compilation required)

> **Note**: As of June 30, 2025, the Isaac ROS Buildfarm for Isaac ROS 2.1 on Ubuntu 20.04 Focal is no longer supported. Local compilation is mandatory.

## Purpose

This workspace provides visual SLAM functionality combining Isaac ROS Visual SLAM 2.1 with RealSense D455 camera to deliver:

- **Localization**: Map-to-odom transformation with loop closure detection
- **Odometry**: Camera-to-odom transformation for real-time pose estimation
- **Navigation**: Integration with Nav2 navigation stack

## Environment

The project runs within the official Isaac ROS container environment with:

- **ROS Distribution**: ROS2 Humble
- **librealsense**: v2.55.1
- **RealSense Firmware**: 5.13.0.50
- **RealSense ROS Driver**: 4.51.1