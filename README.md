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

- **Container ROS Distribution**: ROS2 Humble
- **Host ROS Distribution**: ROS2 Foxy
- **librealsense**: v2.55.1
- **RealSense Firmware**: 5.13.0.50
- **RealSense ROS Driver**: 4.51.1

## Installation

Before using the system, you need to build the required packages using colcon. Build the following packages individually:

```bash
# Build Isaac ROS Common package
colcon build --packages-select isaac_ros_common

# Build Isaac ROS Nitros
colcon build --packages-select isaac_ros_nitros

# Build Isaac ROS Visual SLAM package
colcon build --packages-select isaac_ros_visual_slam
```

> **Note**: The `nvblox` package can be skipped during compilation as it is intended for future 2D costmap conversion functionality, which is not required for current operations. However, the other three packages must be successfully compiled for the system to function properly.

After building, source the workspace:

```bash
source install/setup.bash
```

## Usage

### Demos
Below are two short demo clips stored in the `demo/` folder so they render inline on GitHub. Use the playback controls to view them.

<p align="center">
	<video src="demo/demo%201.mov" width="640" controls loop muted playsinline></video>
</p>

<p align="center">
	<video src="demo/demo%202.mov" width="640" controls loop muted playsinline></video>
</p>

### Container Setup

On the BearCar platform, first enter the Isaac ROS container:

```bash
# Start the Isaac ROS container
docker start -ai isaac_ros_dev-aarch64-container

# For additional terminals, use:
docker exec -it isaac_ros_dev-aarch64-container /bin/bash

# Don't forget to source the workspace in each terminal:
source install/setup.bash
```

### Basic Launch

Start RealSense camera and Visual SLAM:

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

To visualize the SLAM data in RViz:

```bash
rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz
```

### Performance Optimization for Nav2

For Nav2 integration, optimize performance by adjusting these parameters:

**RealSense Node:**
```python
'depth_module.profile': '424x240x60'  # Lower resolution for better performance
```

**Visual SLAM Node:**
```python
'enable_slam_visualization': False    # Disabled for performance
'enable_landmarks_view': False        # Disabled for performance
'enable_observations_view': False     # Disabled for performance
'denoise_input_images': False         # Set to True for low light (higher CPU/GPU cost)
```

### Frame Drop Reduction

To reduce frame drops, adjust the jitter threshold:

```python
'img_jitter_threshold_ms': 100.00     # Increased threshold to reduce frame drops
```

> **Warning**: Higher threshold may impact accuracy.

### IMU Fusion

Enable IMU fusion for improved tracking:

```python
'enable_imu_fusion': True
```

For cameras with IMU drift, tune noise parameters:

```python
'gyro_noise_density': 0.000244
'gyro_random_walk': 0.000019393
'accel_noise_density': 0.001862
'accel_random_walk': 0.003
```

### Frame Configuration

**With Nav2 (complete TF tree):**
```python
'base_frame': 'base_link'
```

**Standalone VSLAM:**
```python
'base_frame': 'camera_link'
```

> **Note**: Using incorrect base_frame will cause TF tree errors.

### Rosbag Playback

Test SLAM with recorded data:

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_with_bag.launch.py
```

Or run bag playback separately, then launch SLAM:

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

For compressed image rosbags:

```bash
ros2 launch isaac_ros_visual_slam isaac_compressed_bag_vslam.launch.py
```

### Standalone Components

**RealSense camera only:**
```bash
ros2 launch isaac_ros_visual_slam isaac_only_realsense.launch.py
```