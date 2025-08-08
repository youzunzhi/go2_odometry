## Overview

This is a ROS2 package for state estimation on the Unitree Go2 robot using invariant extended Kalman filters (InEKF). The package provides multiple odometry sources and converts Unitree-specific messages to standard ROS messages.

## Build and Development Commands

This is a ROS2 ament_cmake package. Common commands:

```bash
# Build the package (from workspace root)
colcon build --packages-select go2_odometry

# Build with debug symbols
colcon build --packages-select go2_odometry --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run tests
colcon test --packages-select go2_odometry

# Lint Python code (ruff configured with line-length=120)
ruff check scripts/
ruff format scripts/

# Lint C++ code (uses ament_lint_auto)
colcon test --packages-select go2_odometry --ctest-args -R lint
```

## Main Launch Files

The primary entry point is `go2_odometry_switch.launch.py` which accepts an `odom_type` parameter:

- `use_full_odom` (default): Uses InEKF-based odometry
- `fake`: Fixed position odometry for debugging
- `mocap`: Motion capture-based odometry

Example usage:
```bash
ros2 launch go2_odometry go2_odometry_switch.launch.py odom_type:=use_full_odom
```

## Architecture

### Core Components

1. **State Converter Node** (`src/go2_state_converter_node.cpp`)
   - Converts Unitree `LowState` messages to standard ROS messages
   - Publishes `/joint_states` and `/imu` topics
   - Handles joint ordering conversion from SDK to URDF format

2. **InEKF Odometry** (`scripts/inekf_odom.py`)
   - Main state estimation using invariant extended Kalman filter
   - Subscribes to `/lowstate` for IMU and joint data
   - Publishes `/tf` and `/odometry/filtered`

3. **Motion Capture Integration** (`scripts/mocap_base_pose.py`)
   - Connects to Qualisys motion capture system
   - Provides ground truth odometry when available

4. **State Publisher** (`go2_state_publisher.launch.py`)
   - Launches robot state publisher and state converter
   - Required for RVIZ visualization and TF tree

### Dependencies

Key external dependencies:
- `unitree_ros2`: Unitree robot interface
- `invariant-ekf`: InEKF library (custom fork)
- `go2_description`: URDF files (custom fork)
- `pinocchio`: Robotics library for kinematics

### Message Flow

```
/lowstate (unitree_go/LowState) -> state_converter_node -> /joint_states, /imu
/lowstate -> inekf_odom.py -> /odometry/filtered, /tf
```

## Development Notes

- Joint ordering: SDK and URDF use different joint orders, handled by `urdf_to_sdk_index_` mapping
- InEKF parameters are configurable via ROS parameters (noise covariances, frame names)
- Motion capture can operate in two modes: odometry replacement or ground truth publishing
- The package supports both C++ (state converter) and Python (estimation algorithms) nodes