## Overview

ROS2 package for state estimation on the Unitree Go2 using invariant extended Kalman filters (InEKF). Converts Unitree-specific messages to standard ROS messages and provides multiple odometry sources.

## Build and Development Commands

```bash
colcon build --packages-select go2_odometry

```

## Main Launch Files

Entry point: `go2_odometry_switch.launch.py` with `odom_type` parameter (`use_full_odom` | `fake` | `mocap`).

```bash
ros2 launch go2_odometry go2_odometry_switch.launch.py odom_type:=use_full_odom
```

## Architecture

- **State Converter** (`src/go2_state_converter_node.cpp`): Converts `/lowstate` to `/joint_states` and `/imu`. Handles SDK-to-URDF joint reordering via `urdf_to_sdk_index_`.
- **InEKF Odometry** (`scripts/inekf_odom.py`): Subscribes to `/lowstate`, publishes `/tf` and `/odometry/filtered`. Noise parameters configurable via ROS params.
- **Motion Capture** (`scripts/mocap_base_pose.py`): Qualisys integration for ground truth odometry.
- **State Publisher** (`go2_state_publisher.launch.py`): Robot state publisher + state converter for TF tree / RVIZ.

Dependencies: `unitree_ros2`, `invariant-ekf` (custom fork), `go2_description` (custom fork), `pinocchio`.

```
/lowstate -> state_converter_node -> /joint_states, /imu
/lowstate -> inekf_odom.py -> /odometry/filtered, /tf
```

## Debug Scripts

All scripts support `--help` for full option details.

- **`save_sensor_msgs.py`**: Records `/lowstate`, `/utlidar/imu`, `/utlidar/robot_odom` to a rosbag2 SQLite database.
  ```bash
  python3 debug_scripts/save_sensor_msgs.py --duration 10
  ```

- **`compare_bag_inekf_odometry.py`**: Replays a recorded bag through two parallel InEKF pipelines (`lowstate` and `utlidar` IMU sources), compares against `/utlidar/robot_odom` reference. Outputs CSV and plots to `comparison_output/`.
  ```bash
  python3 debug_scripts/compare_bag_inekf_odometry.py --bag path/to/bag/
  ```
  The `utlidar` IMU is mounted with a different orientation/position than the body frame. Use `--imu-rotation-rpy` (radians) and `--imu-translation-xyz` (meters) to specify the extrinsic transform; add `--compensate-imu-translation` to correct for lever-arm effects on the accelerometer.

- **`pose_comparison.py`**: Live ROS2 node comparing two odometry topics in real-time with time-synchronized pairs. Outputs plots/CSV after a fixed duration.