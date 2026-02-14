source ~/go2_description/install/setup.bash
source install/setup.bash
ros2 launch go2_odometry go2_odometry_switch.launch.py odom_type:=use_full_odom imu_source:=utlidar