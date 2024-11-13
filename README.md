Go2 Odometry
===

## Summary
Provide a simple state estimation for the unitree Go2 robot, via the [robot_localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package.
It also provides a simple node to convert unitree custom messages into "standard" ros messages and re-publish them on separate topics.

## Launchfiles

### go2_state_publisher.launch.py
starts the following nodes

##### go2_odometry/state_converter_node.cpp
Listen to `unitree_ros2/LowState` messages and split them into "standard" ros messages. It also re-arrange the joint order to match urdf order (which is not the order sent by the unitree robot).

However, because the `unitree_ros2/LowState` msg is not timestamped, the node also listen to `utlidar/imu` to get a time stamp and republish it on `/clock` topic for time synchronization between the computer and the robot.
(Because this node also listen to this clock, all the message are timestamped with it)

Finally the published topics are the following:
* `/imu`: populated by `unitree_ros2/LowState`.`imu`
* `/joint_states`: populated by `unitree_ros2/LowState`.`motor_state`.<`d`/`dq`/...>
* `/clock`: populated by `utlidar/imu`

##### robot_state_publisher
Standard ros node that listen to `/joint_states` topic and publish TF transform for all link of a robot.

Published topics:
* `/tf` and `/tf_static`: Link relative positions, populated from `/joint_states`.
* `/robot_description`:  topic for other nodes to get the urdf of the robot.

### go2_full_odomertry.launch.py
This launchfile first launches the former **go2_state_publisher.launch.py**.
The other nodes launched are:

##### robot_localization/ekf_node
A standard ros2 node for running basic Extended Kalman Filters.

This Kalman is tuned to listen to:
* `/imu`: published in this case by the **go2_state_converter** node.
* `/tf`: populated in this case by the **robot_state_publisher** node.
* `/odometry/feet_pos` and `/odometry/feet_vel`: for XY speed and Z correction of the estimate. (See next node)

It then publishes on:
* `/tf`: The floating base pose estimation
* `/odometry/filtered`: The same pose estimate with covariances.

##### go2_odometry/feet_to_odom.py
Run a inverse kinematics on the robot joints to get a height estimate and a XY velocity estimate (assuming the feet are sticking to the ground).

This node subscribe to:
* `unitree_ros2/LowState`: For joint configuration and velocity, as-well as for **foot_force** to determine if the foot is in contact with the ground.

It the publishes on:
*`/odometry/feet_pos` and `/odometry/feet_vel`: For the ekf node to consume.