#!/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from unitree_go.msg import LowState

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor as PD

from inekf_core import (
    DEFAULT_ACCELEROMETER_BIAS_NOISE,
    DEFAULT_ACCELEROMETER_NOISE,
    DEFAULT_CONTACT_NOISE,
    DEFAULT_CONTACT_VELOCITY_NOISE,
    DEFAULT_GYROSCOPE_BIAS_NOISE,
    DEFAULT_GYROSCOPE_NOISE,
    DEFAULT_JOINT_POSITION_NOISE,
    DEFAULT_ROBOT_FREQ,
    SKIP_WAITING_FOR_CONTACT,
    InekfCore,
    extract_position_quaternion,
)


class Inekf(Node):
    def __init__(self):
        super().__init__("inekf")

        # fmt: off
        self.declare_parameters(
            namespace="",
            parameters=[
                ("base_frame", "base", PD(description="Robot base frame name (for TF)")),
                ("odom_frame", "odom", PD(description="World frame name (for TF)")),
                ("robot_freq", DEFAULT_ROBOT_FREQ, PD(description="Frequency at which the robot publish its state")),
                ("gyroscope_noise", DEFAULT_GYROSCOPE_NOISE, PD(description="Inekf covariance value")),
                ("accelerometer_noise", DEFAULT_ACCELEROMETER_NOISE, PD(description="Inekf covariance value")),
                ("gyroscopeBias_noise", DEFAULT_GYROSCOPE_BIAS_NOISE, PD(description="Inekf covariance value")),
                ("accelerometerBias_noise", DEFAULT_ACCELEROMETER_BIAS_NOISE, PD(description="Inekf covariance value")),
                ("contact_noise", DEFAULT_CONTACT_NOISE, PD(description="Inekf covariance value")),
                ("joint_position_noise", DEFAULT_JOINT_POSITION_NOISE, PD(description="Noise on joint configuration measurements to project using jacobian")),
                ("contact_velocity_noise", DEFAULT_CONTACT_VELOCITY_NOISE, PD(description="Noise on contact velocity")),
            ],
        )
        # fmt: on

        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value

        self.core = InekfCore(
            robot_freq=self.get_parameter("robot_freq").get_parameter_value().double_value,
            gyroscope_noise=self.get_parameter("gyroscope_noise").value,
            accelerometer_noise=self.get_parameter("accelerometer_noise").value,
            gyroscope_bias_noise=self.get_parameter("gyroscopeBias_noise").value,
            accelerometer_bias_noise=self.get_parameter("accelerometerBias_noise").value,
            contact_noise=self.get_parameter("contact_noise").value,
            joint_position_noise=self.get_parameter("joint_position_noise").value,
            contact_velocity_noise=self.get_parameter("contact_velocity_noise").value,
        )
        self._filter_started = False

        # Subscriptions
        self.lowstate_subscription = self.create_subscription(LowState, "/lowstate", self.listener_callback, 10)
        self.get_logger().info("Using IMU from '/lowstate'")

        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", 1)
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        result = self.core.process_lowstate(msg)
        if isinstance(result, str):
            if result == SKIP_WAITING_FOR_CONTACT:
                self.get_logger().info(
                    "Waiting for one or more foot to touch the ground to start filter.", once=True
                )
            return

        if not self._filter_started:
            self._filter_started = True
            self.get_logger().info("One foot (or more) in contact with the ground: starting filter.")

        filter_state, gyro = result
        self.publish_state(filter_state, gyro)

    def publish_state(self, filter_state, twist_angular_vel):
        timestamp = self.get_clock().now().to_msg()

        position, quat = extract_position_quaternion(filter_state)
        state_rotation = filter_state.getRotation()
        velocity = (state_rotation.T @ filter_state.getX()[0:3, 3:4]).reshape(-1)

        # TF2 message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = timestamp
        transform_msg.child_frame_id = self.base_frame
        transform_msg.header.frame_id = self.odom_frame

        transform_msg.transform.translation.x = position[0]
        transform_msg.transform.translation.y = position[1]
        transform_msg.transform.translation.z = position[2]

        transform_msg.transform.rotation.x = quat[0]
        transform_msg.transform.rotation.y = quat[1]
        transform_msg.transform.rotation.z = quat[2]
        transform_msg.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(transform_msg)

        # Odometry topic
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.child_frame_id = self.base_frame
        odom_msg.header.frame_id = self.odom_frame

        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]

        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = velocity[0]
        odom_msg.twist.twist.linear.y = velocity[1]
        odom_msg.twist.twist.linear.z = velocity[2]

        odom_msg.twist.twist.angular.x = float(twist_angular_vel[0])
        odom_msg.twist.twist.angular.y = float(twist_angular_vel[1])
        odom_msg.twist.twist.angular.z = float(twist_angular_vel[2])

        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    inekf_node = Inekf()

    rclpy.spin(inekf_node)

    inekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
