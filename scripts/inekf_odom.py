#!/bin/env python3

import ast
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
    SKIP_WAITING_FOR_IMU,
    InekfCore,
    extract_position_quaternion,
    unpack_imu_msg,
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
                ("imu_source", "lowstate", PD(description="IMU source used for propagation: 'lowstate' or 'utlidar'")),
                ("utlidar_imu_topic", "/utlidar/imu", PD(description="Topic used when imu_source='utlidar'")),
                (
                    "imu_rotation_rpy",
                    [0.0, 0.0, 0.0],
                    PD(description="RPY rotation (rad) from selected IMU frame to filter IMU frame"),
                ),
                (
                    "imu_translation_xyz",
                    [0.0, 0.0, 0.0],
                    PD(description="Translation (m) from selected IMU origin to filter IMU origin"),
                ),
                (
                    "compensate_imu_translation",
                    False,
                    PD(description="Apply angular-rate-based translation compensation between IMU origins"),
                ),
            ],
        )
        # fmt: on

        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value

        imu_source = self.get_parameter("imu_source").get_parameter_value().string_value
        if imu_source not in {"lowstate", "utlidar"}:
            self.get_logger().warning(f"Unknown imu_source '{imu_source}', fallback to 'lowstate'.")
            imu_source = "lowstate"

        self.core = InekfCore(
            imu_source=imu_source,
            robot_freq=self.get_parameter("robot_freq").get_parameter_value().double_value,
            gyroscope_noise=self.get_parameter("gyroscope_noise").value,
            accelerometer_noise=self.get_parameter("accelerometer_noise").value,
            gyroscope_bias_noise=self.get_parameter("gyroscopeBias_noise").value,
            accelerometer_bias_noise=self.get_parameter("accelerometerBias_noise").value,
            contact_noise=self.get_parameter("contact_noise").value,
            joint_position_noise=self.get_parameter("joint_position_noise").value,
            contact_velocity_noise=self.get_parameter("contact_velocity_noise").value,
            imu_rotation_rpy=self.get_vector3_param("imu_rotation_rpy"),
            imu_translation_xyz=self.get_vector3_param("imu_translation_xyz"),
            compensate_imu_translation=self.get_parameter("compensate_imu_translation").get_parameter_value().bool_value,
        )
        self._filter_started = False

        # Subscriptions
        self.lowstate_subscription = self.create_subscription(LowState, "/lowstate", self.listener_callback, 10)
        if imu_source == "utlidar":
            utlidar_imu_topic = self.get_parameter("utlidar_imu_topic").get_parameter_value().string_value
            self.utlidar_imu_subscription = self.create_subscription(Imu, utlidar_imu_topic, self.utlidar_callback, 10)
            self.get_logger().info(f"Using IMU from '{utlidar_imu_topic}'")
        else:
            self.get_logger().info("Using IMU from '/lowstate'")

        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", 1)
        self.tf_broadcaster = TransformBroadcaster(self)

    def get_vector3_param(self, parameter_name):
        raw_value = self.get_parameter(parameter_name).value
        if isinstance(raw_value, str):
            raw_value = ast.literal_eval(raw_value)
        vector = np.array(raw_value, dtype=float).reshape(-1)
        assert vector.shape[0] == 3, (
            f"Parameter '{parameter_name}' must contain exactly 3 values, got {vector.shape[0]}"
        )
        return vector

    def utlidar_callback(self, msg):
        gyro, acc, stamp_sec = unpack_imu_msg(msg)
        self.core.store_utlidar_imu(gyro=gyro, acc=acc, stamp_sec=stamp_sec)

    def listener_callback(self, msg):
        result = self.core.process_lowstate(msg)
        if isinstance(result, str):
            if result == SKIP_WAITING_FOR_IMU:
                self.get_logger().info(
                    "Waiting for first /utlidar/imu message before propagating filter.", once=True
                )
            elif result == SKIP_WAITING_FOR_CONTACT:
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
