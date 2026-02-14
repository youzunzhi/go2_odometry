#!/bin/env python3

import ast
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState
import pinocchio as pin

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor as PD
from inekf import RobotState, NoiseParams, InEKF, Kinematics
from go2_description.loader import loadGo2


# ==============================================================================
# Main Class
# ==============================================================================
class Inekf(Node):
    def __init__(self):
        super().__init__("inekf")

        # Ros params
        # fmt: off
        self.declare_parameters(
            namespace="",
            parameters=[
                ("base_frame", "base", PD(description="Robot base frame name (for TF)")),
                ("odom_frame", "odom", PD(description="World frame name (for TF)")),
                ("robot_freq", 500.0, PD(description="Frequency at which the robot publish its state")),
                ("gyroscope_noise", 0.01, PD(description="Inekf covariance value")),
                ("accelerometer_noise", 0.1, PD(description="Inekf covariance value")),
                ("gyroscopeBias_noise", 0.00001, PD(description="Inekf covariance value")),
                ("accelerometerBias_noise", 0.0001, PD(description="Inekf covariance value")),
                ("contact_noise", 0.001, PD(description="Inekf covariance value")),
                ("joint_position_noise", 0.001, PD(description="Noise on joint configuration measurements to project using jacobian")),
                ("contact_velocity_noise", 0.001, PD(description="Noise on contact velocity")),
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
        self.dt = 1.0 / self.get_parameter("robot_freq").get_parameter_value().double_value
        self.pause = True  # By default filter is paused and wait for the first feet contact to start
        self.imu_source = self.get_parameter("imu_source").get_parameter_value().string_value
        if self.imu_source not in {"lowstate", "utlidar"}:
            self.get_logger().warning(f"Unknown imu_source '{self.imu_source}', fallback to 'lowstate'.")
            self.imu_source = "lowstate"

        imu_rotation_rpy = self.get_vector3_param("imu_rotation_rpy")
        imu_translation_xyz = self.get_vector3_param("imu_translation_xyz")
        self.imu_rotation = pin.rpy.rpyToMatrix(imu_rotation_rpy)
        self.imu_translation = imu_translation_xyz
        self.compensate_imu_translation = (
            self.get_parameter("compensate_imu_translation").get_parameter_value().bool_value
        )
        self.prev_gyro_in_filter_imu = None
        self.prev_imu_stamp_sec = None
        self.latest_utlidar_gyro = None
        self.latest_utlidar_acc = None
        self.latest_utlidar_stamp_sec = None

        # Load robot model
        self.robot = loadGo2()
        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.robot.model.getFrameId(frame_name) for frame_name in self.foot_frame_name]
        self.imu_frame_id = self.robot.model.getFrameId("imu")

        # In/Out topics
        self.lowstate_subscription = self.create_subscription(LowState, "/lowstate", self.listener_callback, 10)
        if self.imu_source == "utlidar":
            utlidar_imu_topic = self.get_parameter("utlidar_imu_topic").get_parameter_value().string_value
            self.utlidar_imu_subscription = self.create_subscription(Imu, utlidar_imu_topic, self.utlidar_callback, 10)
            self.get_logger().info(f"Using IMU from '{utlidar_imu_topic}'")
        else:
            self.get_logger().info("Using IMU from '/lowstate'")
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Invariant EKF
        gravity = np.array([0, 0, -9.81])

        initial_state = RobotState()
        initial_state.setRotation(np.eye(3))
        initial_state.setVelocity(np.zeros(3))
        initial_state.setPosition(np.zeros(3))
        initial_state.setGyroscopeBias(np.zeros(3))
        initial_state.setAccelerometerBias(np.zeros(3))

        # Initialize state covariance
        noise_params = NoiseParams()
        noise_params.setGyroscopeNoise(self.get_parameter("gyroscope_noise").value)
        noise_params.setAccelerometerNoise(self.get_parameter("accelerometer_noise").value)
        noise_params.setGyroscopeBiasNoise(self.get_parameter("gyroscopeBias_noise").value)
        noise_params.setAccelerometerBiasNoise(self.get_parameter("accelerometerBias_noise").value)
        noise_params.setContactNoise(self.get_parameter("contact_noise").value)

        self.joint_pos_noise = self.get_parameter("joint_position_noise").value
        self.contact_vel_noise = self.get_parameter("contact_velocity_noise").value

        self.filter = InEKF(initial_state, noise_params)
        self.filter.setGravity(gravity)

    @staticmethod
    def stamp_to_sec(stamp):
        if stamp is None:
            return None
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def get_vector3_param(self, parameter_name):
        raw_value = self.get_parameter(parameter_name).value
        if isinstance(raw_value, str):
            try:
                raw_value = ast.literal_eval(raw_value)
            except (ValueError, SyntaxError):
                self.get_logger().warning(f"{parameter_name} must be a 3D vector. Fallback to [0, 0, 0].")
                return np.zeros(3)

        try:
            vector = np.array(raw_value, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            self.get_logger().warning(f"{parameter_name} must be a 3D vector. Fallback to [0, 0, 0].")
            return np.zeros(3)

        if vector.shape[0] != 3:
            self.get_logger().warning(f"{parameter_name} must contain 3 values. Fallback to [0, 0, 0].")
            return np.zeros(3)
        return vector

    def utlidar_callback(self, msg):
        self.latest_utlidar_gyro = np.array(
            [
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ]
        )
        self.latest_utlidar_acc = np.array(
            [
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            ]
        )
        self.latest_utlidar_stamp_sec = self.stamp_to_sec(msg.header.stamp)

    def get_imu_measurement(self, lowstate_msg):
        if self.imu_source == "utlidar":
            if self.latest_utlidar_gyro is None or self.latest_utlidar_acc is None:
                self.get_logger().info("Waiting for first /utlidar/imu message before propagating filter.", once=True)
                return None, None
            gyro = self.latest_utlidar_gyro.copy()
            acc = self.latest_utlidar_acc.copy()
            stamp_sec = self.latest_utlidar_stamp_sec
        else:
            gyro = np.array(lowstate_msg.imu_state.gyroscope, dtype=float)
            acc = np.array(lowstate_msg.imu_state.accelerometer, dtype=float)
            stamp_sec = None

        gyro, acc = self.compensate_imu_transform(gyro, acc, stamp_sec)
        imu_state = np.concatenate([gyro, acc])
        return imu_state, gyro

    def compensate_imu_transform(self, gyro, acc, stamp_sec):
        gyro_in_filter_imu = self.imu_rotation @ gyro
        acc_in_filter_imu = self.imu_rotation @ acc

        if self.compensate_imu_translation:
            gyro_dot = np.zeros(3)
            if self.prev_gyro_in_filter_imu is not None:
                dt = self.dt
                if stamp_sec is not None and self.prev_imu_stamp_sec is not None:
                    dt = max(stamp_sec - self.prev_imu_stamp_sec, 1e-6)
                gyro_dot = (gyro_in_filter_imu - self.prev_gyro_in_filter_imu) / max(dt, 1e-6)

            translation_effect = np.cross(gyro_dot, self.imu_translation) + np.cross(
                gyro_in_filter_imu, np.cross(gyro_in_filter_imu, self.imu_translation)
            )
            acc_in_filter_imu = acc_in_filter_imu + translation_effect

        self.prev_gyro_in_filter_imu = gyro_in_filter_imu.copy()
        if stamp_sec is not None:
            self.prev_imu_stamp_sec = stamp_sec
        return gyro_in_filter_imu, acc_in_filter_imu

    def listener_callback(self, msg):
        # Format IMU measurements
        imu_state, gyro = self.get_imu_measurement(msg)
        if imu_state is None:
            return

        # Feet kinematic data
        contact_list, pose_list, normed_covariance_list = self.feet_transformations(msg)

        if self.pause:
            if any(contact_list):
                self.pause = False
                self.get_logger().info("One foot (or more) in contact with the ground: starting filter.")
            else:
                self.get_logger().info("Waiting for one or more foot to touch the ground to start filter.", once=True)
                return  # Skip the rest of the filter

        # Propagation step: using IMU
        self.filter.propagate(imu_state, self.dt)

        # TODO: use IMU quaternion for extra correction step ?

        # Correction step: using feet kinematics
        contact_pairs = []
        kinematics_list = []
        for i in range(len(self.foot_frame_name)):
            contact_pairs.append((i, contact_list[i]))

            velocity = np.zeros(3)

            kinematics = Kinematics(
                i,
                pose_list[i].translation,
                self.joint_pos_noise * normed_covariance_list[i],
                velocity,
                self.contact_vel_noise * np.eye(3),
            )
            kinematics_list.append(kinematics)

        self.filter.setContacts(contact_pairs)
        self.filter.correctKinematics(kinematics_list)

        self.publish_state(self.filter.getState(), gyro)

    def feet_transformations(self, state_msg):
        def unitree_to_urdf_vec(vec):
            # fmt: off
            return  [vec[3],  vec[4],  vec[5],
                     vec[0],  vec[1],  vec[2],
                     vec[9],  vec[10], vec[11],
                     vec[6],  vec[7],  vec[8],]
            # fmt: on

        def feet_contacts(feet_forces):
            CONTACT_THRESHOLD = 22
            ret = [bool(f >= CONTACT_THRESHOLD) for f in feet_forces]
            return ret

        # Get sensor measurement
        q_unitree = [j.q for j in state_msg.motor_state[:12]]
        v_unitree = [j.dq for j in state_msg.motor_state[:12]]
        f_unitree = state_msg.foot_force

        # Rearrange joints according to urdf
        q_pin = np.array([0] * 6 + [1] + unitree_to_urdf_vec(q_unitree))
        v_pin = np.array([0] * 6 + unitree_to_urdf_vec(v_unitree))
        f_pin = [f_unitree[i] for i in [1, 0, 3, 2]]

        # Compute positions and velocities
        pin.forwardKinematics(self.robot.model, self.robot.data, q_pin, v_pin)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        pin.computeJointJacobians(self.robot.model, self.robot.data)

        # Make message
        contact_list = feet_contacts(f_pin)
        pose_list = []
        normed_covariance_list = []
        for i in range(4):
            pose_list.append(self.robot.data.oMf[self.foot_frame_id[i]])

            Jc = pin.getFrameJacobian(self.robot.model, self.robot.data, self.foot_frame_id[i], pin.LOCAL)[:3, 6:]
            normed_cov_pose = Jc @ Jc.transpose()
            normed_covariance_list.append(normed_cov_pose)

        return contact_list, pose_list, normed_covariance_list

    def publish_state(self, filter_state, twist_angular_vel):
        # Get filter state
        timestamp = self.get_clock().now().to_msg()

        state_rotation = filter_state.getRotation()
        state_position = filter_state.getPosition()
        state_velocity = state_rotation.T @ filter_state.getX()[0:3, 3:4]
        state_velocity = state_velocity.reshape(-1)

        state_quaternion = pin.Quaternion(state_rotation)
        state_quaternion.normalize()

        # TF2 messages
        transform_msg = TransformStamped()
        transform_msg.header.stamp = timestamp
        transform_msg.child_frame_id = self.base_frame
        transform_msg.header.frame_id = self.odom_frame

        transform_msg.transform.translation.x = state_position[0]
        transform_msg.transform.translation.y = state_position[1]
        transform_msg.transform.translation.z = state_position[2]

        transform_msg.transform.rotation.x = state_quaternion.x
        transform_msg.transform.rotation.y = state_quaternion.y
        transform_msg.transform.rotation.z = state_quaternion.z
        transform_msg.transform.rotation.w = state_quaternion.w

        self.tf_broadcaster.sendTransform(transform_msg)

        # Odometry topic
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.child_frame_id = self.base_frame
        odom_msg.header.frame_id = self.odom_frame

        odom_msg.pose.pose.position.x = state_position[0]
        odom_msg.pose.pose.position.y = state_position[1]
        odom_msg.pose.pose.position.z = state_position[2]

        odom_msg.pose.pose.orientation.x = state_quaternion.x
        odom_msg.pose.pose.orientation.y = state_quaternion.y
        odom_msg.pose.pose.orientation.z = state_quaternion.z
        odom_msg.pose.pose.orientation.w = state_quaternion.w

        odom_msg.twist.twist.linear.x = state_velocity[0]
        odom_msg.twist.twist.linear.y = state_velocity[1]
        odom_msg.twist.twist.linear.z = state_velocity[2]

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
