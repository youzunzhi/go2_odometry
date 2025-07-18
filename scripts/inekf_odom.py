#!/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
import pinocchio as pin

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from inekf import RobotState, NoiseParams, InEKF, Kinematics
from go2_description.loader import loadGo2


# ==============================================================================
# Main Class
# ==============================================================================
class Inekf(Node):
    def __init__(self):
        super().__init__("inekf")
        print("==GO2 InEKF launched==")
        # ROS2 =================================================================
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("odom_frame", "odom")
        robot_fq = self.declare_parameter(
            "robot_freq", 500.0, ParameterDescriptor(description="Fq at which the robot publish its state")
        ).value
        self.dt = 1.0 / robot_fq

        robot = loadGo2()
        self.rmodel = robot.model
        self.rdata = self.rmodel.createData()
        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.rmodel.getFrameId(frame_name) for frame_name in self.foot_frame_name]
        self.imu_frame_id = self.rmodel.getFrameId("imu")

        self.lowstate_subscription = self.create_subscription(LowState, "/lowstate", self.listener_callback, 10)

        qos_profile_keeplast = QoSProfile(history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", qos_profile_keeplast)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform_msg = TransformStamped()
        self.odom_msg = Odometry()

        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_ids = [0, 1, 2, 3]

        # Data from go2 ========================================================
        self.imu_measurement = np.zeros(6)
        self.imu_measurement_prev = np.zeros(6)
        self.quaternion_measurement = np.zeros(4)
        self.feet_contacts = np.zeros(4)
        self.contacts = {0: False, 1: False, 2: False, 3: False}

        self.joints_unitree_2_urdf = [1, 0, 3, 2]  # index = urdf convention, value = unitree joint numbering

        self.estimated_contact_positions = {
            0: 0,  #! not sure if correct
            1: 0,
            2: 0,
            3: 0,
        }

        # double = float in python
        self.t: float = 0
        self.t_prev: float = 0

        # Invariant EKF
        initial_state = RobotState()

        R0 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # Initial rotation

        v0 = np.zeros(3)  # initial velocity
        p0 = np.array([0, 0, 0.07])  # initial position in simulation
        bg0 = np.zeros(3)  # initial gyroscope bias
        ba0 = np.zeros(3)  # initial accelerometer bias
        gravity = np.array([0, 0, -9.81])

        initial_state.setRotation(R0)
        initial_state.setVelocity(v0)
        initial_state.setPosition(p0)
        initial_state.setGyroscopeBias(bg0)
        initial_state.setAccelerometerBias(ba0)

        # Initialize state covariance
        noise_params = NoiseParams()
        noise_params.setGyroscopeNoise(0.01)
        noise_params.setAccelerometerNoise(0.1)
        noise_params.setGyroscopeBiasNoise(0.00001)
        noise_params.setAccelerometerBiasNoise(0.0001)
        noise_params.setContactNoise(0.001)

        self.filter = InEKF(initial_state, noise_params)
        self.filter.setGravity(gravity)

    def listener_callback(self, msg):
        # IMU measurement - used for propagation ===============================
        #! gyroscope meas in filter are radians
        self.imu_measurement[0] = msg.imu_state.gyroscope[0]  # x
        self.imu_measurement[1] = msg.imu_state.gyroscope[1]  # y
        self.imu_measurement[2] = msg.imu_state.gyroscope[2]  # z

        self.imu_measurement[3] = msg.imu_state.accelerometer[0]  # x
        self.imu_measurement[4] = msg.imu_state.accelerometer[1]  # y
        self.imu_measurement[5] = msg.imu_state.accelerometer[2]  # z

        self.propagate()

        # KINEMATIC data =======================================================
        self.quaternion_measurement[0] = msg.imu_state.quaternion[1]  # x
        self.quaternion_measurement[1] = msg.imu_state.quaternion[2]  # y
        self.quaternion_measurement[2] = msg.imu_state.quaternion[3]  # z
        self.quaternion_measurement[3] = msg.imu_state.quaternion[0]  # w
        # TODO normalize quaternion && find quat type

        # TODO: add feet vel and feet pos (additionnal info on base)

        # FEET KINEMATIC data ==================================================
        contact_list, pose_list, covariance_list = self.feet_transformations(msg)

        contact_pairs = []
        kinematics_list = []
        for i in range(len(self.foot_frame_name)):
            contact_pairs.append((i, contact_list[i]))

            velocity = np.zeros(3)

            kinematics = Kinematics(i, pose_list[i].translation, covariance_list[i], velocity, np.eye(3) * 0.001)
            kinematics_list.append(kinematics)

        self.filter.setContacts(contact_pairs)
        self.filter.correctKinematics(kinematics_list)

    def _unitree_to_urdf_vec(self, vec):
        # fmt: off
        return  [vec[3],  vec[4],  vec[5],
                 vec[0],  vec[1],  vec[2],
                 vec[9],  vec[10], vec[11],
                 vec[6],  vec[7],  vec[8],]
        # fmt: on

    def feet_transformations(self, state_msg):
        # Get sensor measurement
        q_unitree = [j.q for j in state_msg.motor_state[:12]]
        v_unitree = [j.dq for j in state_msg.motor_state[:12]]
        f_unitree = state_msg.foot_force

        # Rearrange joints according to urdf
        q_pin = np.array([0] * 6 + [1] + self._unitree_to_urdf_vec(q_unitree))
        v_pin = np.array([0] * 6 + self._unitree_to_urdf_vec(v_unitree))
        f_pin = [f_unitree[i] for i in [1, 0, 3, 2]]

        # Compute positions and velocities
        pin.forwardKinematics(self.rmodel, self.rdata, q_pin, v_pin)
        pin.updateFramePlacements(self.rmodel, self.rdata)
        pin.computeJointJacobians(self.rmodel, self.rdata)

        # Make message
        contact_list = []
        pose_list = []
        covariance_list = []
        for i in range(4):
            if f_pin[i] >= 20:
                contact_list.append(True)
            else:
                contact_list.append(False)

            pose_list.append(self.rdata.oMf[self.foot_frame_id[i]])

            Jc = pin.getFrameJacobian(self.rmodel, self.rdata, self.foot_frame_id[i], pin.LOCAL)[:3, 6:]
            cov_pose = Jc @ np.eye(12) * 1e-3 @ Jc.transpose()
            covariance_list.append(cov_pose)

        return contact_list, pose_list, covariance_list

    def propagate(self):
        # Transform from odom_frame (unmoving) to base_frame (tied to robot base)
        timestamp = self.get_clock().now().to_msg()
        self.transform_msg.header.stamp = timestamp
        self.transform_msg.child_frame_id = "base"  # self.get_parameter("base_frame").value
        self.transform_msg.header.frame_id = "odom"  # self.get_parameter("odom_frame").value

        self.odom_msg.header.stamp = timestamp
        self.odom_msg.child_frame_id = "base"  # self.get_parameter("base_frame").value
        self.odom_msg.header.frame_id = "odom"  # self.get_parameter("odom_frame").value

        self.filter.propagate(self.imu_measurement, self.dt)

        new_state = self.filter.getState()
        new_r = new_state.getRotation()
        new_p = new_state.getPosition()
        new_v = new_r.T @ new_state.getX()[0:3, 3:4]
        new_v = new_v.reshape(-1)
        # self.get_logger().info('imu measure ' + str(self.imu_measurement))
        # self.get_logger().info('Position ' + str(new_p))

        quat = pin.Quaternion(new_r)
        quat.normalize()

        # ROS2 transform
        self.transform_msg.transform.translation.x = new_p[0]
        self.transform_msg.transform.translation.y = new_p[1]
        self.transform_msg.transform.translation.z = new_p[2]

        self.odom_msg.pose.pose.position.x = new_p[0]
        self.odom_msg.pose.pose.position.y = new_p[1]
        self.odom_msg.pose.pose.position.z = new_p[2]

        self.transform_msg.transform.rotation.x = quat.x
        self.transform_msg.transform.rotation.y = quat.y
        self.transform_msg.transform.rotation.z = quat.z
        self.transform_msg.transform.rotation.w = quat.w

        self.odom_msg.pose.pose.orientation.x = quat.x
        self.odom_msg.pose.pose.orientation.y = quat.y
        self.odom_msg.pose.pose.orientation.z = quat.z
        self.odom_msg.pose.pose.orientation.w = quat.w

        self.odom_msg.twist.twist.linear.x = new_v[0]
        self.odom_msg.twist.twist.linear.y = new_v[1]
        self.odom_msg.twist.twist.linear.z = new_v[2]

        self.odom_msg.twist.twist.angular.x = self.imu_measurement[0]
        self.odom_msg.twist.twist.angular.y = self.imu_measurement[1]
        self.odom_msg.twist.twist.angular.z = self.imu_measurement[2]

        self.tf_broadcaster.sendTransform(self.transform_msg)
        self.odom_publisher.publish(self.odom_msg)

    # Attributes ===============================================================

    estimated_landmarks = {}  # int:int
    contacts = {}  # int:bool
    estimated_contact_positions = {}  # int:int


def main(args=None):
    rclpy.init(args=args)

    inekf_node = Inekf()

    rclpy.spin(inekf_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
