#!/usr/bin/env python3
"""Shared InEKF odometry core: filter initialization, IMU handling,
feet kinematics, and the propagate+correct cycle.

Used by scripts/inekf_odom.py (online ROS node) and
debug_scripts/compare_bag_inekf_odometry.py (offline bag replay).
"""

import sys
from pathlib import Path
from typing import Tuple, Union

import numpy as np
import pinocchio as pin

GO2_DESCRIPTION_SITE_PACKAGES = Path(
    "/home/unitree/go2_description/install/go2_description/lib/python3.8/site-packages"
)
assert GO2_DESCRIPTION_SITE_PACKAGES.is_dir(), (
    f"Expected go2_description install at {GO2_DESCRIPTION_SITE_PACKAGES}"
)
sys.path.append(str(GO2_DESCRIPTION_SITE_PACKAGES))
from go2_description.loader import loadGo2
from inekf import InEKF, Kinematics, NoiseParams, RobotState

CONTACT_THRESHOLD = 22
NUM_FEET = 4

# Unitree SDK joint order -> URDF joint order
_URDF_FROM_SDK = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

# Unitree SDK foot order -> pinocchio foot order (FL, FR, RL, RR)
_PIN_FROM_SDK_FOOT = [1, 0, 3, 2]

# Noise parameter defaults (shared by inekf_odom.py and compare_bag_inekf_odometry.py)
DEFAULT_ROBOT_FREQ = 500.0
DEFAULT_GYROSCOPE_NOISE = 0.01
DEFAULT_ACCELEROMETER_NOISE = 0.1
DEFAULT_GYROSCOPE_BIAS_NOISE = 0.00001
DEFAULT_ACCELEROMETER_BIAS_NOISE = 0.0001
DEFAULT_CONTACT_NOISE = 0.001
DEFAULT_JOINT_POSITION_NOISE = 0.001
DEFAULT_CONTACT_VELOCITY_NOISE = 0.001

def extract_position_quaternion(filter_state) -> Tuple[np.ndarray, np.ndarray]:
    """Extract (position(3,), quaternion_xyzw(4,)) from an InEKF RobotState."""
    position = filter_state.getPosition().reshape(3).copy()
    q = pin.Quaternion(filter_state.getRotation())
    q.normalize()
    quaternion_xyzw = np.array([q.x, q.y, q.z, q.w])
    return position, quaternion_xyzw


SKIP_WAITING_FOR_CONTACT = "waiting_for_contact"


class InekfCore:
    """
    Core InEKF filter: lowstate IMU handling, feet forward kinematics,
    contact detection, and the InEKF propagate+correct cycle.
    """

    def __init__(
        self,
        robot_freq: float = DEFAULT_ROBOT_FREQ,
        gyroscope_noise: float = DEFAULT_GYROSCOPE_NOISE,
        accelerometer_noise: float = DEFAULT_ACCELEROMETER_NOISE,
        gyroscope_bias_noise: float = DEFAULT_GYROSCOPE_BIAS_NOISE,
        accelerometer_bias_noise: float = DEFAULT_ACCELEROMETER_BIAS_NOISE,
        contact_noise: float = DEFAULT_CONTACT_NOISE,
        joint_position_noise: float = DEFAULT_JOINT_POSITION_NOISE,
        contact_velocity_noise: float = DEFAULT_CONTACT_VELOCITY_NOISE,
    ):
        self.dt = 1.0 / float(robot_freq)
        self.pause = True

        self.robot = loadGo2()
        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.robot.model.getFrameId(name) for name in self.foot_frame_name]

        gravity = np.array([0.0, 0.0, -9.81])
        initial_state = RobotState()
        initial_state.setRotation(np.eye(3))
        initial_state.setVelocity(np.zeros(3))
        initial_state.setPosition(np.zeros(3))
        initial_state.setGyroscopeBias(np.zeros(3))
        initial_state.setAccelerometerBias(np.zeros(3))

        noise_params = NoiseParams()
        noise_params.setGyroscopeNoise(gyroscope_noise)
        noise_params.setAccelerometerNoise(accelerometer_noise)
        noise_params.setGyroscopeBiasNoise(gyroscope_bias_noise)
        noise_params.setAccelerometerBiasNoise(accelerometer_bias_noise)
        noise_params.setContactNoise(contact_noise)

        self.joint_pos_noise = joint_position_noise
        self.contact_vel_noise = contact_velocity_noise

        self.filter = InEKF(initial_state, noise_params)
        self.filter.setGravity(gravity)

    def get_imu_measurement(self, lowstate_msg) -> Tuple[np.ndarray, np.ndarray]:
        """Get IMU measurement from lowstate. Returns (imu_6d, gyro)."""
        gyro = np.array(lowstate_msg.imu_state.gyroscope, dtype=float)
        acc = np.array(lowstate_msg.imu_state.accelerometer, dtype=float)
        imu_state = np.array(np.concatenate([gyro, acc]), dtype=float)
        return imu_state, gyro

    def feet_transformations(self, state_msg):
        """Compute foot contacts, SE3 poses, and Jacobian-based covariances from joint state."""
        q_unitree = [float(j.q) for j in state_msg.motor_state[:12]]
        v_unitree = [float(j.dq) for j in state_msg.motor_state[:12]]
        f_unitree = state_msg.foot_force

        q_pin = np.array([0] * 6 + [1] + [q_unitree[i] for i in _URDF_FROM_SDK])
        v_pin = np.array([0] * 6 + [v_unitree[i] for i in _URDF_FROM_SDK])
        f_pin = [f_unitree[i] for i in _PIN_FROM_SDK_FOOT]

        pin.forwardKinematics(self.robot.model, self.robot.data, q_pin, v_pin)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        pin.computeJointJacobians(self.robot.model, self.robot.data)

        contact_list = [bool(f >= CONTACT_THRESHOLD) for f in f_pin]
        pose_list = []
        normed_covariance_list = []
        for i in range(NUM_FEET):
            pose_list.append(self.robot.data.oMf[self.foot_frame_id[i]])
            jc = pin.getFrameJacobian(
                self.robot.model, self.robot.data, self.foot_frame_id[i], pin.LOCAL
            )[:3, 6:]
            normed_covariance_list.append(jc @ jc.transpose())

        return contact_list, pose_list, normed_covariance_list

    def process_lowstate(self, msg) -> Union[Tuple[RobotState, np.ndarray], str]:
        """
        Run the full InEKF propagate+correct cycle on a LowState message.

        Returns (filter_state, gyro) after the correction step,
        or a SKIP_* string if the filter cannot produce output yet.
        """
        imu_state, gyro = self.get_imu_measurement(msg)

        contact_list, pose_list, normed_covariance_list = self.feet_transformations(msg)
        if self.pause:
            if any(contact_list):
                self.pause = False
            else:
                return SKIP_WAITING_FOR_CONTACT

        self.filter.propagate(imu_state, self.dt)

        contact_pairs = []
        kinematics_list = []
        for i in range(NUM_FEET):
            contact_pairs.append((i, contact_list[i]))
            kinematics = Kinematics(
                i,
                pose_list[i].translation,
                self.joint_pos_noise * normed_covariance_list[i],
                np.zeros(3),
                self.contact_vel_noise * np.eye(3),
            )
            kinematics_list.append(kinematics)

        self.filter.setContacts(contact_pairs)
        self.filter.correctKinematics(kinematics_list)
        return self.filter.getState(), gyro
