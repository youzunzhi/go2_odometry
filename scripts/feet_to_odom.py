#!/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from unitree_go.msg import LowState

import numpy as np
import pinocchio as pin
from go2_description import loadGo2

class FeetToOdom(Node):

    def __init__(self):
        super().__init__('feet_to_odom')

        self.vel_publisher_ = self.create_publisher(Odometry, 'odometry/feet_vel', 10)
        self.pos_publisher_ = self.create_publisher(Odometry, 'odometry/feet_pos', 10)
        self.subscription_ = self.create_subscription(
            LowState,
            '/lowstate',
            self.listener_callback,
            10)

        self.robot = loadGo2()
        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.robot.model.getFrameId(frame_name) for frame_name in self.foot_frame_name]
        self.imu_frame_id = self.robot.model.getFrameId("imu")

        self.prefilled_vel_msg = Odometry()
        self.prefilled_vel_msg.pose.covariance = [0.]*36
        self.prefilled_vel_msg.twist.covariance = [0.]*36

        self.prefilled_vel_msg.twist.covariance[0 * (6+1)] = 1**2
        self.prefilled_vel_msg.twist.covariance[1 * (6+1)] = 1**2
        self.prefilled_vel_msg.twist.covariance[2 * (6+1)] = 1**2

        self.prefilled_pos_msg = Odometry()
        self.prefilled_pos_msg.header.frame_id = "base"
        self.prefilled_pos_msg.pose.covariance = [0.]*36
        self.prefilled_pos_msg.twist.covariance = [0.]*36
        self.prefilled_pos_msg.pose.covariance[2 * (6+1)] = 1**2

    def _unitree_to_urdf_vec(self, vec):
        return  [vec[3],  vec[4],  vec[5],
                 vec[0],  vec[1],  vec[2],
                 vec[9],  vec[10], vec[11],
                 vec[6],  vec[7],  vec[8],]


    def listener_callback(self, state_msg):
        # Get sensor measurement
        q_unitree = [j.q for j in state_msg.motor_state[:12]]
        v_unitree = [j.dq for j in state_msg.motor_state[:12]]
        fc_unitree = state_msg.foot_force

        # Rearrange joints according to urdf
        q = np.array([0]*6 + [1] + self._unitree_to_urdf_vec(q_unitree))
        v = np.array([0]*6 + self._unitree_to_urdf_vec(v_unitree))
        f_contact = [fc_unitree[i] for i in [1, 0, 3, 2]]

        # Compute positions and velocities
        ## f = foot, i = imu, b = base
        self.robot.forwardKinematics(q, v)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        bMf_list = [self.robot.data.oMf[id] for id in self.foot_frame_id]
        Vf_list = [pin.getFrameVelocity(self.robot.model, self.robot.data, id, pin.LOCAL) for id in self.foot_frame_id]

        # Make message
        odom_msg = self.prefilled_vel_msg
        pos_msg = self.prefilled_pos_msg
        pos_msg.header.stamp = odom_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(4):
            if(f_contact[i]<20):
                continue # Feet in the air : skip
            vb = -Vf_list[i]
            bMf = bMf_list[i]

            odom_msg.header.frame_id = self.foot_frame_name[i]
            odom_msg.child_frame_id = self.foot_frame_name[i]
            odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z = vb.linear
            self.vel_publisher_.publish(odom_msg)

            pos_msg.header.frame_id = "odom"
            pos_msg.child_frame_id = "base"
            pos_msg.pose.pose.position.z = -bMf.translation[2]
            self.pos_publisher_.publish(pos_msg)

def main(args=None):
    rclpy.init(args=args)

    feet_to_odom = FeetToOdom()

    rclpy.spin(feet_to_odom)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    feet_to_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()