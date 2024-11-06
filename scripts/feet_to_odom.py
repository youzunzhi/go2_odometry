#!/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from unitree_go.msg import LowState

import numpy as np
import pinocchio as pin
from example_robot_data.robots_loader import Go2Loader

class FeetToOdom(Node):

    def __init__(self):
        super().__init__('feet_to_odom')
        self.publisher_ = self.create_publisher(Odometry, 'feet_odom', 10)
        self.subscription_ = self.create_subscription(
            LowState,
            '/lowstate',
            self.listener_callback,
            10)

        self.robot = Go2Loader().robot
        self.foot_frame_id = [self.robot.model.getFrameId(prefix + "_foot") for prefix in ["FL", "FR", "RL", "RR"]]
        self.imu_frame_id = self.robot.model.getFrameId("imu")

        self.prefilled_msg = Odometry()
        self.prefilled_msg.header.frame_id = "odom"
        self.prefilled_msg.child_frame_id = "utlidar_imu"
        self.prefilled_msg.pose.covariance = [0]*36
        self.prefilled_msg.twist.covariance = [0]*36

        self.prefilled_msg.pose.covariance[2 * (6+1)] = 0.005**2
        self.prefilled_msg.twist.covariance[7 * (6+1)] = 0.1**2
        self.prefilled_msg.twist.covariance[8 * (6+1)] = 0.1**2
        self.prefilled_msg.twist.covariance[9 * (6+1)] = 0.1**2


    def listener_callback(self, state_msg):
        q = np.array([0]*6 + [1] + [j.q for j in state_msg.motor_state[:12]])
        v = np.array([0]*6 + [j.dq for j in state_msg.motor_state[:12]])
        self.robot.framesForwardKinematics(q, v)
        oMf_list = [self.robot.data.oMf[id] for id in self.foot_frame_id]
        oMi = self.robot.data.oMf[self.imu_frame_id]
        v_list = [pin.getFrameVelocity(self.model, self.data, id, pin.world) for id in self.foot_frame_id]
        ovi_o = pin.getFrameVelocity(self.model, self.data, self.imu_frame_id, pin.world)

        oMf = oMf_list[0]
        ovf_o = v_list[0]

        fMi = oMf.actInv(oMi) # TODO : It's not what we want !!
        ovi_f = ovi_o - ovf_o# TODO : It's not what we want !!
        fvi_f = oMf.actInv(ovi_f)
        odom_msg = self.prefilled_msg
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = fMi.translation
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z = fvi_f.translation
        odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z = fvi_f.rotation
        self.publisher_.publish(odom_msg)

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