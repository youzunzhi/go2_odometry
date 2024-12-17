#!/bin/env python3
'''
Gives odometry of the robot_base based on motion capture readings
Motion capture : Qualisys with RealTime protocol version 1.24
'''

import asyncio
import qtm_rt
import PyKDL
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np


class MocapOdometryNode(Node):
    def __init__(self):
        super().__init__('feet_to_odom') #? change name of node maybe ?

        self.declare_parameter("base_frame", "base")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("wanted_body","go2_head")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odometry_publisher = self.create_publisher(Odometry, 'odometry/filtered', 10)

        # Connecting to the motion capture
        asyncio.ensure_future(self.setup())
        asyncio.get_event_loop().run_forever()

    async def setup(self):
        """ Connects to the Motion Capture system and sets callback on packet received """
        connection = await qtm_rt.connect("192.168.75.2", version="1.24") # version changed to 1.24

        if connection is None:
            #! for now the node does not destroy itself when failing to connect
            return
        
        xml_string = await connection.get_parameters(parameters=["6d"])
        self.body_index = self.create_body_index(xml_string)
        
        await connection.stream_frames(components=["6d"], on_packet=self.on_packet)

    def on_packet(self, packet):
        """ Callback function called every time a data packet arrives from QTM """

        info, bodies = packet.get_6d()
        
        for body in bodies:
            # select wanted body among all rigid bodies streamed
            if self.get_parameter("wanted_body").value is not None and self.get_parameter("wanted_body").value in self.body_index:
                
                position, rotation = body
                rotation_matrix = PyKDL.Rotation(rotation[0][0], rotation[0][1], rotation[0][2],rotation[0][3], rotation[0][4], rotation[0][5],rotation[0][6],rotation[0][7],rotation[0][8])
                qx,qy,qz,qw = rotation_matrix.GetQuaternion()

                if not(np.isnan(qx)):

                    timestamp = self.get_clock().now().to_msg()

                    # Transform from odom_frame (unmoving) to base_frame (tied to robot base)
                    transform_msg = TransformStamped()
                    transform_msg.header.stamp = timestamp
                    transform_msg.child_frame_id = self.get_parameter("base_frame").value
                    transform_msg.header.frame_id = self.get_parameter("odom_frame").value

                    # translation + convert to meter
                    transform_msg.transform.translation.x = position[0]*0.001
                    transform_msg.transform.translation.y = position[1]*0.001
                    transform_msg.transform.translation.z = position[2]*0.001

                    # rotation
                    transform_msg.transform.rotation.x = qx
                    transform_msg.transform.rotation.y = qy
                    transform_msg.transform.rotation.z = qz
                    transform_msg.transform.rotation.w = qw

                    self.tf_broadcaster.sendTransform(transform_msg)
                    

                    odometry_msg = Odometry()
                    odometry_msg.header.stamp = timestamp
                    odometry_msg.child_frame_id = self.get_parameter("base_frame").value
                    odometry_msg.header.frame_id = self.get_parameter("odom_frame").value

                    # position + convert to meter
                    odometry_msg.pose.pose.position.x = position[0]*0.001
                    odometry_msg.pose.pose.position.y = position[1]*0.001
                    odometry_msg.pose.pose.position.z = position[2]*0.001 

                    #orientation
                    odometry_msg.pose.pose.orientation.x = qx
                    odometry_msg.pose.pose.orientation.y = qy
                    odometry_msg.pose.pose.orientation.z = qz
                    odometry_msg.pose.pose.orientation.w = qw 

                    # linear speed
                    odometry_msg.twist.twist.linear.x = 0. 
                    odometry_msg.twist.twist.linear.y = 0.
                    odometry_msg.twist.twist.linear.z = 0.

                    # angular speed
                    odometry_msg.twist.twist.angular.x = 0.
                    odometry_msg.twist.twist.angular.y = 0.
                    odometry_msg.twist.twist.angular.z = 0.

                    self.odometry_publisher.publish(odometry_msg)
            else :
                self.get_logger().info("Object {} was not found".format(self.get_parameter("wanted_body").value))
                
                

    def create_body_index(self,xml_string):
        """ Extract a name to index dictionary from 6dof settings xml,
            Taken from qualisys_python_sdk/examples/stream_6dof_example.py """
        
        xml = ET.fromstring(xml_string)

        for index, body in enumerate(xml.findall("*/Body/Name")):
            self.body_index[body.text.strip()] = index

        return self.body_index

    # Class attributes
    body_index = {}

def main(args=None):
    rclpy.init(args=args)

    node = MocapOdometryNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()