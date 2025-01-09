#!/bin/env python3
'''
Gives odometry of the robot_base based on motion capture readings
Motion capture : Qualisys with RealTime protocol version 1.24
'''

import asyncio
import qtm_rt
from qtm_rt.packet import QRTComponentType
import xml.etree.ElementTree as ET
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class MocapOdometryNode(Node):
    def __init__(self):
        super().__init__('feet_to_odom') 

        self.declare_parameter("base_frame", "base")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("wanted_body","cube") # go2, cube or None
        self.declare_parameter("qualisys_ip","192.168.75.2")
        self.declare_parameter("publishing_freq",110)     # in Hz : due to the discretisation the frequency may be slightly lower than what is it set to. Max limit of 300Hz (set in MoCap software)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odometry_publisher = self.create_publisher(Odometry, 'odometry/filtered', 10)

        # Connecting to the motion capture
        asyncio.ensure_future(self.setup())
        self.loop = asyncio.get_event_loop()
        self.loop.run_forever()
        

    async def setup(self):
        """ Connects to the Motion Capture system and sets callback on packet received """
        connection = await qtm_rt.connect(self.get_parameter("qualisys_ip").value, version="1.24") # version changed to 1.24

        if connection is None:
            self.get_logger().error("Could not connect to the Motion Capture")
            #! for now the node does not destroy itself when failing to connect
            self.destroy_node() #! to check
            return
        
        xml_string = await connection.get_parameters(parameters=['6d'])
        self.body_index = self.create_body_index(xml_string)

        # disconnect if the wanted body isn't streamed 
        if (self.get_parameter("wanted_body").value is None) or (self.get_parameter("wanted_body").value not in self.body_index):
            self.get_logger().error("Wanted body not found or wanted_body is None")
            connection.disconnect()

        await connection.stream_frames(components=["6d"], on_packet=self.on_packet)

    def on_packet(self, packet):
        """ Callback function called every time a data packet arrives from QTM """

        # Time management
        self.new_timestamp = packet.timestamp*0.000001

        if (self.new_timestamp - self.prec_timestamp) >= (1/self.get_parameter("publishing_freq").value):

            info, bodies = packet.get_6d()

            i = 0 #index
            for body in bodies:
                # select wanted body 
                if (self.body_index[self.get_parameter("wanted_body").value] == i):

                    position, rotation = body

                    # Quaternions from rotation matrix
                    qx,qy,qz,qw = self.quaternions_from_rot(rotation)

                    timestamp = self.get_clock().now().to_msg()
                
                    # Transform from odom_frame (unmoving) to base_frame (tied to robot base)
                    self.transform_msg.header.stamp = timestamp
                    self.transform_msg.child_frame_id = self.get_parameter("base_frame").value
                    self.transform_msg.header.frame_id = self.get_parameter("odom_frame").value

                    # translation + convert to meter
                    self.transform_msg.transform.translation.x = position[0]*0.001
                    self.transform_msg.transform.translation.y = position[1]*0.001
                    self.transform_msg.transform.translation.z = position[2]*0.001

                    # rotation
                    self.transform_msg.transform.rotation.x = qx
                    self.transform_msg.transform.rotation.y = qy
                    self.transform_msg.transform.rotation.z = qz
                    self.transform_msg.transform.rotation.w = qw


                    self.odometry_msg.header.stamp = timestamp
                    self.odometry_msg.child_frame_id = self.get_parameter("base_frame").value
                    self.odometry_msg.header.frame_id = self.get_parameter("odom_frame").value

                    # position + convert to meter
                    self.odometry_msg.pose.pose.position.x = position[0]*0.001
                    self.odometry_msg.pose.pose.position.y = position[1]*0.001
                    self.odometry_msg.pose.pose.position.z = position[2]*0.001 

                    #orientation
                    self.odometry_msg.pose.pose.orientation.x = qx
                    self.odometry_msg.pose.pose.orientation.y = qy
                    self.odometry_msg.pose.pose.orientation.z = qz
                    self.odometry_msg.pose.pose.orientation.w = qw 

                    # linear speed
                    self.odometry_msg.twist.twist.linear.x = 0. 
                    self.odometry_msg.twist.twist.linear.y = 0.
                    self.odometry_msg.twist.twist.linear.z = 0.

                    # angular speed
                    self.odometry_msg.twist.twist.angular.x = 0.
                    self.odometry_msg.twist.twist.angular.y = 0.
                    self.odometry_msg.twist.twist.angular.z = 0.
                    self.tf_broadcaster.sendTransform(self.transform_msg)
                    self.odometry_publisher.publish(self.odometry_msg)

                else:
                    pass # ignore other bodies

                i += 1
            self.prec_timestamp = self.new_timestamp

                
    def create_body_index(self,xml_string):
        """ Extract a name to index dictionary from 6dof settings xml,
            Taken from qualisys_python_sdk/examples/stream_6dof_example.py """
        
        xml = ET.fromstring(xml_string)

        for index, body in enumerate(xml.findall("*/Body/Name")):
            self.body_index[body.text.strip()] = index

        return self.body_index
    
    def quaternions_from_rot(self,rotation):
        """ Converts a rotation matrix to quaternions using Mike Day's algorithm
        (https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf)"""

        m00, m01, m02 = rotation[0][0], rotation[0][1], rotation[0][2]
        m10, m11, m12 = rotation[0][3], rotation[0][4], rotation[0][5]
        m20, m21, m22 = rotation[0][6], rotation[0][7], rotation[0][8]

        if (m22 < 0):
            if (m00 > m11):
                t = 1 + m00 - m11 - m22
                qx,qy,qz,qw = t, m01+m10, m20+m02, m12-m21
                
            
            else:
                t = 1 - m00 + m11 - m22
                qx,qy,qz,qw = m01+m10, t, m12+m21, m20-m02 
                
        else:
            if (m00 < -m11):
                t = 1 - m00 - m11 + m22
                qx,qy,qz,qw = m20+m02, m12+m21, t, m01-m10 
                
            else:
                t = 1 + m00 + m11 + m22
                qx,qy,qz,qw = m12-m21, m20-m02, m01-m10, t 
                

        qx *= 0.5 / math.sqrt(t)
        qy *= 0.5 / math.sqrt(t)
        qz *= 0.5 / math.sqrt(t)
        qw *= 0.5 / math.sqrt(t)

        return qx, qy, qz, qw



    # Class attributes
    transform_msg = TransformStamped()
    odometry_msg = Odometry()
    body_index = {}
    new_timestamp = 0
    prec_timestamp = 0


def main(args=None):
    rclpy.init(args=args)

    node = MocapOdometryNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()