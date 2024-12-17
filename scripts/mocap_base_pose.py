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
        super().__init__('feet_to_odom')

        self.declare_parameter("base_height", 0.30)
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

        xml_string = await connection.get_parameters(parameters=["6d"])
        self.body_index = self.create_body_index(xml_string)
        
        if connection is None:
            print("Did not connect")
            return
        
        await connection.stream_frames(components=["6d"], on_packet=self.on_packet)

    def on_packet(self, packet):
        """ Callback function called every time a data packet arrives from QTM """

        info, bodies = packet.get_6d()
 
        # print("Framenumber: {} - Body count: {}".format(packet.framenumber, info.body_count))


        for body in bodies:
            # select wanted body among all rigid bodies streamed
            if self.get_parameter("wanted_body").value is not None and self.get_parameter("wanted_body").value in self.body_index:

                # print("Object {} has been found!".format(self.get_parameter("wanted_body").value))
                
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
                    transform_msg.transform.translation.z = position[2]*0.001 # + self.get_parameter("base_height").value

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

                    # position
                    odometry_msg.pose.pose.position.x = position[0]*0.001
                    odometry_msg.pose.pose.position.y = position[1]*0.001
                    odometry_msg.pose.pose.position.z = position[2]*0.001 # + self.get_parameter("base_height").value

                    #orientation
                    odometry_msg.pose.pose.orientation.x = qx
                    odometry_msg.pose.pose.orientation.y = qy
                    odometry_msg.pose.pose.orientation.z = qz
                    odometry_msg.pose.pose.orientation.w = qw 

                    # vitesse linéaire
                    odometry_msg.twist.twist.linear.x = 0. 
                    odometry_msg.twist.twist.linear.y = 0.
                    odometry_msg.twist.twist.linear.z = 0.

                    # vitesse angulaire
                    odometry_msg.twist.twist.angular.x = 0.
                    odometry_msg.twist.twist.angular.y = 0.
                    odometry_msg.twist.twist.angular.z = 0.

                    self.odometry_publisher.publish(odometry_msg)
            else :
                print("Object {} was not found".format(self.get_parameter("wanted_body").value))
                #!change to roslog
                

    def create_body_index(self,xml_string):
        """ Extract a name to index dictionary from 6dof settings xml,
        Taken from qualisys_python_sdk/examples/stream_6dof_example.py"""
        
        xml = ET.fromstring(xml_string)

        # body_to_index = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            self.body_index[body.text.strip()] = index

        print(self.body_index)
        return self.body_index

    # Class attributes
    body_index = {}

'''
    def publish_odom_cb(self):
        timestamp = self.get_clock().now().to_msg()

        # Transform from odom_frame to base_frame
        transform_msg = TransformStamped()
        transform_msg.header.stamp = timestamp
        transform_msg.child_frame_id = self.get_parameter("base_frame").value
        transform_msg.header.frame_id = self.get_parameter("odom_frame").value

        transform_msg.transform.translation.x = 0.
        transform_msg.transform.translation.y = 0.
        transform_msg.transform.translation.z = self.get_parameter("base_height").value

        transform_msg.transform.rotation.x = 0.
        transform_msg.transform.rotation.y = 0.
        transform_msg.transform.rotation.z = 0.
        transform_msg.transform.rotation.w = 1.

        self.tf_broadcaster.sendTransform(transform_msg)


        odometry_msg = Odometry()
        odometry_msg.header.stamp = timestamp
        odometry_msg.child_frame_id = self.get_parameter("base_frame").value
        odometry_msg.header.frame_id = self.get_parameter("odom_frame").value

        odometry_msg.pose.pose.position.x = 0.
        odometry_msg.pose.pose.position.y = 0.
        odometry_msg.pose.pose.position.z = self.get_parameter("base_height").value

        odometry_msg.pose.pose.orientation.x = 0.
        odometry_msg.pose.pose.orientation.y = 0.
        odometry_msg.pose.pose.orientation.z = 0.
        odometry_msg.pose.pose.orientation.w = 1.

        odometry_msg.twist.twist.linear.x = 0.
        odometry_msg.twist.twist.linear.y = 0.
        odometry_msg.twist.twist.linear.z = 0.

        odometry_msg.twist.twist.angular.x = 0.
        odometry_msg.twist.twist.angular.y = 0.
        odometry_msg.twist.twist.angular.z = 0.

        self.odometry_publisher.publish(odometry_msg)
'''

def main(args=None):
    rclpy.init(args=args)

    node = MocapOdometryNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






#---------------------------------------------------------------------------------------------------------------------------------------------------------------------

"""
    Streaming 6Dof from QTM


import asyncio
import xml.etree.ElementTree as ET
import pkg_resources

import qtm_rt

QTM_FILE = pkg_resources.resource_filename("qtm_rt", "data/Demo.qtm")


def create_body_index(xml_string):
   #Extract a name to index dictionary from 6dof settings xml 
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index

def body_enabled_count(xml_string):
    xml = ET.fromstring(xml_string)
    return sum(enabled.text == "true" for enabled in xml.findall("*/Body/Enabled"))

async def main():

    # Connect to qtm
    connection = await qtm_rt.connect("127.0.0.1")

    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return

    # Take control of qtm, context manager will automatically release control after scope end
    async with qtm_rt.TakeControl(connection, "password"):

        realtime = False

        if realtime:
            # Start new realtime
            await connection.new()
        else:
            # Load qtm file
            await connection.load(QTM_FILE)

            # start rtfromfile
            await connection.start(rtfromfile=True)

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    print("{} of {} 6DoF bodies enabled".format(body_enabled_count(xml_string), len(body_index)))

    wanted_body = "L-frame"

    def on_packet(packet):
        info, bodies = packet.get_6d()
        print(
            "Framenumber: {} - Body count: {}".format(
                packet.framenumber, info.body_count
            )
        )

        if wanted_body is not None and wanted_body in body_index:
            # Extract one specific body
            wanted_index = body_index[wanted_body]
            position, rotation = bodies[wanted_index]
            print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))
        else:
            # Print all bodies
            for position, rotation in bodies:
                print("Pos: {} - Rot: {}".format(position, rotation))

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)

    # Wait asynchronously 5 seconds
    await asyncio.sleep(5)

    # Stop streaming
    await connection.stream_frames_stop()


if __name__ == "__main__":
    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
"""