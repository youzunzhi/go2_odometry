#!/bin/env python3

# Gives odometry of the robot_base based on motion capture readings
import asyncio
import qtm_rt
import PyKDL

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry



''' Example of transformation
import tf.transformations as tr

R = tr.random_rotation_matrix()
# Note, in the transformations library conventions, even though the above
# method says it returns a rotation matrix, it actually returns a 4x4 SE(3)
# matrix with the rotation portion in the upper left 3x3 block.
q = tr.quaternion_from_matrix(R)
'''


class MocapOdometryNode(Node):
    def __init__(self):

        self.declare_parameter("base_height", 0.30)
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("odom_frame", "odom")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odometry_publisher = self.create_publisher(Odometry, 'odometry/filtered', 10)

        self.timer = self.create_timer(0.01, self.publish_odom_cb) # publishes the odom every callback

        asyncio.ensure_future(self.setup())
        
    async def setup(self):
        """ Connects to the Motion Capture system and sets callback on packet received """
        connection = await qtm_rt.connect("192.168.75.2",version="1.24") # version changed to 1.24
        
        if connection is None:
            return
        
        await connection.stream_frames(components=["6d"], on_packet=self.on_packet)

    def on_packet(self.packet):
        """ Callback function that is called everytime a data packet arrives from QTM """

        # print("Framenumber: {}".format(packet.framenumber))
        info, bodies = self.packet.get_6d()
        # print("Framenumber: {} - Body count: {}".format(packet.framenumber, info.body_count))
        # print(type(bodies))
        for body in bodies:
            #Todo if body is Go2 (cf qualisys_python_sdk/examples/stream_6dof_example.py)
            position, rotation = body
            rotation_matrix = PyKDL.Rotation(rotation[0][0], rotation[0][1], rotation[0][2],rotation[0][3], rotation[0][4], rotation[0][5],rotation[0][6],rotation[0][7],rotation[0][8])
            # TODO add conversion to quaternion





            timestamp = self.get_clock().now().to_msg()

            # Transform from odom_frame to base_frame
            transform_msg = TransformStamped()
            transform_msg.header.stamp = timestamp
            transform_msg.child_frame_id = self.get_parameter("base_frame").value
            transform_msg.header.frame_id = self.get_parameter("odom_frame").value

            transform_msg.transform.translation.x = position[0]
            transform_msg.transform.translation.y = position[1]
            transform_msg.transform.translation.z = position[2] # + self.get_parameter("base_height").value

            transform_msg.transform.rotation.x = 0
            transform_msg.transform.rotation.y = 0.
            transform_msg.transform.rotation.z = 0.
            transform_msg.transform.rotation.w = 1.

            self.tf_broadcaster.sendTransform(transform_msg)

            odometry_msg = Odometry()
            odometry_msg.header.stamp = timestamp
            odometry_msg.child_frame_id = self.get_parameter("base_frame").value
            odometry_msg.header.frame_id = self.get_parameter("odom_frame").value

            odometry_msg.pose.pose.position.x = position[0]
            odometry_msg.pose.pose.position.y = position[1]
            odometry_msg.pose.pose.position.z = position[2] # + self.get_parameter("base_height").value

            odometry_msg.pose.pose.orientation.x = 0.
            odometry_msg.pose.pose.orientation.y = 0.
            odometry_msg.pose.pose.orientation.z = 0.
            odometry_msg.pose.pose.orientation.w = 1. # find & change

            odometry_msg.twist.twist.linear.x = 0. # vitesse linéaire
            odometry_msg.twist.twist.linear.y = 0.
            odometry_msg.twist.twist.linear.z = 0.

            odometry_msg.twist.twist.angular.x = 0. # vitesse angulaire
            odometry_msg.twist.twist.angular.y = 0.
            odometry_msg.twist.twist.angular.z = 0.

        
        # #
        # Odometry / state estimation
        # odometry_msg.header.stamp = timestamp
        # odometry_msg.header.frame_id = "odom"
        # odometry_msg.child_frame_id = "base"

        # odometry_msg.pose.pose.position.x,
        #  odometry_msg.pose.pose.position.y,
        #  odometry_msg.pose.pose.position.z = position

        # odometry_msg.pose.pose.orientation.x,
        #  odometry_msg.pose.pose.orientation.y,
        #  odometry_msg.pose.pose.orientation.z,
        #  odometry_msg.pose.pose.orientation.w = orientation

        # odometry_msg.twist.twist.linear.x,
        #  odometry_msg.twist.twist.linear.y,
        #  odometry_msg.twist.twist.linear.z = linear_vel

        # odometry_msg.twist.twist.angular.x,
        #  odometry_msg.twist.twist.angular.y,
        #  odometry_msg.twist.twist.angular.z = angular_vel

        # self.odometry_publisher.publish(odometry_msg)

        # # Forwar odometry on tf
        # transform_msg.header.stamp = timestamp
        # transform_msg.header.frame_id = "odom"
        # transform_msg.child_frame_id = "base"
        # transform_msg.transform.translation.x,
        #  transform_msg.transform.translation.y,
        #  transform_msg.transform.translation.z = position

        # transform_msg.transform.rotation.x,
        #  transform_msg.transform.rotation.y,
        #  transform_msg.transform.rotation.z,
        #  transform_msg.transform.rotation.w  = orientation

        # self.tf_broadcaster.sendTransform(transform_msg)
        # #
        

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

def main(args=None):
    rclpy.init(args=args)

    node = MocapOdometryNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
