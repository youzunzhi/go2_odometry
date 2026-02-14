#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters
import matplotlib.pyplot as plt
import numpy as np

class PoseComparator(Node):
    """
    A ROS2 node to compare pose data from two odometry topics and visualize the difference.
    """
    def __init__(self):
        """
        Initializes the node, subscribers, and plot.
        """
        super().__init__('pose_comparator')

        # Use message_filters to subscribe to the topics and synchronize them.
        self.utlidar_sub = message_filters.Subscriber(self, Odometry, '/utlidar/robot_odom')
        self.filtered_sub = message_filters.Subscriber(self, Odometry, '/odometry/filtered')

        # ApproximateTimeSynchronizer to handle messages that don't have the exact same timestamp.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.utlidar_sub, self.filtered_sub], 10, 0.1)
        self.ts.registerCallback(self.odom_callback)

        # Data storage for plotting
        self.time_data = []
        self.pos_diff = {'x': [], 'y': [], 'z': []}
        self.orient_diff = {'roll': [], 'pitch': [], 'yaw': []}

        # Initialize the plot
        self.fig, self.ax = plt.subplots(2, 1)
        self.start_time = self.get_clock().now()
        
        # Timer to save plot and shutdown after 5 seconds
        self.create_timer(15.0, self.save_plot_and_shutdown)

    def save_plot_and_shutdown(self):
        """
        Saves the plot and shuts down the node.
        """
        self.get_logger().info('Saving plot to pose_comparison.png')
        plt.tight_layout()
        self.fig.savefig('pose_comparison.png')
        self.get_logger().info('Plot saved. Shutting down.')
        rclpy.shutdown()

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw).
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def odom_callback(self, utlidar_msg, filtered_msg):
        """
        Callback function for synchronized odometry messages.
        Calculates and stores the difference in pose.
        """
        # Position difference
        pos1 = utlidar_msg.pose.pose.position
        pos2 = filtered_msg.pose.pose.position
        self.pos_diff['x'].append(pos1.x - pos2.x)
        self.pos_diff['y'].append(pos1.y - pos2.y)
        self.pos_diff['z'].append(pos1.z - pos2.z)

        # Orientation difference
        q1 = utlidar_msg.pose.pose.orientation
        q2 = filtered_msg.pose.pose.orientation
        roll1, pitch1, yaw1 = self.quaternion_to_euler(q1.x, q1.y, q1.z, q1.w)
        roll2, pitch2, yaw2 = self.quaternion_to_euler(q2.x, q2.y, q2.z, q2.w)

        self.orient_diff['roll'].append(np.unwrap([0, roll1 - roll2])[1])
        self.orient_diff['pitch'].append(np.unwrap([0, pitch1 - pitch2])[1])
        self.orient_diff['yaw'].append(np.unwrap([0, yaw1 - yaw2])[1])
        
        time_now = self.get_clock().now()
        self.time_data.append((time_now - self.start_time).nanoseconds / 1e9)

        self.update_plot()

    def update_plot(self):
        """
        Updates the matplotlib plot with the latest data.
        """
        # Plot position differences
        self.ax[0].clear()
        self.ax[0].plot(self.time_data, self.pos_diff['x'], label='x diff')
        self.ax[0].plot(self.time_data, self.pos_diff['y'], label='y diff')
        self.ax[0].plot(self.time_data, self.pos_diff['z'], label='z diff')
        self.ax[0].legend()
        self.ax[0].set_title('Position Difference (utlidar - filtered)')
        self.ax[0].set_xlabel('Time (s)')
        self.ax[0].set_ylabel('Difference (m)')

        # Plot orientation differences
        self.ax[1].clear()
        self.ax[1].plot(self.time_data, np.rad2deg(self.orient_diff['roll']), label='roll diff')
        self.ax[1].plot(self.time_data, np.rad2deg(self.orient_diff['pitch']), label='pitch diff')
        self.ax[1].plot(self.time_data, np.rad2deg(self.orient_diff['yaw']), label='yaw diff')
        self.ax[1].legend()
        self.ax[1].set_title('Orientation Difference (utlidar - filtered)')
        self.ax[1].set_xlabel('Time (s)')
        self.ax[1].set_ylabel('Difference (deg)')

        # No interactive plotting, plot will be saved on exit

def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    pose_comparator = PoseComparator()
    
    try:
        rclpy.spin(pose_comparator)
    except KeyboardInterrupt:
        pass
    finally:
        pose_comparator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
