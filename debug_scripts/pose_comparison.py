#!/usr/bin/env python3
import csv
from datetime import datetime
from pathlib import Path

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
        self.output_dir = Path('comparison_output')
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.timestamp = datetime.now().strftime('%m%d-%H%M%S')

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
        if not self.time_data:
            self.get_logger().warn('No synchronized samples received. Nothing to save.')
            rclpy.shutdown()
            return

        suffix = f'lowstate_{self.timestamp}'
        plot_path = self.output_dir / f'pose_comparison_{suffix}.png'
        csv_path = self.output_dir / f'pose_comparison_{suffix}.csv'

        self.get_logger().info(f'Saving plot to {plot_path}')
        plt.tight_layout()
        self.fig.savefig(plot_path)

        self.save_raw_csv(csv_path)
        self.print_error_statistics()
        self.get_logger().info(f'Output saved under {self.output_dir}/. Shutting down.')
        rclpy.shutdown()

    def save_raw_csv(self, csv_path: Path):
        """
        Save synchronized raw pose-difference data to CSV.
        """
        with csv_path.open('w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    'time_s',
                    'pos_diff_x_m', 'pos_diff_y_m', 'pos_diff_z_m',
                    'orient_diff_roll_rad', 'orient_diff_pitch_rad', 'orient_diff_yaw_rad',
                    'orient_diff_roll_deg', 'orient_diff_pitch_deg', 'orient_diff_yaw_deg',
                ]
            )
            for i, t in enumerate(self.time_data):
                roll_rad = self.orient_diff['roll'][i]
                pitch_rad = self.orient_diff['pitch'][i]
                yaw_rad = self.orient_diff['yaw'][i]
                writer.writerow(
                    [
                        t,
                        self.pos_diff['x'][i], self.pos_diff['y'][i], self.pos_diff['z'][i],
                        roll_rad, pitch_rad, yaw_rad,
                        np.rad2deg(roll_rad), np.rad2deg(pitch_rad), np.rad2deg(yaw_rad),
                    ]
                )
        self.get_logger().info(f'Raw comparison data saved to {csv_path}')

    def _stats_for(self, data):
        arr = np.asarray(data, dtype=float)
        return {
            'mean': float(np.mean(arr)),
            'std': float(np.std(arr)),
            'rmse': float(np.sqrt(np.mean(np.square(arr)))),
            'max_abs': float(np.max(np.abs(arr))),
        }

    def print_error_statistics(self):
        """
        Print summary statistics for position and orientation error.
        """
        self.get_logger().info('--- Error Statistics ---')
        for axis in ['x', 'y', 'z']:
            s = self._stats_for(self.pos_diff[axis])
            self.get_logger().info(
                f"Position {axis} [m] | mean={s['mean']:.6f}, std={s['std']:.6f}, "
                f"rmse={s['rmse']:.6f}, max_abs={s['max_abs']:.6f}"
            )
        for axis in ['roll', 'pitch', 'yaw']:
            rad_stats = self._stats_for(self.orient_diff[axis])
            deg_values = np.rad2deg(self.orient_diff[axis])
            deg_stats = self._stats_for(deg_values)
            self.get_logger().info(
                f"Orientation {axis} | mean={rad_stats['mean']:.6f} rad ({deg_stats['mean']:.3f} deg), "
                f"std={rad_stats['std']:.6f} rad ({deg_stats['std']:.3f} deg), "
                f"rmse={rad_stats['rmse']:.6f} rad ({deg_stats['rmse']:.3f} deg), "
                f"max_abs={rad_stats['max_abs']:.6f} rad ({deg_stats['max_abs']:.3f} deg)"
            )
        self.get_logger().info('------------------------')

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
