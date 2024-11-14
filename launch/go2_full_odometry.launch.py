from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config_file             = PathJoinSubstitution([
                                    FindPackageShare("go2_odometry"),
                                    'config',
                                    'go2_ekf.yaml'
                                  ])
    state_publisher_launch_file = PathJoinSubstitution([
                                    FindPackageShare('go2_odometry'),
                                    'launch',
                                    'go2_state_publisher.launch.py'
                                  ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_publisher_launch_file])
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file],
           ),
        Node(
            package='go2_odometry',
            executable='feet_to_odom.py',
            name='feet_to_odom',
            output='screen',
            parameters=[],
           ),
])
