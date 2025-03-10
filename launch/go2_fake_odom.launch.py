from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    minimal_state_publisher_launch_file = PathJoinSubstitution([
                                    FindPackageShare('go2_odometry'),
                                    'launch',
                                    'go2_state_publisher.launch.py'
                                  ])


    return LaunchDescription([

        Node(
            package="go2_odometry",
            executable="fake_odom.py",
            name='fake_odom'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([minimal_state_publisher_launch_file])

        )
])