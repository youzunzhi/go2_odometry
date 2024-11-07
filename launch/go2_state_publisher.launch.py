import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from example_robot_data import getModelPath

def generate_launch_description():
    # Read go2 urdf from example-robot-data
    urdf_subpath = "go2_description/urdf/go2.urdf"
    urdf_path = os.path.join(getModelPath(urdf_subpath), urdf_subpath)
    print(urdf_path)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
            arguments=[urdf_path]),
        Node(
            package='go2_odometry',
            executable='state_converter_node',
            name='state_converter_node',
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])