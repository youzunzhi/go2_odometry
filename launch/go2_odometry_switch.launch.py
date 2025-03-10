from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    odom_type_arg = DeclareLaunchArgument(
        'odom_type',
        default_value='use_full_odom',
        description='Type of odometry desired between : fake, mocap'
    )

    mocap_launch_file = PathJoinSubstitution([
                        FindPackageShare('go2_odometry'),
                        'launch',
                        'go2_mocap.launch.py'
                        ])
    
    fake_odom_launch_file = PathJoinSubstitution([
                            FindPackageShare('go2_odometry'),
                            'launch',
                            'go2_fake_odom.launch.py'
                            ])

    full_state_publisher_launch_file = PathJoinSubstitution([
                                    FindPackageShare('go2_odometry'),
                                    'launch',
                                    'go2_full_odometry.launch.py'
                                  ])


    return LaunchDescription([

        odom_type_arg,

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([mocap_launch_file]),
        condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'mocap"]))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([fake_odom_launch_file]),
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'fake"]))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([full_state_publisher_launch_file]),
            condition=IfCondition(PythonExpression(["'",LaunchConfiguration('odom_type'),"' == 'use_full_odom'"]))
        )

])