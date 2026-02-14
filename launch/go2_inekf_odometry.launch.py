from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_source_arg = DeclareLaunchArgument(
        "imu_source",
        default_value="lowstate",
        description="IMU source used by InEKF: lowstate or utlidar",
    )
    utlidar_imu_topic_arg = DeclareLaunchArgument(
        "utlidar_imu_topic",
        default_value="/utlidar/imu",
        description="Topic used when imu_source is utlidar",
    )
    imu_rotation_rpy_arg = DeclareLaunchArgument(
        "imu_rotation_rpy",
        default_value=TextSubstitution(text="[0.0, 0.0, 0.0]"),
        description="RPY extrinsic rotation (rad) from selected imu frame to filter imu frame",
    )
    imu_translation_xyz_arg = DeclareLaunchArgument(
        "imu_translation_xyz",
        default_value=TextSubstitution(text="[0.0, 0.0, 0.0]"),
        description="Extrinsic translation (m) from selected imu origin to filter imu origin",
    )
    compensate_imu_translation_arg = DeclareLaunchArgument(
        "compensate_imu_translation",
        default_value="false",
        description="Apply translation compensation between IMU origins",
    )

    state_publisher_launch_file = PathJoinSubstitution(
        [FindPackageShare("go2_odometry"), "launch", "go2_state_publisher.launch.py"]
    )

    return LaunchDescription(
        [
            imu_source_arg,
            utlidar_imu_topic_arg,
            imu_rotation_rpy_arg,
            imu_translation_xyz_arg,
            compensate_imu_translation_arg,
            IncludeLaunchDescription(PythonLaunchDescriptionSource([state_publisher_launch_file])),
            Node(
                package="go2_odometry",
                executable="inekf_odom.py",
                name="inekf_odom",
                output="screen",
                parameters=[
                    {
                        "imu_source": LaunchConfiguration("imu_source"),
                        "utlidar_imu_topic": LaunchConfiguration("utlidar_imu_topic"),
                        "imu_rotation_rpy": LaunchConfiguration("imu_rotation_rpy"),
                        "imu_translation_xyz": LaunchConfiguration("imu_translation_xyz"),
                        "compensate_imu_translation": ParameterValue(
                            LaunchConfiguration("compensate_imu_translation"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
