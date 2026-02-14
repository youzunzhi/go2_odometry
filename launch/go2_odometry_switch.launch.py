from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    odom_type_arg = DeclareLaunchArgument(
        "odom_type", default_value="use_full_odom", description="Type of odometry desired between : fake, mocap"
    )

    fake_odom_base_height_arg = DeclareLaunchArgument(
        "base_height",
        default_value=TextSubstitution(text="0.30"),
        description="[IF FAKE ODOM] Height of the robot base.",
    )
    inekf_imu_source_arg = DeclareLaunchArgument(
        "imu_source",
        default_value="lowstate",
        description="[IF USE_FULL_ODOM] IMU source used by InEKF: lowstate or utlidar",
    )
    inekf_utlidar_topic_arg = DeclareLaunchArgument(
        "utlidar_imu_topic",
        default_value="/utlidar/imu",
        description="[IF USE_FULL_ODOM] Topic used when imu_source is utlidar",
    )
    inekf_imu_rotation_arg = DeclareLaunchArgument(
        "imu_rotation_rpy",
        default_value=TextSubstitution(text="[0.0, 0.0, 0.0]"),
        description="[IF USE_FULL_ODOM] RPY rotation (rad) from selected imu frame to filter imu frame",
    )
    inekf_imu_translation_arg = DeclareLaunchArgument(
        "imu_translation_xyz",
        default_value=TextSubstitution(text="[0.0, 0.0, 0.0]"),
        description="[IF USE_FULL_ODOM] Translation (m) from selected imu origin to filter imu origin",
    )
    inekf_compensate_translation_arg = DeclareLaunchArgument(
        "compensate_imu_translation",
        default_value="false",
        description="[IF USE_FULL_ODOM] Apply translation compensation between IMU origins",
    )

    mocap_base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base",
        description="[IF MOCAP] Name of the base frame (fixed to robot) of the mocap.",
    )
    mocap_odom_frame_arg = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="[IF MOCAP] Name of the odom frame (fixed to world) of the mocap.",
    )
    mocap_object_name_arg = DeclareLaunchArgument(
        "wanted_body", default_value="Go2", description="[IF MOCAP] Name of the object tracked by the motion capture."
    )
    mocap_ip_arg = DeclareLaunchArgument(
        "qualisys_ip",
        default_value="128.93.64.222",
        description="[IF MOCAP] IP used by the motion capture to publish data.",
    )
    mocap_publishing_freq_arg = DeclareLaunchArgument(
        "publishing_freq",
        default_value=TextSubstitution(text="110"),
        description="[IF MOCAP] Publishing frequency of the odom to robot base transform. Max limit of 300Hz.",
    )

    # launchfiles
    mocap_launch_file = PathJoinSubstitution([FindPackageShare("go2_odometry"), "launch", "go2_mocap.launch.py"])

    fake_odom_launch_file = PathJoinSubstitution(
        [FindPackageShare("go2_odometry"), "launch", "go2_fake_odom.launch.py"]
    )

    full_state_publisher_launch_file = PathJoinSubstitution(
        [FindPackageShare("go2_odometry"), "launch", "go2_inekf_odometry.launch.py"]
    )

    return LaunchDescription(
        [
            mocap_base_frame_arg,
            mocap_odom_frame_arg,
            mocap_object_name_arg,
            mocap_ip_arg,
            mocap_publishing_freq_arg,
            fake_odom_base_height_arg,
            inekf_imu_source_arg,
            inekf_utlidar_topic_arg,
            inekf_imu_rotation_arg,
            inekf_imu_translation_arg,
            inekf_compensate_translation_arg,
            odom_type_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([mocap_launch_file]),
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration("odom_type"), "' == 'mocap'"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([fake_odom_launch_file]),
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration("odom_type"), "' == 'fake'"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([full_state_publisher_launch_file]),
                launch_arguments={
                    "imu_source": LaunchConfiguration("imu_source"),
                    "utlidar_imu_topic": LaunchConfiguration("utlidar_imu_topic"),
                    "imu_rotation_rpy": LaunchConfiguration("imu_rotation_rpy"),
                    "imu_translation_xyz": LaunchConfiguration("imu_translation_xyz"),
                    "compensate_imu_translation": LaunchConfiguration("compensate_imu_translation"),
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("odom_type"), "' == 'use_full_odom'"])
                ),
            ),
        ]
    )
