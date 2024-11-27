#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Lidar configuration
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # Resolve URDF file path dynamically
    urdf_file_path = os.path.join(
        get_package_share_directory('robot_publisher_pkg'),
        'urdf',
        'robot_model.urdf'
    )

    # Ensure the URDF file exists
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_file_path}")

    # Localization parameters file
    ekf_params_file = os.path.join(
        get_package_share_directory('robot_localization'),
        'params',
        'ekf.yaml'
    )

    # BNO055 parameter file
    config = os.path.join(
        get_package_share_directory('bno055'),
        'config',
        'bno055_params.yaml'
        )

    return LaunchDescription([
        # Declare Lidar Arguments
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying USB port to which lidar is connected'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying USB port baudrate for lidar'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle compensation of scan data'
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'
        ),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_file_path).read()}
            ]
        ),

        # Odometry Node
        Node(
            package='odometry_pkg',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),

        # EKF Localization Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file],
        ),

        # Velocity Subscriber Node
        Node(
            package='velocity_subscriber_xyz',
            executable='vel_sub',
            name='velocity_subscriber',
            output='screen',
        ),

        # LiDAR Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[
                {
                    'channel_type': channel_type,
                    'serial_port': serial_port,
                    'serial_baudrate': serial_baudrate,
                    'frame_id': frame_id,
                    'inverted': inverted,
                    'angle_compensate': angle_compensate,
                    'scan_mode': scan_mode
                }
            ],
            output='screen',
        ),

        # BNO055 Node
        Node(
            package='imu_packages_pkg',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
        ),
        
        # IMU Broadcaster Node
        Node(
            package='imu_packages_pkg',
            executable='imu_broadcaster',
            name='imu_broadcaster',
            output='screen',
        )
    ])

