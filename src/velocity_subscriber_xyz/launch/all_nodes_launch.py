from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Resolve URDF file path dynamically
    urdf_file_path = os.path.join(
        get_package_share_directory('robot_publisher_pkg'),
        'urdf',
        'robot_model.urdf'
    )

    # Ensure the URDF file exists
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_file_path}")

    return LaunchDescription([
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
        
        # Optional Joint State Publisher Node
        Node(
            package='robot_publisher_pkg',
            executable='custom_joint_state_publisher',
            name='custom_joint_state_publisher',
            output='screen',
        ),

        # IMU Publisher Node
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            output='screen',
        ),

        # IMU Broadcaster Node
        Node(
            package='bno055',
            executable='imu_tf_broadcaster',
            name='imu_tf_broadcaster',
            output='screen',
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
            parameters=[os.path.join(
                get_package_share_directory('robot_localization'),
                'params',
                'ekf.yaml')],
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
            parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser_frame'}],
            output='screen'
        )
    ])

