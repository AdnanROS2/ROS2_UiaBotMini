from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the IMU Publisher Node
        Node(
            package='imu_packages_pkg',  # Replace with the actual package name
            executable='imu_publisher',  # Ensure this matches the publisher script's filename
            name='imu_publisher',
            output='screen'
        ),
        
        # Start the IMU Broadcaster Node
        Node(
            package='imu_packages_pkg',  # Replace with the actual package name
            executable='imu_broadcaster',  # Ensure this matches the broadcaster script's filename
            name='imu_broadcaster',
            output='screen'
        )
    ])

