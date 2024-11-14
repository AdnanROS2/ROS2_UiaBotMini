from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='imu_broadcaster',
            name='imu_broadcaster',
            output='screen'
        ),
    ])
