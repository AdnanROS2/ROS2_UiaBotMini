

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start robot_state_publisher for URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}, '/home/adnan/Ros2_WS_Real/src/robot_publisher/urdf/simplified_robot.urdf']
        ),

        # Start joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # Start diff_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
            parameters=['/home/adnan/Ros2_WS_Real/src/robot_publisher/urdf/diff_drive_controller.yaml']  # Oppdatert sti her
        ),
    ])

