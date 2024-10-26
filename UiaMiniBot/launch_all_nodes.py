import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Korrekt sti til URDF-filen
    urdf_path = '/home/adnan/Ros2_WS_Final/src/robot_publisher_pkg/urdf/robot_model.urdf'

    # Les inn URDF-innholdet direkte
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    return LaunchDescription([
        # Kjør robot_state_publisher for å publisere URDF-modellen
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        # Start RViz for å vise robotmodellen
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
