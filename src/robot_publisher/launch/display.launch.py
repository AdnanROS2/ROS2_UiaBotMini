from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file =  '/home/adnan/Ros2_WS_Real/src/robot_publisher/urdf/simplified_robot.urdf'


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

