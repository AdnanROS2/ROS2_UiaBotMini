from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, ExecuteProcess
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Få stien til URDF- og RViz-filene
    urdf_path = os.path.join(
        get_package_share_directory('robot_publisher_pkg'),
        'urdf',
        'robot_model.urdf'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('robot_publisher_pkg'),
        'rviz',
        'robot_view.rviz'
    )

    # Sett opp robot_description-parameteren ved å lese URDF-filen
    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str
    )

    # Definer robot_publisher noden
    robot_publisher_node = Node(
        package='robot_publisher_pkg',
        executable='robot_publisher',
        name='robot_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Definer odometry_node noden
    odometry_node = Node(
        package='odometry_pkg',
        executable='odometry_node',
        name='odometry_node'
    )

    # Definer RViz prosessen
    rviz_process = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        # Start robot_publisher etter 1 sekund
        TimerAction(
            period=1.0,
            actions=[robot_publisher_node]
        ),
        # Start odometry_node etter 2 sekunder
        TimerAction(
            period=2.0,
            actions=[odometry_node]
        ),
        # Start RViz etter 2 sekunder
        TimerAction(
            period=2.0,
            actions=[rviz_process]
        ),
    ])
