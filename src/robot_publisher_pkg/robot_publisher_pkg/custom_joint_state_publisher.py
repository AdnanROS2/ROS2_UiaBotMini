import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math


class CustomJointStatePublisher(Node):
    def __init__(self):
        super().__init__('custom_joint_state_publisher')

        # Subscriber to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for /joint_states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Robot parameters
        self.wheel_separation = 0.3  # Distance between wheels (meters)
        self.wheel_radius = 0.05     # Wheel radius (meters)

        # Joint positions
        self.right_wheel_position = 0.0
        self.left_wheel_position = 0.0

        # Velocity placeholders
        self.right_wheel_velocity = 0.0
        self.left_wheel_velocity = 0.0

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        self.get_logger().info("CustomJointStatePublisher initialized.")

    def cmd_vel_callback(self, msg):
        """
        Callback for /cmd_vel. Converts Twist messages to wheel velocities.
        """
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate individual wheel velocities (rad/s)
        self.right_wheel_velocity = (linear_velocity + angular_velocity * self.wheel_separation / 2) / self.wheel_radius
        self.left_wheel_velocity = (linear_velocity - angular_velocity * self.wheel_separation / 2) / self.wheel_radius

    def publish_joint_states(self):
        """
        Publish the wheel joint states based on the velocities.
        """
        # Time step for integration (assuming 10 Hz update rate)
        dt = 0.1

        # Update wheel positions based on velocities
        self.right_wheel_position += self.right_wheel_velocity * dt
        self.left_wheel_position += self.left_wheel_velocity * dt

        # Create and publish the JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['right_wheel_joint', 'left_wheel_joint']
        msg.position = [self.right_wheel_position, self.left_wheel_position]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published joint states: {msg.position}")


def main(args=None):
    rclpy.init(args=args)
    node = CustomJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CustomJointStatePublisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

