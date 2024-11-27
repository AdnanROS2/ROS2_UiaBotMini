import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped
import transforms3d.quaternions as quat
from tf2_ros import TransformBroadcaster
import math


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Parameters for wheel separation and radius
        self.declare_parameter('wheel_separation', 0.15)  # Example value in meters
        self.declare_parameter('wheel_radius', 0.03)  # Example value in meters

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Subscribers and Publishers
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster for odometry and wheel transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry tracking variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_left_wheel = None
        self.prev_right_wheel = None
        self.prev_time = self.get_clock().now()

    def joint_state_callback(self, msg: JointState):
        current_time = self.get_clock().now()
        try:
            # Extract wheel joint positions
            left_wheel = msg.position[msg.name.index('left_wheel_joint')]
            right_wheel = msg.position[msg.name.index('right_wheel_joint')]
        except ValueError as e:
            self.get_logger().error(f"Joint names not found in /joint_states: {e}")
            return

        # Initialize previous wheel positions
        if self.prev_left_wheel is None or self.prev_right_wheel is None:
            self.prev_left_wheel = left_wheel
            self.prev_right_wheel = right_wheel
            self.prev_time = current_time
            return

        # Calculate deltas
        left_wheel_delta = left_wheel - self.prev_left_wheel
        right_wheel_delta = right_wheel - self.prev_right_wheel

        self.prev_left_wheel = left_wheel
        self.prev_right_wheel = right_wheel

        # Calculate distances and odometry
        left_distance = left_wheel_delta * self.wheel_radius * 0.8
        right_distance = right_wheel_delta * self.wheel_radius
        distance = (left_distance + right_distance) / 2.0
        dtheta = (right_distance - left_distance) / self.wheel_separation

        # Time delta
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Update robot position
        self.th += dtheta
        self.x += distance * math.cos(self.th)
        self.y += distance * math.sin(self.th)

        # Publish odometry
        self.publish_odometry(current_time, distance / dt, dtheta / dt)

        # Publish wheel transforms
        self.publish_wheel_transforms(current_time)

    def publish_odometry(self, current_time, linear_velocity, angular_velocity):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom_quat = quat.axangle2quat([0, 0, 1], self.th)
        if not all(math.isfinite(q) for q in odom_quat):
            self.get_logger().error(f"Invalid quaternion calculated: {odom_quat}")
            return

        odom.pose.pose.orientation = Quaternion(
            x=odom_quat[1],
            y=odom_quat[2],
            z=odom_quat[3],
            w=odom_quat[0]
        )

        # Set velocity
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Publish odometry message
        self.odom_publisher.publish(odom)

        # Publish odometry transform
        odom_transform = TransformStamped()
        odom_transform.header.stamp = current_time.to_msg()
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_link"
        odom_transform.transform.translation.x = self.x
        odom_transform.transform.translation.y = self.y
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation = Quaternion(
            x=odom_quat[1],
            y=odom_quat[2],
            z=odom_quat[3],
            w=odom_quat[0]
        )
        self.tf_broadcaster.sendTransform(odom_transform)


    def publish_wheel_transforms(self, current_time):
        # Left wheel transform
        left_wheel_transform = TransformStamped()
        left_wheel_transform.header.stamp = current_time.to_msg()
        left_wheel_transform.header.frame_id = "base_link"
        left_wheel_transform.child_frame_id = "left_wheel"
        left_wheel_transform.transform.translation.x = 0.2
        left_wheel_transform.transform.translation.y = 0.15
        left_wheel_transform.transform.translation.z = -0.05
        left_wheel_transform.transform.rotation.x = 0.0
        left_wheel_transform.transform.rotation.y = 0.0
        left_wheel_transform.transform.rotation.z = 0.0
        left_wheel_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(left_wheel_transform)

        # Right wheel transform
        right_wheel_transform = TransformStamped()
        right_wheel_transform.header.stamp = current_time.to_msg()
        right_wheel_transform.header.frame_id = "base_link"
        right_wheel_transform.child_frame_id = "right_wheel"
        right_wheel_transform.transform.translation.x = 0.2
        right_wheel_transform.transform.translation.y = -0.15
        right_wheel_transform.transform.translation.z = -0.05
        right_wheel_transform.transform.rotation.x = 0.0
        right_wheel_transform.transform.rotation.y = 0.0
        right_wheel_transform.transform.rotation.z = 0.0
        right_wheel_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(right_wheel_transform)


def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        odometry_node.get_logger().info("OdometryNode shutting down.")
    finally:
        odometry_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

