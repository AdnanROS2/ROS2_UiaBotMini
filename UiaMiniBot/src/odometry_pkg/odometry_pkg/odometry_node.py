

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

        # Set wheel_separation and wheel_radius based on URDF values
        self.declare_parameter('wheel_separation', 0.3)  # Updated to 0.3 meters as per URDF
        self.declare_parameter('wheel_radius', 0.1)  # Updated to 0.1 meters as per URDF

        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_left_wheel = None
        self.prev_right_wheel = None
        self.prev_time = self.get_clock().now()

    def joint_state_callback(self, msg: JointState):
        current_time = self.get_clock().now()
        left_wheel = msg.position[msg.name.index('left_wheel_joint')]
        right_wheel = msg.position[msg.name.index('right_wheel_joint')]

        if self.prev_left_wheel is None or self.prev_right_wheel is None:
            self.prev_left_wheel = left_wheel
            self.prev_right_wheel = right_wheel
            self.prev_time = current_time
            return

        left_wheel_delta = left_wheel - self.prev_left_wheel
        right_wheel_delta = right_wheel - self.prev_right_wheel

        self.prev_left_wheel = left_wheel
        self.prev_right_wheel = right_wheel

        # Use the updated wheel radius and separation
        left_distance = left_wheel_delta * self.wheel_radius
        right_distance = right_wheel_delta * self.wheel_radius
        distance = (left_distance + right_distance) / 2.0
        dtheta = (right_distance - left_distance) / self.wheel_separation

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        self.th += dtheta
        self.x += distance * math.cos(self.th)
        self.y += distance * math.sin(self.th)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "world"  # Changed to world frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom_quat = quat.axangle2quat([0, 0, 1], self.th)
        odom.pose.pose.orientation.x = odom_quat[1]
        odom.pose.pose.orientation.y = odom_quat[2]
        odom.pose.pose.orientation.z = odom_quat[3]
        odom.pose.pose.orientation.w = odom_quat[0]
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = distance / dt
        odom.twist.twist.angular.z = dtheta / dt

        self.odom_publisher.publish(odom)

        # Publish transform for base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "world"  # base_link is relative to world
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[1]
        t.transform.rotation.y = odom_quat[2]
        t.transform.rotation.z = odom_quat[3]
        t.transform.rotation.w = odom_quat[0]

        self.tf_broadcaster.sendTransform(t)

        # Publish wheel transforms
        self.publish_wheel_transforms(current_time)

    def publish_wheel_transforms(self, current_time):
        # Transform for left wheel
        left_wheel_transform = TransformStamped()
        left_wheel_transform.header.stamp = current_time.to_msg()
        left_wheel_transform.header.frame_id = "base_link"
        left_wheel_transform.child_frame_id = "left_wheel"
        left_wheel_transform.transform.translation.x = 0.2  # Same as in URDF origin
        left_wheel_transform.transform.translation.y = 0.15
        left_wheel_transform.transform.translation.z = -0.05

        # Here you can adjust the left wheel rotation:
        # Change the second argument to your desired rotation in radians.
        left_wheel_quat = quat.axangle2quat([0, 0, 1], 1.5708)  # Adjust this rotation if needed
        left_wheel_transform.transform.rotation.x = left_wheel_quat[1]
        left_wheel_transform.transform.rotation.y = left_wheel_quat[2]
        left_wheel_transform.transform.rotation.z = left_wheel_quat[3]
        left_wheel_transform.transform.rotation.w = left_wheel_quat[0]

        self.tf_broadcaster.sendTransform(left_wheel_transform)

        # Transform for right wheel
        right_wheel_transform = TransformStamped()
        right_wheel_transform.header.stamp = current_time.to_msg()
        right_wheel_transform.header.frame_id = "base_link"
        right_wheel_transform.child_frame_id = "right_wheel"
        right_wheel_transform.transform.translation.x = 0.2  # Same as in URDF origin
        right_wheel_transform.transform.translation.y = -0.15
        right_wheel_transform.transform.translation.z = -0.05

        # Here you can adjust the right wheel rotation:
        # Change the second argument to your desired rotation in radians.
        right_wheel_quat = quat.axangle2quat([0, 0, 1], 1.5708)  # Adjust this rotation if needed
        right_wheel_transform.transform.rotation.x = right_wheel_quat[1]
        right_wheel_transform.transform.rotation.y = right_wheel_quat[2]
        right_wheel_transform.transform.rotation.z = right_wheel_quat[3]
        right_wheel_transform.transform.rotation.w = right_wheel_quat[0]

        self.tf_broadcaster.sendTransform(right_wheel_transform)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdometryNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


