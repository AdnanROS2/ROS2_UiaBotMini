
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

        self.declare_parameter('wheel_separation', 0.2)
        self.declare_parameter('wheel_radius', 0.05)

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

        left_distance = left_wheel_delta * self.wheel_radius
        right_distance = right_wheel_delta * self.wheel_radius
        distance = (left_distance + right_distance) / 2.0
        dtheta = (right_distance - left_distance) / self.wheel_separation

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        self.th += dtheta
        self.x += distance * math.cos(self.th)
        self.y += distance * math.sin(self.th)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
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

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"  # odom is the fixed frame
        t.child_frame_id = "base_link"  # base_link moves relative to odom
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[1]
        t.transform.rotation.y = odom_quat[2]
        t.transform.rotation.z = odom_quat[3]
        t.transform.rotation.w = odom_quat[0]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdometryNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

