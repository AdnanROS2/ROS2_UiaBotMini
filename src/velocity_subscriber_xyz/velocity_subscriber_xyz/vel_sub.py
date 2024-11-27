import serial
import struct
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')

        # ROS 2 callback groups for concurrency
        self.serial_callback_group = MutuallyExclusiveCallbackGroup()
        self.cmd_vel_callback_group = MutuallyExclusiveCallbackGroup()

        # ROS 2 subscriptions and publishers
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10, callback_group=self.cmd_vel_callback_group)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Wheel state variables
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0

        # Attempt to open serial port
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.05)  # Reduce timeout for faster reads
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None

        # Timer for reading serial data
        self.timer = self.create_timer(0.05, self.read_serial_data, callback_group=self.serial_callback_group)
        self.last_time = self.get_clock().now()

    def listener_callback(self, msg):
        """Handle received cmd_vel messages."""
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        self.send_values(linear_velocity, angular_velocity)

    def send_values(self, linear_velocity, angular_velocity):
        """Send velocity commands to the serial device."""
        if self.ser:
            try:
                value_buff = struct.pack('=BBff', 36, 36, linear_velocity, angular_velocity)
                self.ser.write(value_buff)
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send values: {e}")

    def read_serial_data(self):
        """Read wheel velocities from the serial device."""
        if self.ser:
            try:
                input_data = self.ser.read(40)
                if len(input_data) < 40:
                    self.get_logger().warning("Incomplete data received from serial.")
                    return

                # Check for header
                header_pos = input_data.find(b'\x24\x24')
                if header_pos == -1:
                    self.get_logger().warning("Header not found in serial data.")
                    return

                # Extract wheel velocity data
                in_data = input_data[header_pos + 2:header_pos + 10]
                if len(in_data) == 8:
                    v_left, v_right = struct.unpack('=ff', in_data)

                    # Validate velocities
                    if not self.is_valid_velocity(v_left) or not self.is_valid_velocity(v_right):
                        self.get_logger().error("Invalid wheel velocities detected. Skipping update.")
                        return

                    # Publish valid joint states
                    self.publish_joint_states(v_left, v_right)
            except (struct.error, serial.SerialException) as e:
                self.get_logger().error(f"Error reading from serial: {e}")

    def is_valid_velocity(self, velocity):
        """Check if the velocity is valid (not NaN, infinite, or unreasonably large)."""
        return not (math.isnan(velocity) or math.isinf(velocity) or abs(velocity) > 100.0)

    def publish_joint_states(self, v_left, v_right):
        """Publish joint state messages based on wheel velocities."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0:  # Avoid division by zero or negative time
            return

        self.last_time = current_time

        # Update wheel positions based on velocities
        self.left_wheel_position += v_left * dt
        self.right_wheel_position += v_right * dt

        # Create and publish joint state message
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_position * 0.1, self.right_wheel_position * 0.1]
        joint_state.velocity = [v_left, v_right]

        self.joint_state_publisher.publish(joint_state)

    def close_serial(self):
        """Close the serial port if open."""
        if self.ser:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    velocity_subscriber = VelocitySubscriber()
    try:
        rclpy.spin(velocity_subscriber)
    except KeyboardInterrupt:
        velocity_subscriber.get_logger().info("Shutdown requested by user...")
    finally:
        velocity_subscriber.close_serial()
        velocity_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

