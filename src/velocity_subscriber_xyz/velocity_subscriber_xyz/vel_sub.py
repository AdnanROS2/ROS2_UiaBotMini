

import serial
import struct
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')

        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Publisher for /joint_states
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Wheel velocity and position variables
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0

        # Serial connection setup
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=0.1
            )
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            exit(1)

        # Timer-based loop every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_time = self.get_clock().now()

    def listener_callback(self, msg):
        """Handles incoming velocity commands from /cmd_vel."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f'Received cmd_vel - linear.x: {self.linear_velocity}, angular.z: {self.angular_velocity}')
        self.send_values(self.linear_velocity, self.angular_velocity)

    def timer_callback(self):
        """Timer-based callback to read serial data."""
        self.read_serial_data()

    def send_values(self, linear_velocity, angular_velocity):
        try:
            # Pack and send the velocity data via serial
            value_buff = struct.pack('=BBff', 36, 36, linear_velocity, angular_velocity)
            self.ser.write(value_buff)
            self.get_logger().info(f"Sent values to serial: linear_velocity={linear_velocity}, angular_velocity={angular_velocity}")
            sleep(0.01)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send values: {e}")

    def read_serial_data(self):
        try:
            # Read incoming serial data (40 bytes)
            input_data = self.ser.read(40)

            if len(input_data) < 40:
                self.get_logger().info("Incomplete data received")
                return

            self.ser.reset_input_buffer()

            # Find the header position
            headerPos = input_data.find(b'\x24\x24')

            if headerPos == -1:
                self.get_logger().info("Header not found in received data")
                return

            # Extract the float data (4 + 4 bytes)
            inData = input_data[(headerPos + 2):(headerPos + 10)]
            if len(inData) != 8:
                self.get_logger().info("Incomplete float data received")
                return

            # Unpack the left and right wheel velocities
            receivedValues = struct.unpack('=ff', inData)
            v_left = receivedValues[0]
            v_right = receivedValues[1]

            # Log the received values
            self.get_logger().info(f"Received from serial: left wheel={v_left}, right wheel={v_right}")

            # Publish the joint states
            self.publish_joint_states(v_left, v_right)

        except struct.error as e:
            self.get_logger().error(f"Error unpacking data: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def publish_joint_states(self, v_left, v_right):
        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Integrate the velocities to get positions (Euler method)
        self.left_wheel_position += v_left * dt
        self.right_wheel_position += v_right * dt

        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']

        # Update positions by integrating velocity
        joint_state.position = [self.left_wheel_position, self.right_wheel_position]

        # Publish wheel velocities
        joint_state.velocity = [v_left, v_right]

        # Log the publishing of joint states
        self.get_logger().info(f'Publishing joint states - left wheel pos: {self.left_wheel_position}, right wheel pos: {self.right_wheel_position}, velocities: {v_left}, {v_right}')
        
        # Publish the JointState message
        self.joint_state_publisher.publish(joint_state)

    def close_serial(self):
        # Clean up the serial connection before shutdown
        self.ser.flush()
        self.ser.close()
        self.get_logger().info("Serial port closed.")

def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    try:
        # ROS2 spinning
        while rclpy.ok():
            rclpy.spin_once(velocity_subscriber)
            velocity_subscriber.read_serial_data()
            sleep(0.1)

    except KeyboardInterrupt:
        velocity_subscriber.get_logger().info("Shutdown requested by user...")

    finally:
        velocity_subscriber.close_serial()
        velocity_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




