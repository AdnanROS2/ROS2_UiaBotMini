import time
import board
import busio
import adafruit_bno055
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
from transforms3d.euler import euler2quat  # Updated import

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Setup I2C connection and BNO055 IMU sensor
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu_sensor = adafruit_bno055.BNO055_I2C(i2c)
            self.get_logger().info("IMU sensor initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize IMU sensor: {e}")
            return
        
        # Publisher for IMU data
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # Set the publish rate to 10 Hz
        self.timer = self.create_timer(0.1, self.publish_imu_data)

    def publish_imu_data(self):
        # Read orientation from BNO055 sensor
        orientation = self.imu_sensor.euler  # Returns (heading, roll, pitch) in degrees
        if orientation is None:
            self.get_logger().warning("No IMU orientation data available")
            return
        
        # Convert Euler angles (heading, roll, pitch) from degrees to radians, then to Quaternion
        try:
            q = euler2quat(
                math.radians(orientation[1]),  # roll
                math.radians(orientation[2]),  # pitch
                math.radians(orientation[0])   # yaw (heading)
            )
        except TypeError:
            self.get_logger().warning("IMU orientation data is incomplete")
            return

        # Populate Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Set orientation
        imu_msg.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])

        # Read linear acceleration and angular velocity, if available
        accel = self.imu_sensor.linear_acceleration
        if accel is not None:
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]

        gyro = self.imu_sensor.gyro
        if gyro is not None:
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]

        # Publish the IMU message
        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("IMU Publisher shutting down.")
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

