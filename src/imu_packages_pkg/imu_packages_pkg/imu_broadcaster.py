import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class IMUBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_broadcaster')
        
        # Initialize dynamic and static TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribe to the IMU data topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Broadcast the static transform between base_link and imu_link once
        self.broadcast_static_tf()

    def imu_callback(self, msg):
        # Create a TransformStamped message based on IMU data
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        
        # Set translation as zero (assuming the IMU is mounted at the robot’s center)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Use the IMU orientation to set the rotation of imu_link relative to base_link
        t.transform.rotation = msg.orientation  # Use the orientation from the IMU message

        # Broadcast the dynamic transform
        self.tf_broadcaster.sendTransform(t)

    def broadcast_static_tf(self):
        # Static transform between base_link and imu_link
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'imu_link'

        # Example static offset (adjust if needed for IMU's actual mounting position)
        static_transform.transform.translation.x = 0.1
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # No rotation for static transform

        # Broadcast the static transform once
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Broadcasted static transform between base_link and imu_link")

def main(args=None):
    rclpy.init(args=args)
    imu_broadcaster = IMUBroadcaster()
    try:
        rclpy.spin(imu_broadcaster)
    except KeyboardInterrupt:
        imu_broadcaster.get_logger().info("IMU Broadcaster shutting down.")
    finally:
        imu_broadcaster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

