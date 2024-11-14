import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class IMUBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_broadcaster')
        
        # Initialize the TransformBroadcaster for dynamic transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize the StaticTransformBroadcaster for static transforms
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribe to the IMU data topic from the BNO055
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Broadcast the static transform between base_link and imu_link once
        self.broadcast_static_tf()

    def imu_callback(self, msg):
        # Dynamic transform based on the IMU's orientation
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation  # Use the orientation from the IMU message

        # Broadcast the dynamic transform
        self.tf_broadcaster.sendTransform(t)

    def broadcast_static_tf(self):
        # Static transform between base_link and imu_link
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'imu_link'

        # Set fixed offset of IMU relative to base_link (adjust as needed)
        static_transform.transform.translation.x = 0.1  # example offset
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # No rotation in static tf

        # Broadcast the static transform
        self.static_broadcaster.sendTransform(static_transform)

def main(args=None):
    rclpy.init(args=args)
    node = IMUBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
