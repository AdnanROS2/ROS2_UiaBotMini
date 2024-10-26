import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotPublisher(Node):
    def __init__(self):
        super().__init__('robot_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1  # Eksempel lineær hastighet
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg}"')

def main(args=None):
    rclpy.init(args=args)
    robot_publisher = RobotPublisher()
    rclpy.spin(robot_publisher)
    robot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
