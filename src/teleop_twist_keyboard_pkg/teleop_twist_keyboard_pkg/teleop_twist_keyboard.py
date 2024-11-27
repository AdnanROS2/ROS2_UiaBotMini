import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your Robot!
---------------------------
Moving around:
   w
a  d  s     
   x

q/z : increase/decrease max speeds by 10%
e/c : increase/decrease turn speed by 10%
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    's': (-1, 0, 0, 0),
    'd': (0, 0, 0, -1),
}

# Definer hastighets- og rotasjonsbindings for kontrollene
speedBindings = {
    'q': (1.1, 1.0),  # Øker begge hastigheter
    'z': (0.9, 1.0),  # Reduserer begge hastigheter
    'e': (1.0, 1.1),  # Øker kun rotasjonshastighet
    'c': (1.0, 0.9),  # Reduserer kun rotasjonshastighet
}

# Lagre terminalinnstillinger
settings = termios.tcgetattr(sys.stdin)

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.speed = 0.5
        self.turn = 1.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print(msg)
        while True:
            key = self.get_key()
            if key in moveBindings.keys():
                x, y, z, th = moveBindings[key]
            elif key in speedBindings.keys():
                self.speed *= speedBindings[key][0]
                self.turn *= speedBindings[key][1]
                print(f"current speed: {self.speed}, turn: {self.turn}")
                continue
            else:
                x, y, z, th = (0, 0, 0, 0)
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = x * self.speed
            twist.linear.y = y * self.speed
            twist.linear.z = z * self.speed
            twist.angular.z = th * self.turn
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistKeyboard()
    try:
        node.run()
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
