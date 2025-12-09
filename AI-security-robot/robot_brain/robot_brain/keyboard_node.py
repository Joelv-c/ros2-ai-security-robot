import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardNode(Node):
    """
    ROS 2 Node for Manual Input.
    Reads keyboard strokes (WASD) and publishes commands to the driver node.
    """

    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(String, 'detection_topic', 10)
        self.get_logger().info("Keyboard Control Active.")
        self.get_logger().info("Controls: W (Forward), A (Left), S (Stop), D (Right), Q (Quit)")

    def get_key(self):
        """
        Reads a single keypress from stdin without blocking.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        """
        Main execution loop for capturing input.
        """
        while True:
            key = self.get_key()
            msg = String()
            
            if key == 'w':
                msg.data = "forward"
            elif key == 's':
                msg.data = "stop"
            elif key == 'a':
                msg.data = "left"
            elif key == 'd':
                msg.data = "right"
            elif key == 'q':
                break
            
            # Publish only if a valid key command was assigned
            if msg.data: 
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()