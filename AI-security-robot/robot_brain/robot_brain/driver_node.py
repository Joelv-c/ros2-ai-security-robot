import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configuration Constants
USE_HARDWARE = True 

# Import Hardware Libraries safely
try:
    import gopigo3
    LIBS_AVAILABLE = True
except ImportError:
    LIBS_AVAILABLE = False

class DriverNode(Node):
    """
    ROS 2 Node for Motor Control.
    Subscribes to detection topics and controls the GoPiGo3 hardware.
    """

    def __init__(self):
        super().__init__('driver_node')
        
        self.subscription = self.create_subscription(
            String, 'detection_topic', self.listener_callback, 10)
        
        self.robot = None
        self.hardware_connected = False

        if USE_HARDWARE and LIBS_AVAILABLE:
            try:
                self.robot = gopigo3.GoPiGo3()
                self.get_logger().info("GoPiGo3 hardware connected successfully.")
                self.hardware_connected = True
            except Exception as e:
                self.get_logger().error(f"Hardware initialization failed: {e}")
        else:
            self.get_logger().warn("Simulation Mode: No hardware detected.")

    def listener_callback(self, msg):
        """
        Callback function executed when a message is received from the Vision or Keyboard node.
        """
        command = msg.data
        
        if not self.hardware_connected: 
            self.get_logger().info(f"[SIMULATION] Motors executing: {command}")
            return

        # Priority Logic: Safety Stops
        if command == "Bad Guy":
            self.get_logger().info("Safety Alert: Bad Guy detected. Stopping.")
            self.robot.set_led(self.robot.LED_LEFT_EYE, 255, 0, 0) # Red LED
            self.robot.set_motor_dps(self.robot.MOTOR_LEFT + self.robot.MOTOR_RIGHT, 0) # Stop
        
        elif command == "vehicle":
            self.get_logger().info("Safety Alert: Vehicle detected. Stopping.")
            self.robot.set_led(self.robot.LED_LEFT_EYE, 0, 0, 255) # Blue LED
            self.robot.set_motor_dps(self.robot.MOTOR_LEFT + self.robot.MOTOR_RIGHT, 0) # Stop

        # Movement Logic (from Keyboard)
        elif command == "forward":
            self.robot.set_motor_dps(self.robot.MOTOR_LEFT + self.robot.MOTOR_RIGHT, 300)
        
        elif command == "left":
            self.robot.set_motor_dps(self.robot.MOTOR_LEFT, -150)
            self.robot.set_motor_dps(self.robot.MOTOR_RIGHT, 150)
        
        elif command == "right":
            self.robot.set_motor_dps(self.robot.MOTOR_LEFT, 150)
            self.robot.set_motor_dps(self.robot.MOTOR_RIGHT, -150)
        
        elif command == "stop":
            self.robot.stop()
        
        else:
            # Idle / Patrol State (Green LED)
            self.robot.set_led(self.robot.LED_LEFT_EYE, 0, 255, 0)

def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()