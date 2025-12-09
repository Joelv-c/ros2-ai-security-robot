import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from ultralytics import YOLO
import os

# Configuration Constants
USE_CAMERA = True
TEST_IMAGE_PATH = "/home/joel/robot_project/test_photo.jpg"
MODEL_PATH = "/home/joel/robot_project/best.pt"

class VisionNode(Node):
    """
    ROS 2 Node for Computer Vision using YOLOv8.
    Captures video frames, performs object detection, and publishes detected entities.
    """

    def __init__(self):
        super().__init__('vision_node')
        
        # Initialize Publisher
        self.publisher_ = self.create_publisher(String, 'detection_topic', 10)
        
        # Set processing loop to 10Hz (0.1 seconds)
        self.timer = self.create_timer(0.1, self.detect_objects)
        
        self.get_logger().info(f"Loading YOLO model from: {MODEL_PATH}")
        try:
            self.model = YOLO(MODEL_PATH)
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            exit()
        
        self.cap = None
        if USE_CAMERA:
            self.find_camera()
        else:
            self.get_logger().info(f"Simulation Mode: Watching static file {TEST_IMAGE_PATH}")

    def find_camera(self):
        """
        Scans video indices 0-9 to find a working camera.
        """
        for i in range(10):
            self.get_logger().info(f"Scanning camera index {i}...")
            temp_cap = cv2.VideoCapture(i)
            if temp_cap.isOpened():
                ret, _ = temp_cap.read()
                if ret:
                    self.cap = temp_cap
                    self.get_logger().info(f"Success: Connected to camera index {i}")
                    return
                temp_cap.release()
        self.get_logger().error("No camera found. Please check hardware connection.")

    def detect_objects(self):
        """
        Main loop: Grabs frame, runs inference, draws visualization, and publishes alerts.
        """
        frame = None
        
        # 1. Acquire Frame
        if self.cap:
            ret, frame = self.cap.read()
            if not ret:
                return
        elif os.path.exists(TEST_IMAGE_PATH):
            frame = cv2.imread(TEST_IMAGE_PATH)
        
        if frame is None:
            return

        # 2. Run Inference
        # stream=True is more efficient for video
        results = self.model(frame, stream=True, verbose=False)
        
        msg = String()
        msg.data = "none"

        # 3. Process Results
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                raw_name = self.model.names[class_id]
                confidence = float(box.conf[0])
                
                # Filter low confidence detections
                if confidence < 0.4:
                    continue

                display_name = raw_name
                is_threat = False

                if raw_name == 'person':
                    display_name = "Bad Guy"
                    msg.data = "Bad Guy"
                    is_threat = True
                elif raw_name in ['car', 'truck', 'bus', 'motorcycle']:
                    display_name = "vehicle"
                    msg.data = "vehicle"
                    is_threat = True

                # Visualization: Draw Bounding Boxes
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Color coding (BGR format): Red for Bad Guy, Blue for Vehicle, Green otherwise
                color = (0, 255, 0)
                if display_name == "Bad Guy":
                    color = (0, 0, 255) # Red
                elif display_name == "vehicle":
                    color = (255, 0, 0) # Blue
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                label = f"{display_name} {confidence:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 4. Publish Message
        self.publisher_.publish(msg)
        
        # 5. Update Live View Window
        try:
            cv2.imshow("Robot Vision System", frame)
            cv2.waitKey(1)
        except Exception:
            pass

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()