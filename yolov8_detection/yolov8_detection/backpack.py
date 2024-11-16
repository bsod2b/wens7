import rclpy
from rclpy.node import Node   
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class BackpackDetectionNode(Node):
    def __init__(self, name): 
        super().__init__(name)
        self.model = YOLO('../models/yolov8n.pt')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, "/yolov8/annotated_image", 10)
        self.BACKPACK_CLASS_INDEX = 24

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform YOLOv8 detection (filtering for "backpack")
            results = self.model.predict(frame, conf=0.5, classes=[self.BACKPACK_CLASS_INDEX])
            annotated_frame = results[0].plot()  # Annotate frame with detections

            # Display or process annotated frame
            cv2.imshow("YOLOv8 Detections", annotated_frame)
            cv2.waitKey(1)
            
            # Optional: Publish the annotated image (or detection info) back to ROS
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.detection_pub.publish(annotated_msg)

        except Exception as e:
            print(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BackpackDetectionNode("backpack_detection_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()