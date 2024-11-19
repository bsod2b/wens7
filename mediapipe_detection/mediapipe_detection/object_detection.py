import rclpy
from rclpy.node import Node   
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import mediapipe as mp
import time
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from utils import visualize

class ObjectDetectionNode(Node):

    buzzer = False 

    # Reference: https://github.com/google-ai-edge/mediapipe-samples/blob/main/examples/object_detection/raspberry_pi/detect.py
    def __init__(self, name): 
        super().__init__(name)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 1)
        self.buzzer_pub = self.create_publisher(Bool, "Buzzer", 1)
        # TODO: Download alternative pretrained model
        # Only Objectron is contained within mp lib        
        self.mp_drawing = mp.solutions.drawing_utils
        self.base_options = python.BaseOptions(model_asset_path= "PATH/TO/MODEL")
        # self.save_result might not work
        self.options = vision.ObjectDetectorOptions(running_mode=vision.RunningMode.LIVE_STREAM, base_options=self.base_options, max_results=1, score_threshold=0.5, category_allowlist=["backpack"], result_callback=self.save_result)
        self.detector = vision.ObjectDetector.create_from_options(self.options)
        self.detection_result_list = []
        self.detection_frame = None 
    
    def save_result(self, result):
        self.detection_result_list.append(result)

    def image_callback(self, msg):            
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_rgb_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        self.detector.detect_async(mp_rgb_frame, time.time_ns())

        if self.detection_result_list: 
            current_frame = visualize(current_frame, self.detection_result_list[0])
            self.detection_frame = current_frame
            self.get_logger().info('Shoe detected!')
            self.publish_message(True)
            self.timer = self.create_timer(1.0, self.publish_false)
            self.detection_result_list.clear()
        
        if self.detection_frame is not None:
            cv2.imshow('Object Detection', frame)
            cv2.waitKey(1)

    def publish_message(self, value):
        msg = Bool()
        msg.data = value
        self.buzzer_pub.publish(msg)

    def publish_false(self):
        self.publish_message(False)
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode("object_detection_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
