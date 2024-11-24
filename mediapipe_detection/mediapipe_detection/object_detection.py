import os
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class ObjectDetectionNode(Node):

    buzzer = False 
    timestamp = 0

    # Reference: https://github.com/google-ai-edge/mediapipe-samples/blob/main/examples/object_detection/raspberry_pi/detect.py
    def __init__(self, name): 
        super().__init__(name)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 1)
        self.buzzer_pub = self.create_publisher(Bool, "Buzzer", 1)
        # Only Objectron is contained within mp lib        
        self.mp_drawing = mp.solutions.drawing_utils

        package_share_directory = get_package_share_directory('mediapipe_detection')
        model_path = os.path.join(package_share_directory, "models/efficientdet.tflite")
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found at {model_path}")
        self.base_options = python.BaseOptions(model_asset_path=model_path)
        # self.save_result might not work
        self.options = vision.ObjectDetectorOptions(
            running_mode=vision.RunningMode.LIVE_STREAM, 
            base_options=self.base_options, 
            max_results=1, 
            score_threshold=0.3, 
            category_allowlist=["person", "backpack"], 
            result_callback=self.save_result
            )
        self.detector = vision.ObjectDetector.create_from_options(self.options)
        self.detection_result_list = []
        self.detection_frame = None 
    
    def save_result(self, result):
        self.get_logger().info('save result')
        self.detection_result_list.append(result)

    def image_callback(self, msg):            
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # resized_rgb_frame = cv2.resize(rgb_frame, (320, 320))
        mp_rgb_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        self.get_logger().info('Detecting...')
        self.detector.detect_async(mp_rgb_frame, self.timestamp)
        self.timestamp += 1

        if self.detection_result_list: 
            current_frame = self.visualize(current_frame, self.detection_result_list[0])
            self.detection_frame = current_frame
            self.get_logger().info('Backpack detected!')
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

    # Reference: https://github.com/google-ai-edge/mediapipe-samples/blob/main/examples/object_detection/python/object_detector_live_stream/utils.py
    def visualize(
        image,
        detection_result
    ) -> np.ndarray:
        MARGIN = 10  # pixels
        ROW_SIZE = 10  # pixels
        FONT_SIZE = 1
        FONT_THICKNESS = 1
        TEXT_COLOR = (255, 0, 0)  # red
        for detection in detection_result.detections:
            # Draw bounding_box
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)

            # Draw label and score
            category = detection.categories[0]
            category_name = category.category_name
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (MARGIN + bbox.origin_x,
                            MARGIN + ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode("object_detection_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
