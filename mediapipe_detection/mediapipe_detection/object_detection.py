import rclpy
from rclpy.node import Node   
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import mediapipe as mp

BaseOptions = mp.tasks.BaseOptions
DetectionResult = mp.tasks.components.containers.detections.DetectionResult
ObjectDetector = mp.tasks.vision.ObjectDetector
ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
VisionRunningMode = mp.tasks.vision.RunningMode

class ObjectDetectionNode(Node):
    def __init__(self, name): 
        super().__init__(name)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 1)
        self.buzzer_pub = self.create_publisher(Bool, "/buzzer", 10)
        self.mp_objectron = mp.solutions.objectron
        self.objectron = self.mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=1,
            min_detection_confidence=0.5,
            model_name='Bag'
        )
        self.mp_drawing = mp.solutions.drawing_utils

    def image_callback(self, msg):        
        # options = ObjectDetectorOptions(
        #    base_options=BaseOptions(model_asset_path='/path/to/model.tflite'),
        #    running_mode=VisionRunningMode.LIVE_STREAM,
        #    max_results=5,
        #    result_callback=callback,
        #    category_allowlist=['backpack'])
        # with ObjectDetector.create_from_options(options) as detector:
                   
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = self.objectron.process(rgb_frame)
            if results.detected_objects:
                buzzer_msg = Bool()
                buzzer_msg.data = True
                self.buzzer_pub.publish(buzzer_msg)
                for detected_object in results.detected_objects:
                    self.mp_drawing.draw_landmarks(
                        frame,
                        detected_object.landmarks_2d,
                        self.mp_objectron.BOX_CONNECTIONS
                    )
                    self.get_logger().info('Backpack detected!')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode("object_detection_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
