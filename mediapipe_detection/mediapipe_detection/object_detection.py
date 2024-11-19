import rclpy
from rclpy.node import Node   
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import mediapipe as mp

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
            model_name='Shoe'
        )
        self.mp_drawing = mp.solutions.drawing_utils

    def image_callback(self, msg):                         
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.objectron.process(rgb_frame)
        if results.detected_objects:
            buzzer_msg = Bool()
            buzzer_msg.data = True
            self.buzzer_pub.publish(buzzer_msg)

            cv2.show('Object Detection', frame)
            cv2.waitKey(1)
            for detected_object in results.detected_objects:
                self.mp_drawing.draw_landmarks(
                    frame,
                    detected_object.landmarks_2d,
                    self.mp_objectron.BOX_CONNECTIONS
                )
                self.get_logger().info('Shoe detected!')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode("object_detection_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
