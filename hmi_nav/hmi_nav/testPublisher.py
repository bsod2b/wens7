import os
import rclpy
from rclpy.node import Node
from yahboomcar_msgs.msg import Goal
from ament_index_python.packages import get_package_share_directory
import yaml

CONFIG_PATH = 'config.yml'
PACKAGE_NAME = 'hmi_nav'

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.pub_new_goal = self.create_publisher(Goal, 'nav/new_goal', 1)

        package_share_directory = get_package_share_directory(PACKAGE_NAME)
        absolute_config_path = os.path.join(package_share_directory, CONFIG_PATH)
        with open(absolute_config_path, 'r') as file:
            configs = yaml.safe_load(file)

        valid_input = False

        while not valid_input:
            print('Available configs:')
            for config in configs['configs']:
                self.get_logger().info(f" - {config}")

            user_input = input("Enter the config name: ")
            if user_input in configs['configs']:
                self.config = configs['configs'][user_input]
                self.get_logger().info(f"Loaded config '{user_input}'")
                valid_input = True
            else:
                self.get_logger().error(f"Config '{user_input}' not found in config.yaml")

        msg = Goal()
        msg.x_data = self.config['x_data']
        msg.y_data = self.config['y_data']
        msg.yaw = self.config['yaw']
        self.pub_new_goal.publish(msg)
        self.get_logger().info(f'Publishing: x={msg.xData}, y={msg.yData}, yaw={msg.yaw}')

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    rclpy.spin(test_publisher)
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()