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
            self.configs = yaml.safe_load(file)['configs']

    def publish_goal(self, config_name):
        if config_name in self.configs:
            config = self.configs[config_name]
            msg = Goal()
            msg.x_data = config['x_data']
            msg.y_data = config['y_data']
            msg.yaw = config['yaw']
            self.pub_new_goal.publish(msg)
            self.get_logger().info(f'Publishing: x={msg.x_data}, y={msg.y_data}, yaw={msg.yaw}')
        else:
            self.get_logger().error(f"Config '{config_name}' not found in config.yaml")

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()

    try:
        while rclpy.ok():
            print('Available configs:')
            for config in test_publisher.configs:
                test_publisher.get_logger().info(f" - {config}")

            user_input = input("Enter the config name (or 'exit' to quit): ")
            if user_input.lower() == 'exit':
                break

            test_publisher.publish_goal(user_input)
    except KeyboardInterrupt:
        pass

    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()