import rclpy
from rclpy.node import Node
from yahboomcar_msgs.msg import Goal
import yaml

CONFIG_PATH = 'config.yaml'

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.pub_new_goal = self.create_publisher(Goal, 'nav/new_goal', 1)

        with open(CONFIG_PATH, 'r') as file:
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
        msg.xData = self.config['xData']
        msg.yData = self.config['yData']
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