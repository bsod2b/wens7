import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from yahboomcar_msgs.msg import Goal

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sub_newGoal = self.create_subscription(Goal, "nav/new_goal", self.send_goal, 1)

    def send_goal(self, msg):
        if not isinstance(msg, Goal): return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = msg.x_data
        goal_msg.pose.pose.position.y = msg.y_data
        goal_msg.pose.pose.orientation.z = msg.yaw  # Use a quaternion here for real navigation

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal completed with result: {result}')

def main(args=None):
    rclpy.init(args=args)
    goal_node = GoalSender()
    rclpy.spin(goal_node)

if __name__ == '__main__':
    main()
