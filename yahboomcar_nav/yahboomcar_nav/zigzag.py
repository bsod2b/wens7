import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SystematicDriver(Node):
    def __init__(self):
        super().__init__('systematic_driver')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Bool, '/camera/object_detection', self.image_callback, 10)
        self.timer = self.create_timer(0.1, self.drive)

        self.obstacle_detected = False
        self.turning = False
        self.turn_step = 0

    def lidar_callback(self, scan_msg):
        # Check for obstacles within 0.5 meters in front
        front_distances = scan_msg.ranges[0:30] + scan_msg.ranges[-30:]
        self.obstacle_detected = any(d < 0.5 for d in front_distances if d > 0)

    def image_callback(self, msg):
        self.timer.destroy()
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def drive(self):
        msg = Twist()

        if self.turning:
            # Turn 90 degrees
            self.get_logger().info('Turning...')
            msg.angular.z = 0.5
            self.turn_step += 1
            if self.turn_step > 20:  # Example turning duration
                self.turning = False
                self.turn_step = 0
        elif self.obstacle_detected:
            # Obstacle detected: stop and initiate a turn
            msg.linear.x = 0.1
            msg.linear.y = 0.1
            self.turning = True
        else:
            msg.linear.x = 0.6  # Move forward
            msg.linear.y = 0.6   

        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystematicDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
