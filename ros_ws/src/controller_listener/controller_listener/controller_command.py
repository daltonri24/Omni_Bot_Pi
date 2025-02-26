import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy


class ControllerCommand(Node):


    def __init__(self):
        super().__init__('controller_command')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            1)

        timer_period = 0.1 # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cmd_vel = Twist()

    def timer_callback(self):
        self.publisher_.publish(self.cmd_vel)
        #self.get_logger().info('Publishing:')

    def listener_callback(self, msg):
        self.cmd_vel.linear.x = 1/4 * msg.axes[1] + 1/4 * msg.axes[7]
        self.cmd_vel.linear.y = -1/4 * msg.axes[0] - 1/4 * msg.axes[6]
        self.cmd_vel.angular.z = -3.14 * 1/4 * msg.axes[3]


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ControllerCommand()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()