import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Bringup(Node):

    def __init__(self):
        super().__init__('assistbot_bringup')
        self.subscription = self.create_subscription(
            String,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    bringup = Bringup()
    rclpy.spin(bringup)

    bringup.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()