import rclpy
from rclpy.node import Node

from ros2_esp32_interfaces.msg import PinValues


class ValuesPubSub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PinValues, '/esp32_write_pins', 10)
        self.subscriber = self.create_subscription(PinValues, 'esp32_read_pins', self.sub_callback, 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = PinValues()
        msg.values = [1. for _ in range(36)]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.values)

    def sub_callback(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg.values)


def main(args=None):
    rclpy.init(args=args)
    pub_sub = ValuesPubSub()
    rclpy.spin(pub_sub)
    pub_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 
