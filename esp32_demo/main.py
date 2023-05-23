import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from ros2_esp32_interfaces.msg import PinValues


class PubNode(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(PinValues, '/esp32_write_pins', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = PinValues()
        msg.values = [1. for _ in range(36)]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.values)


class SubNode(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscriber = self.create_subscription(PinValues, '/esp32_read_pins', self.sub_callback, qos_profile=qos_profile_sensor_data)

    def sub_callback(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg.values)


def main(args=None):
    rclpy.init(args=args)
    try:
        pub = PubNode()
        sub = SubNode()

        executor = SingleThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            pub.destroy_node()
            sub.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
