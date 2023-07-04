import rclpy
from typing import Generator
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import time
#import matplotlib.pyplot as plt
from ros2_esp32_messages.msg import PinValues

def generator_alternating() -> Generator:
    val = False
    while True:
        yield int(val)
        val = not val

def generator_sine(ampl: float, shift: float) -> Generator:
    time_max = 10
    time_start = time.time()
    while True:
        delta_time = time.time() - time_start
        t = 2*np.pi/time_max * delta_time
        yield ampl * np.sin(t) + shift


class PubNode(Node):
    def __init__(self, vals: list, pin_out: int, gen: Generator):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(PinValues, '/esp32_write_pins', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.vals = vals
        self.pin_out = pin_out
        self.data_gen = gen

    def timer_callback(self):
        msg = PinValues()

        msg.values = np.zeros(40)
        val = next(self.data_gen)
        self.vals.append([time.time(), val])
        msg.values[self.pin_out] = val
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.values)


class SubNode(Node):
    def __init__(self, vals: list, pin_in: int):
        super().__init__('subscriber')
        self.subscriber = self.create_subscription(PinValues, '/esp32_read_pins', self.sub_callback, qos_profile=qos_profile_sensor_data)
        self.vals = vals
        self.pin_in = pin_in

    def sub_callback(self, msg):
        self.vals.append([time.time(), msg.values[self.pin_in]])
        self.get_logger().info('Receiving: "%s"' % msg.values)


def plot_values(vals_out, vals_in) -> None:
    vals_out = np.array(vals_out)
    vals_in = np.array(vals_in)
    plt.figure()
    plt.plot(vals_out[:, 0], vals_out[:, 1], label='Sent')
    plt.plot(vals_in[:, 0], vals_in[:, 1], label='Received')
    plt.legend()
    plt.savefig('test.png')

def save_values(vals_sent, vals_rec):
    vals = np.array([vals_sent, vals_rec], dtype=object)
    with open('src/esp32_demo/data/vals.csv', 'wb') as f:
        np.save(f, vals)

def run_experiment() -> None:
    vals_out, vals_in = [], []
    rclpy.init()
    try:
        pub = PubNode(vals_out, 25, generator_sine(100, 100))
        sub = SubNode(vals_in, 33)

        executor = SingleThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)

        try:
            #executor.spin()
            start = time.time()
            while True:
                executor.spin_once()
                if time.time() - start > 10:
                    break
        finally:
            executor.shutdown()
            pub.destroy_node()
            sub.destroy_node()

    finally:
        rclpy.shutdown()

    print(vals_out)
    print(vals_in)
    save_values(vals_out, vals_in)

def main():
    run_experiment()

if __name__ == '__main__':
    main() 
