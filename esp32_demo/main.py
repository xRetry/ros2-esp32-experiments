import rclpy
from typing import Generator
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import time
import json
#import matplotlib.pyplot as plt
from ros2_esp32_messages.msg import PinValues
from ros2_esp32_messages.srv import SetConfig


def generator_alternating() -> Generator:
    val = False
    while True:
        yield int(val)
        val = not val

def generator_const(val: float) -> Generator:
    while True:
        yield val

def generator_sine(ampl: float, shift: float) -> Generator:
    time_max = 10
    time_start = time.time()
    while True:
        delta_time = time.time() - time_start
        t = 2*np.pi/time_max * delta_time
        yield ampl * np.sin(t) + shift

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service')
        self.cli = self.create_client(SetConfig, '/esp32_set_config')
        while not self.cli.wait_for_service(timeout_sec=5.):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetConfig.Request()

    def set_pins(self, pin_nums: list, pin_modes: list):
        self.req.read_only = False;
        self.req.pin_modes = np.zeros(40, dtype=np.uint8)
        for pin_num, pin_mode in zip(pin_nums, pin_modes):
            self.req.pin_modes[pin_num] = pin_mode

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        resp = self.future.result()
        for p in pin_nums:
            print('Mode:', resp.pin_modes[p])
            print('Error:', resp.pin_errors[p])

class PubNode(Node):
    def __init__(self, vals: list, pin_out: int, ampl: float, shift: float):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(PinValues, '/esp32_write_pins', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.vals = vals
        self.pin_out = pin_out
        self.data_gen = generator_sine(ampl, shift)
        #self.data_gen = generator_const(100)
        #self.data_gen = generator_const(0)
        self.ampl = ampl
        self.shift = shift

    def timer_callback(self):
        msg = PinValues()

        msg.values = np.zeros(40)

        val = next(self.data_gen)
        self.vals.append([time.time(), val])

        msg.values[self.pin_out] = val
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing...')


class SubNode(Node):
    def __init__(self, vals: list, pin_in: int):
        super().__init__('subscriber')
        self.subscriber = self.create_subscription(PinValues, '/esp32_read_pins', self.sub_callback, qos_profile=qos_profile_sensor_data)
        self.vals = vals
        self.pin_in = pin_in
        self.cali_max = 1430
        self.cali_min = 0 

    def sub_callback(self, msg):
        self.vals.append([time.time(), msg.values[self.pin_in]])
        print(msg.values[self.pin_in])
        #self.get_logger().info('Receiving...')


def plot_values(vals_out, vals_in) -> None:
    vals_out = np.array(vals_out)
    vals_in = np.array(vals_in)
    plt.figure()
    plt.plot(vals_out[:, 0], vals_out[:, 1], label='Sent')
    plt.plot(vals_in[:, 0], vals_in[:, 1], label='Received')
    plt.legend()
    plt.savefig('test.png')

def save_values(vals_sent, vals_rec, ampl, shift):
    vals = dict(
        sent=vals_sent,
        received=vals_rec,
        ampl=ampl,
        shift=shift
    )
    with open('src/esp32_demo/data/vals.json', 'w') as f:
        json.dump(vals, f)

def run_experiment() -> None:
    vals_out, vals_in = [], []
    ampl = 100
    shift = 100

    rclpy.init()
    try:
        srv = ServiceNode()
        srv.set_pins([25, 33], [4, 3])
        pub = PubNode(vals_out, 25, ampl, shift)
        sub = SubNode(vals_in, 33)

        executor = SingleThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)

        try:
            start = time.time()
            while True:
                executor.spin_once()
                if time.time() - start > 10:
                    break
        finally:
            executor.shutdown()
            srv.destroy_node()
            pub.destroy_node()
            sub.destroy_node()

    finally:
        rclpy.shutdown()

    save_values(vals_out, vals_in, ampl, shift)

def main():
    run_experiment()

if __name__ == '__main__':
    main() 
