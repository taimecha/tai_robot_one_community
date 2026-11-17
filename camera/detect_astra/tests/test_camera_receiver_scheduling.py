"""ROS regression checks; run on an isolated domain without a camera/robot."""
from pathlib import Path
import multiprocessing
import sys
import threading
import time
from types import SimpleNamespace
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from astra_rgbd_stream import CameraReceiver


def publish_test_rgb(stop):
    # Camera delivery is a separate process on the robot. A Python publisher
    # thread in the executor process introduces unrelated GIL contention.
    rclpy.init(domain_id=174)
    node = Node('scheduling_test_rgb_publisher')
    publisher = node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
    try:
        while not stop.is_set():
            msg = Image()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.width = msg.height = msg.step = 8
            msg.encoding = 'mono8'
            msg.data = bytes(64)
            publisher.publish(msg)
            stop.wait(.025)
    finally:
        node.destroy_node()
        rclpy.shutdown()


class SlowGraphReceiver(CameraReceiver):
    def __init__(self):
        self.arrivals = []
        self.slow_checks = 0
        super().__init__(SimpleNamespace(max_age_ms=250., max_delta_ms=50.),
                         'camera_receiver_scheduling_test')

    def inspect_sources(self, rgb, depth):
        self.slow_checks += 1
        time.sleep(.65)  # Reproduce synchronous ROS graph delay.
        return super().inspect_sources(rgb, depth)

    def receive(self, stream, msg):
        self.arrivals.append(time.monotonic())
        super().receive(stream, msg)


class CameraReceiverSchedulingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=174)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def measure_gap(self, executor):
        receiver = SlowGraphReceiver()
        executor.add_node(receiver)
        context = multiprocessing.get_context('spawn')
        stop = context.Event()
        spin = threading.Thread(target=executor.spin)
        feed = context.Process(target=publish_test_rgb, args=(stop,))
        try:
            spin.start()
            feed.start()
            deadline = time.monotonic()+8.
            while not receiver.arrivals and time.monotonic() < deadline:
                stop.wait(.01)
            self.assertTrue(receiver.arrivals, 'test RGB publisher did not connect')
            stop.wait(3.3)
        finally:
            stop.set()
            feed.join(timeout=2.)
            if feed.is_alive():
                feed.terminate()
                feed.join(timeout=2.)
            executor.shutdown(timeout_sec=3.)
            spin.join(timeout=2.)
            receiver.destroy_node()
        self.assertGreaterEqual(receiver.slow_checks, 2)
        self.assertGreater(len(receiver.arrivals), 30)
        return max(b-a for a, b in zip(receiver.arrivals, receiver.arrivals[1:]))

    def test_slow_graph_check_does_not_starve_images(self):
        old_gap = self.measure_gap(SingleThreadedExecutor())
        fixed_gap = self.measure_gap(MultiThreadedExecutor(num_threads=2))
        print(f'Image callback gap under 650-ms graph delay: single={old_gap:.3f}s, independent={fixed_gap:.3f}s')
        self.assertGreater(old_gap, .5)
        # The regression was a complete input blackout exceeding the tracker's
        # 500-ms gap gate. Freshness of each metric pose is checked separately.
        self.assertLess(fixed_gap, .5)
        self.assertGreater(old_gap-fixed_gap, .2)

    def test_in_flight_registration_cannot_override_new_second_writer(self):
        node = CameraReceiver(SimpleNamespace(), 'registration_race_test')
        try:
            node.source_observations['rgb'] = {'first_gid': {}}
            node.set_registration(True, 'earlier single-writer snapshot')
            self.assertTrue(node.registration['verified'])
            # Image callback discovers another writer after the graph snapshot.
            node.source_observations['rgb']['second_gid'] = {}
            node.set_registration(True, 'stale single-writer snapshot')
            self.assertFalse(node.registration['verified'])
            self.assertIn('multiple observed rgb GIDs', node.registration['reason'])
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
