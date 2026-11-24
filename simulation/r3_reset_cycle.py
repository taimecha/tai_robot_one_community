#!/usr/bin/env python3
"""Reset R3 per-cycle handoff state on r3_amcl_bridge without global re-localization."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('r3_reset_cycle')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest_status = [None]

    def on_status(msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                latest_status[0] = data
        except (ValueError, TypeError):
            pass

    node.create_subscription(String, '/pallet/r3/localization/status', on_status, latched)
    client = node.create_client(Trigger, '/pallet/r3/localization/reset_cycle')
    deadline = time.monotonic() + args.timeout
    try:
        if not client.wait_for_service(timeout_sec=max(0.5, deadline - time.monotonic())):
            raise RuntimeError('Không tìm thấy dịch vụ /pallet/r3/localization/reset_cycle.')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.5, deadline - time.monotonic()))
        if not future.done() or future.result() is None or not future.result().success:
            msg = future.result().message if (future.done() and future.result() is not None) else 'timeout'
            raise RuntimeError(f'Không đặt lại được chu trình định vị: {msg}')
        reply = json.loads(future.result().message)
        reset_ros_s = float(reply.get('at_ros_s', 0.0))

        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            st = latest_status[0] or {}
            if (st.get('published_ros_s', 0.0) >= reset_ros_s
                    and st.get('navigation_monitor') is True
                    and st.get('trial_handoff') is None
                    and st.get('delivery_handoff') is None
                    and not st.get('localization_fault')
                    and st.get('ready') is True):
                print(json.dumps({'status': 'CYCLE_LOCALIZATION_READY',
                                  'run_id': st.get('run_id'),
                                  'at_ros_s': st.get('published_ros_s')}), flush=True)
                return
        st = latest_status[0] or {}
        raise RuntimeError(f'Hết thời gian chờ AMCL sẵn sàng cho chu trình mới: {st.get("reason", "chưa có trạng thái")}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
