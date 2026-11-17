#!/usr/bin/env python3
"""Start a new pallet recognition or pause pallet-only tracking; no motion."""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


def request(node, topic, enabled, wait_timeout):
    client = node.create_client(SetBool, topic)
    if not client.wait_for_service(timeout_sec=wait_timeout):
        raise RuntimeError(f'missing service {topic}')
    future = client.call_async(SetBool.Request(data=enabled))
    rclpy.spin_until_future_complete(node, future, timeout_sec=wait_timeout)
    if not future.done():
        client.remove_pending_request(future)
        raise RuntimeError(f'{topic} did not answer within {wait_timeout:g} s')
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f'{topic} refused: {response.message if response else "empty response"}')
    return response.message


def start_recognition(node, wait_timeout):
    """Confirm a new session, including when its service response is lost."""
    states = []
    def receive(message):
        try:
            state = json.loads(message.data)
            if isinstance(state, dict):
                states[:] = [state]
        except (ValueError, TypeError):
            pass
    subscription = node.create_subscription(String, '/pallet/recognition/status', receive,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(Trigger, '/pallet/r3/start_recognition')
    future = None
    try:
        if not client.wait_for_service(timeout_sec=wait_timeout):
            raise RuntimeError('YOLO start service unavailable; check perception.log')
        # Consume the previous latched state before sending the only start request.
        rclpy.spin_once(node, timeout_sec=0.)
        previous_session = (states[-1].get('session', 0) if states else 0)
        sent_ros = node.get_clock().now().nanoseconds*1e-9
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+min(wait_timeout, 3.)
        confirmed = None
        confirmed_wall = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if future.done():
                response = future.result()
                if response is None or not response.success:
                    raise RuntimeError('YOLO refused recognition: '
                                       + (response.message if response else 'empty response'))
                return 'Đã bắt đầu phiên YOLO mới: ' + response.message
            state = states[-1] if states else {}
            if (state.get('session', 0) > previous_session and state.get('enabled') is True
                    and state.get('started_ros_s', 0.) >= sent_ros-.05
                    and state.get('published_ros_s', 0.) >= sent_ros):
                if confirmed_wall is None:
                    confirmed, confirmed_wall = state, time.monotonic()
                # The status publish precedes the service response. Give the
                # response time to arrive before disposing of its client.
                if time.monotonic()-confirmed_wall >= .5:
                    return f'Đã xác nhận phiên YOLO {confirmed["session"]} qua trạng thái trực tiếp.'
        if confirmed is not None:
            return f'Đã xác nhận phiên YOLO {confirmed["session"]} qua trạng thái trực tiếp.'
        raise RuntimeError('YOLO did not confirm a fresh recognition session within 3 seconds')
    finally:
        if future is not None and not future.done():
            client.remove_pending_request(future)
        node.destroy_client(client)
        node.destroy_subscription(subscription)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('recognize', 'pause', 'resume'))
    parser.add_argument('--wait-timeout', type=float, default=3.)
    args = parser.parse_args()
    if not math.isfinite(args.wait_timeout) or args.wait_timeout <= 0:
        parser.error('wait timeout must be finite and positive')
    rclpy.init()
    node = Node('r3_perception_switch')
    try:
        if args.action == 'recognize':
            request(node, '/pallet/r3/pause_target_tracking', False, args.wait_timeout)
            print(start_recognition(node, args.wait_timeout), flush=True)
        else:
            print(request(node, '/pallet/r3/pause_target_tracking',
                          args.action == 'pause', args.wait_timeout), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
