#!/usr/bin/env python3
"""Pause Nav2 commands only while a map-transform jump is being checked.

This does not cancel an action or limit ordinary AMCL update intervals. The
original velocity smoother and collision monitor retain braking/obstacle
ownership. A delayed release cannot reopen a held command path.
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


ZERO = (0., 0., 0.)
# Match the existing native velocity_smoother velocity_timeout, rather than
# adding a tighter watchdog that would chop ordinary controller scheduling gaps.
COMMAND_MAX_AGE_S = .5
RELEASE_MAX_AGE_S = .5


class RecoveryInterlock:
    def __init__(self, wait_for_delivery_handoff=False, shared_navigation=False):
        self.wait_for_delivery_handoff = wait_for_delivery_handoff
        self.shared_navigation = shared_navigation
        self.pickup_seen = False
        self.delivery_started = False
        self.identity = None
        self.last_status_stamp = None
        self.hold = wait_for_delivery_handoff or shared_navigation
        self.fault = None
        self.reason = ('waiting for delivery localization handoff' if wait_for_delivery_handoff
                       else 'native navigation')

    def update(self, status, now):
        if not isinstance(status, dict):
            return False
        stamp = status.get('published_ros_s')
        if (not isinstance(stamp, (int, float)) or isinstance(stamp, bool)
                or not math.isfinite(stamp) or stamp > now+.05
                or self.last_status_stamp is not None and stamp <= self.last_status_stamp):
            return False
        identity = (status.get('run_id'), status.get('map_file'))
        if not all(isinstance(value, str) and value for value in identity):
            return False
        if self.wait_for_delivery_handoff and self.identity is None:
            handoff = status.get('delivery_handoff')
            if (not isinstance(handoff, dict) or status.get('navigation_monitor') is not True
                    or (handoff.get('run_id'), handoff.get('map_file')) != identity
                    or now-stamp > RELEASE_MAX_AGE_S):
                return False
        if self.identity is None:
            self.identity = identity
        elif identity != self.identity:
            self.fault = 'localization session/map changed'
        if status.get('localization_fault'):
            self.fault = 'localization integrity fault: '+str(status['localization_fault'])
        self.last_status_stamp = stamp
        if status.get('navigation_monitor') is not True:
            trial = status.get('trial_handoff')
            if (self.shared_navigation and not self.delivery_started
                    and isinstance(trial, dict) and trial.get('enabled') is True):
                self.pickup_seen = self.hold = True
                self.reason = 'waiting for the pickup controller to release the base'
                return True
            self.fault = 'localization role changed'
        if self.shared_navigation and (self.pickup_seen or status.get('trial_handoff') is not None):
            handoff = status.get('delivery_handoff')
            if (not isinstance(handoff, dict) or (handoff.get('run_id'), handoff.get('map_file')) != identity
                    or now-stamp > RELEASE_MAX_AGE_S):
                self.hold = True
                self.reason = 'waiting for delivery localization handoff'
                return True
            self.delivery_started = True
        recovery = status.get('navigation_recovery')
        if not isinstance(recovery, dict) or type(recovery.get('hold_commands')) is not bool:
            return False
        if recovery['hold_commands']:
            self.hold = True
            self.reason = 'checking AMCL map transform; retaining the same Nav2 goal'
        elif now-stamp <= RELEASE_MAX_AGE_S:
            self.hold = False
            self.reason = 'native navigation'
        return True

    def filter(self, command, received_at, now):
        if (received_at is None or not -.05 <= now-received_at <= COMMAND_MAX_AGE_S
                or len(command) != 3 or not all(math.isfinite(v) for v in command)):
            return ZERO, 'Nav2 command missing/stale/invalid'
        if self.fault:
            return ZERO, self.fault
        if self.hold:
            return ZERO, self.reason
        return tuple(command), self.reason


class NavigationRecoveryGate(Node):
    def __init__(self):
        super().__init__('r3_navigation_recovery_gate')
        wait = self.declare_parameter('wait_for_delivery_handoff', False).value
        shared = self.declare_parameter('shared_navigation', False).value
        self.interlock = RecoveryInterlock(wait_for_delivery_handoff=wait, shared_navigation=shared)
        self.command = ZERO
        self.command_received_at = None
        self.last_reason = None
        self.output = self.create_publisher(Twist, '/cmd_vel_nav_checked', 1)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_command, 1)
        self.create_subscription(String, '/pallet/r3/localization/status', self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_command(self, message):
        self.command = (message.linear.x, message.linear.y, message.angular.z)
        self.command_received_at = self.now()

    def on_status(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        self.interlock.update(status, self.now())

    def tick(self):
        if self.command_received_at is None:
            return
        command, reason = self.interlock.filter(self.command, self.command_received_at, self.now())
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        self.output.publish(message)
        if reason != self.last_reason:
            self.last_reason = reason
            self.get_logger().info(reason)


def main():
    rclpy.init()
    node = NavigationRecoveryGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
