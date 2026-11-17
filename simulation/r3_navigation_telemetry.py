#!/usr/bin/env python3
"""Passively record Nav2 commands and measured motion; publishes nothing."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist,TwistStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def finite(value):
    return float(value) if math.isfinite(value) else None


class NavigationTelemetry(Node):
    def __init__(self,output):
        super().__init__('r3_navigation_telemetry')
        output.parent.mkdir(parents=True,exist_ok=True)
        self.file=output.open('w',buffering=65536);self.latest={};self.ticks=0
        self.plans_file=output.with_name('navigation_plans.jsonl').open('w',buffering=65536)
        self.last_record_at=time.monotonic()
        self.file.write(json.dumps(dict(schema='r3_navigation_motion_v2',
            observer_only=True,robot_commands_from_this_process=0))+'\n')
        for topic in ['/cmd_vel_nav','/cmd_vel_nav_checked','/cmd_vel_smoothed','/cmd_vel_safe']:
            self.create_subscription(Twist,topic,
                lambda m,t=topic:self.command(t,m),qos_profile_sensor_data)
        self.create_subscription(TwistStamped,'/base_controller/cmd_vel_out',
            lambda m:self.command('/base_controller/cmd_vel_out',m.twist,self.stamp(m)),qos_profile_sensor_data)
        for topic in ['/odom','/base_controller/odom']:
            self.create_subscription(Odometry,topic,
                lambda m,t=topic:self.odometry(t,m),qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl,qos_profile_sensor_data)
        self.create_subscription(NavPath,'/plan',self.plan,qos_profile_sensor_data)
        # Capture both ends of the filter and the actual TF edge stamps. An
        # AMCL pose callback delayed in another process cannot establish
        # whether LiDAR delivery, AMCL processing or that observer was late.
        for topic in ('/scan','/scan_filtered'):
            self.create_subscription(LaserScan,topic,
                lambda m,t=topic:self.scan(t,m),qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf',self.transforms,qos_profile_sensor_data)
        self.create_subscription(String, '/pallet/r3/localization/status', self.localization,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/pallet/r3/control_odom/status', self.control_odometry,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05,self.record)
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    @staticmethod
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    @staticmethod
    def yaw(q):return finite(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def command(self,topic,msg,stamp=None):
        self.latest[topic]=dict(received_ros_s=self.now(),stamp_ros_s=stamp,
            vx=finite(msg.linear.x),wz=finite(msg.angular.z))
    def odometry(self,topic,msg):
        p=msg.pose.pose;t=msg.twist.twist
        self.latest[topic]=dict(received_ros_s=self.now(),stamp_ros_s=self.stamp(msg),x=finite(p.position.x),
            y=finite(p.position.y),yaw=self.yaw(p.orientation),vx=finite(t.linear.x),wz=finite(t.angular.z))
    def imu(self,msg):
        self.latest['/imu']=dict(received_ros_s=self.now(),stamp_ros_s=self.stamp(msg),yaw=self.yaw(msg.orientation),
            wz=finite(msg.angular_velocity.z))
    def amcl(self,msg):
        p=msg.pose.pose
        self.latest['/amcl_pose']=dict(received_ros_s=self.now(),stamp_ros_s=self.stamp(msg),x=finite(p.position.x),
            y=finite(p.position.y),yaw=self.yaw(p.orientation))
    def plan(self,msg):
        poses=[[finite(p.pose.position.x),finite(p.pose.position.y),self.yaw(p.pose.orientation)]
               for p in msg.poses]
        length=sum(math.hypot(b[0]-a[0],b[1]-a[1])
                   for a,b in zip(poses,poses[1:]) if None not in (*a[:2],*b[:2]))
        self.plans_file.write(json.dumps(dict(ros_s=self.now(),stamp_ros_s=self.stamp(msg),
            topic='/plan',frame_id=msg.header.frame_id,poses_xyyaw=poses,
            goal_pose_stamp_ros_s=self.stamp(msg.poses[-1]) if msg.poses else None,
            path_length_m=length,robot_commands_from_this_process=0),allow_nan=False)+'\n')
        self.plans_file.flush()
    def scan(self,topic,msg):
        self.latest[topic]=dict(received_ros_s=self.now(),stamp_ros_s=self.stamp(msg),
            frame_id=msg.header.frame_id,beam_count=len(msg.ranges),scan_time_s=finite(msg.scan_time))
    def transforms(self,msg):
        received=self.now()
        for transform in msg.transforms:
            edge=(transform.header.frame_id,transform.child_frame_id)
            if edge not in (('map','odom'),('odom','base_footprint')):
                continue
            t=transform.transform
            self.latest['tf:'+edge[0]+'->'+edge[1]]=dict(received_ros_s=received,
                stamp_ros_s=self.stamp(transform),x=finite(t.translation.x),y=finite(t.translation.y),
                yaw=self.yaw(t.rotation),stamp_includes_amcl_tolerance=edge==('map','odom'))
    def localization(self,msg):
        try:
            value=json.loads(msg.data)
            self.latest['localization_recovery']={key:value.get(key) for key in (
                'published_ros_s','run_id','role','ready','localization_fault',
                'navigation_recovery','map_update_event','amcl_pipeline')}
        except (ValueError, TypeError, AttributeError):
            pass
    def record(self):
        wall=time.monotonic()
        self.file.write(json.dumps(dict(ros_s=self.now(),
            observer_period_s=wall-self.last_record_at,**self.latest),allow_nan=False)+'\n')
        self.last_record_at=wall
        self.ticks+=1
        if self.ticks%20==0:self.file.flush()

    def control_odometry(self,msg):
        try:
            value=json.loads(msg.data)
            self.latest['control_odom_status']={key:value.get(key) for key in (
                'published_ros_s','ready','reason','integrity_fault','control_mode',
                'control_stream_id','output_samples','raw_input_samples',
                'latest_input_capture_stamp_s','latest_input_received_ros_s',
                'latest_output_capture_stamp_s')}
        except (ValueError,TypeError,AttributeError):
            pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rclpy.init();node=NavigationTelemetry(args.output)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.file.flush();node.file.close();node.plans_file.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
