"""Astra Pro low-obstacle scan, adapted from detect_astra_pro's preview launch.

The robot_state_publisher and motors belong to nav_real.launch.py. This file
contains only the camera pipeline and may also be used for sensor inspection.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    astra_share = get_package_share_directory('astra_camera')
    return LaunchDescription([
        DeclareLaunchArgument('camera_scan_hz', default_value='20.0'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                astra_share, 'launch', 'astra_pro.launch.xml')),
            launch_arguments={
                'enable_color': 'false',
                'use_uvc_camera': 'false',
                'enable_ir': 'false',
                'enable_depth': 'true',
                'enable_point_cloud': 'true',
                'enable_colored_point_cloud': 'false',
                'publish_tf': 'false',
                'tf_publish_rate': '0.0',
                'queue_size': '1',
                'depth_width': '320',
                'depth_height': '240',
                # The Astra's validated 320x240 depth mode runs at 30 fps.
                # The Pi produces about 19 cloud/scan messages per second.
                'depth_fps': '30',
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='astra_depth_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'camera_link_optical',
                '--child-frame-id', 'camera_depth_optical_frame',
            ],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='camera_obstacle_scan',
            output='screen',
            remappings=[
                ('cloud_in', '/camera/depth/points'),
                ('scan', '/camera/obstacle_scan_raw'),
            ],
            parameters=[{
                'use_sim_time': False,
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.10,
                # Values from detect_astra_pro's pallet obstacle preview.
                'min_height': 0.06,
                'max_height': 0.75,
                'angle_min': -0.5235987756,
                'angle_max': 0.5235987756,
                'angle_increment': 0.01745329252,
                'queue_size': 1,
                'scan_time': 0.05,
                'range_min': 0.60,
                'range_max': 6.00,
                'use_inf': True,
            }],
        ),
        # Throttle only the 61-beam LaserScan, never the large point cloud.
        # The older cloud throttle delayed the obstacle scan by seconds.
        Node(
            package='topic_tools',
            executable='throttle',
            name='camera_obstacle_scan_rate',
            output='screen',
            arguments=[
                'messages', '/camera/obstacle_scan_raw',
                LaunchConfiguration('camera_scan_hz'),
                '/camera/obstacle_scan',
            ],
        ),
    ])
