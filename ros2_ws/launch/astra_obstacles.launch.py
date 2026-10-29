"""Astra Pro low-obstacle scan from detect_astra's calibrated depth cloud.

The robot_state_publisher and motors belong to nav_real.launch.py. This file
contains only the camera pipeline and may also be used for sensor inspection.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    astra_share = get_package_share_directory('astra_camera')
    robot_share = get_package_share_directory('tai_robot_one')
    rgb_calibration = os.path.join(
        robot_share, 'config', 'calibration',
        'astra_pro_rgb_640x480_2026-10-11.yaml')
    depth_correction = os.path.join(
        robot_share, 'config', 'calibration',
        'astra_pro_depth_x_skew_preview.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_scan_hz', default_value='15.0'),
        DeclareLaunchArgument('camera_cloud_pixel_stride', default_value='2'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                astra_share, 'launch', 'astra_pro.launch.xml')),
            launch_arguments={
                'depth_registration': 'true',
                'enable_color': 'true',
                'use_uvc_camera': 'true',
                'color_info_url': 'file://' + rgb_calibration,
                'enable_ir': 'false',
                'enable_depth': 'true',
                'enable_point_cloud': 'false',
                'enable_colored_point_cloud': 'false',
                'publish_tf': 'true',
                'tf_publish_rate': '0.0',
                'queue_size': '1',
                'color_width': '640',
                'color_height': '480',
                'color_fps': '15',
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': '30',
            }.items(),
        ),
        Node(
            package='tai_robot_one',
            executable='publish_rgb_calibrated_cloud.py',
            name='spatial_depth_cloud',
            output='screen',
            parameters=[depth_correction, {
                'pixel_stride': ParameterValue(
                    LaunchConfiguration('camera_cloud_pixel_stride'), value_type=int),
                'max_rate_hz': ParameterValue(
                    LaunchConfiguration('camera_scan_hz'), value_type=float),
                'output_topic': '/camera/depth/points_nav_corrected',
            }],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='camera_obstacle_scan',
            output='screen',
            remappings=[
                ('cloud_in', '/camera/depth/points_nav_corrected'),
                ('scan', '/camera/obstacle_scan'),
            ],
            parameters=[{
                'use_sim_time': False,
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.10,
                # Preserve the robot's low-obstacle height window.
                'min_height': 0.06,
                'max_height': 0.75,
                'angle_min': -0.5235987756,
                'angle_max': 0.5235987756,
                # At 2 m, half-degree rays are about 1.7 cm apart, closer
                # than the local costmap's 2.5 cm cells. This lets free rays
                # erase old marks after a person steps out of the corridor.
                'angle_increment': 0.00872664626,
                'queue_size': 1,
                'scan_time': ParameterValue(PythonExpression([
                    '1.0 / ', LaunchConfiguration('camera_scan_hz')
                ]), value_type=float),
                'range_min': 0.60,
                'range_max': 6.00,
                'use_inf': True,
            }],
        ),
    ])
