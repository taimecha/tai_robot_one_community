"""Publish a low-obstacle scan from the Astra node already running on this Pi."""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def obstacle_actions(context):
    root=Path(__file__).resolve().parents[3]
    floor_alignment=LaunchConfiguration('floor_alignment').perform(context).lower()=='true'
    cloud_topic='/camera/depth/points_nav_floor_corrected' if floor_alignment else '/camera/depth/points_nav_corrected'
    nodes=[
        Node(
            package='tai_robot_one', executable='publish_rgb_calibrated_cloud.py',
            name='spatial_depth_cloud', output='screen',
            parameters=[{
                'pixel_stride': 4, 'max_rate_hz': 4.0,
                'floor_reference': True,
                'fallback_to_static_slope': not floor_alignment,
                'inverse_depth_x_slope': 0.00015807,
                'reference_u': 370.,
                'max_input_age_s': .5,
                'output_topic': '/camera/depth/points_nav_corrected',
            }],
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='camera_obstacle_scan', output='screen',
            remappings=[('cloud_in', cloud_topic),
                        ('scan', '/camera/obstacle_scan')],
            parameters=[{
                'use_sim_time': False,
                'target_frame': 'base_footprint', 'transform_tolerance': .10,
                'min_height': .06, 'max_height': .75,
                'angle_min': -.5235987756, 'angle_max': .5235987756,
                'angle_increment': .00872664626, 'queue_size': 1,
                'scan_time': .25, 'range_min': .60, 'range_max': 6.0,
                'use_inf': True,
            }],
        ),
    ]
    if floor_alignment:
        report=LaunchConfiguration('floor_alignment_report').perform(context)
        nodes.insert(1,ExecuteProcess(cmd=[sys.executable,str(root/'simulation/r3_floor_aligned_cloud.py'),
            '--ros-args','-p','report_file:='+report],output='screen',
            additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('floor_alignment',default_value='false'),
        DeclareLaunchArgument('floor_alignment_report',default_value=''),
        OpaqueFunction(function=obstacle_actions),
    ])
