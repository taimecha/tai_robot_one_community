"""Preview Astra pallet obstacles without starting the robot's motors or Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robot_share = get_package_share_directory('tai_robot_one')
    astra_share = get_package_share_directory('astra_camera')
    robot_urdf = xacro.process_file(
        os.path.join(robot_share, 'description', 'robot.urdf.xacro'),
        mappings={
            'use_sim': 'false',
            'hardware_plugin': 'tai_robot_one/TaiRobotSerialSystem',
            'use_lift': 'false',
        },
    ).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='pallet_preview_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf, 'use_sim_time': False}],
        ),
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
                'depth_width': '320',
                'depth_height': '240',
                'depth_fps': '30',
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pallet_preview_depth_optical_tf',
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
            name='pallet_obstacle_scan',
            output='screen',
            remappings=[
                ('cloud_in', '/camera/depth/points'),
                ('scan', '/camera/obstacle_scan'),
            ],
            parameters=[{
                'use_sim_time': False,
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.10,
                'min_height': 0.06,
                'max_height': 0.75,
                'angle_min': -0.5235987756,
                'angle_max': 0.5235987756,
                'angle_increment': 0.01745329252,
                'queue_size': 1,
                'scan_time': 0.033,
                'range_min': 0.60,
                'range_max': 6.00,
                'use_inf': True,
            }],
        ),
    ])
