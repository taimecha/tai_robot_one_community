"""AMCL/map server for an r3 trial; existing hardware, TF and scan are reused.

This launch never starts navigation servers, wheel controllers, a LiDAR driver,
or a second odometry estimator. AMCL alone supplies map -> odom; the existing
EKF continues to own odom -> base_footprint.
"""
import math
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_map_file(value):
    """Require the caller's real map, including a readable local image."""
    path = Path(value)
    if not path.is_absolute():
        raise ValueError('map_file must be an absolute path to the selected real map YAML')
    if not path.is_file() or path.suffix.lower() not in ('.yaml', '.yml'):
        raise ValueError(f'map YAML does not exist: {path}')
    with path.open(encoding='utf-8') as stream:
        description = yaml.safe_load(stream)
    if not isinstance(description, dict) or not isinstance(description.get('image'), str):
        raise ValueError('map YAML must contain an image path')
    image = Path(description['image'])
    if not image.is_absolute():
        image = path.parent/image
    if not image.is_file():
        raise ValueError(f'map image does not exist: {image}')
    resolution = description.get('resolution')
    origin = description.get('origin')
    if (isinstance(resolution, bool) or not isinstance(resolution, (int, float))
            or not math.isfinite(resolution) or resolution <= 0
            or not isinstance(origin, list) or len(origin) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in origin)):
        raise ValueError('map YAML has invalid resolution or origin')
    return path


def localization_actions(context):
    map_file = validate_map_file(LaunchConfiguration('map_file').perform(context))
    config = Path(__file__).resolve().parents[1]/'config/r3_localization.yaml'
    stationary = LaunchConfiguration('stationary_inspection', default='false').perform(context).strip().lower()
    if stationary not in ('true', 'false', '1', '0'):
        raise ValueError('stationary_inspection must be an explicit true/false or 1/0')
    precision = LaunchConfiguration('precision_localization', default='false').perform(context).strip().lower()
    if precision not in ('true','false','1','0'):
        raise ValueError('precision_localization must be an explicit true/false or 1/0')
    amcl_parameters = [str(config)]
    if stationary in ('true', '1') or precision in ('true','1'):
        amcl_parameters.append(str(config.with_name('r3_stationary_localization.yaml')))
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[str(config), {'yaml_filename': str(map_file)}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=amcl_parameters,
             # Keep sensor processing independent of synchronous DDS writes
             # to remote RViz readers. This node publishes no drive commands.
             additional_env={'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS'}),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[{
                 'use_sim_time': False, 'autostart': True,
                 'node_names': ['map_server', 'amcl'],
                 'bond_timeout': 4.0, 'attempt_respawn_reconnection': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file', description='Absolute YAML path for the selected real map'),
        DeclareLaunchArgument('stationary_inspection', default_value='false',
                              description='Use the parked localization AMCL sensor profile'),
        DeclareLaunchArgument('precision_localization', default_value='false',
                              description='Use the validated product likelihood profile during localization and motion'),
        OpaqueFunction(function=localization_actions),
    ])
