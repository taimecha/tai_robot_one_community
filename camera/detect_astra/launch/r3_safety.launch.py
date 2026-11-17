"""Real r3 obstacle chain. Requires a fresh locked plan; no wheel commands here."""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project = Path(__file__).resolve().parents[3]
    config = Path(__file__).resolve().parents[1] / 'config/r3_collision_monitor.yaml'
    filter_process = ExecuteProcess(
        cmd=[sys.executable, str(project/'simulation/r3_real_obstacle_filter.py'),
             '--plan', LaunchConfiguration('plan'), '--allow-provisional',
             '--diagnostics-dir', LaunchConfiguration('diagnostics_dir'),
             '--collision-config', str(config)],
        name='r3_pallet_obstacle_filter', output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('plan', description='Fresh frozen real RGB-D plan'),
        DeclareLaunchArgument('diagnostics_dir', default_value='',
                              description='Optional per-run StopZone scan evidence directory'),
        filter_process,
        RegisterEventHandler(OnProcessExit(
            target_action=filter_process,
            on_exit=[EmitEvent(event=Shutdown(reason='Pallet scan filter exited'))])),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='r3_collision_monitor', output='screen', parameters=[str(config)]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='r3_safety_manager', output='screen', parameters=[{
                 'autostart': True, 'node_names': ['r3_collision_monitor'],
                 'bond_timeout': 2.0, 'attempt_respawn_reconnection': False}]),
    ])
