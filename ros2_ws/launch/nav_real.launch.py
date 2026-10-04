"""Bring up the real robot sensors, AMCL, and optionally Nav2 navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def _validate_arguments(context):
    map_file = LaunchConfiguration('map_file').perform(context)
    if not os.path.isabs(map_file):
        raise RuntimeError(
            f'The map argument must be an absolute path, got: {map_file}')
    if not os.path.isfile(map_file):
        raise RuntimeError(f'Map YAML file does not exist: {map_file}')
    if not map_file.lower().endswith(('.yaml', '.yml')):
        raise RuntimeError(f'Map argument must name a YAML file: {map_file}')

    navigation = LaunchConfiguration('start_navigation').perform(context)
    teleop = LaunchConfiguration('use_cm029_teleop').perform(context)
    if navigation.lower() in ('1', 'true', 'yes', 'on') and teleop.lower() in (
            '1', 'true', 'yes', 'on'):
        raise RuntimeError(
            'CM029 teleop and Nav2 cannot both publish velocity commands')
    return []


def _nav2_actions(context, nav2_share, params_file):
    map_file = LaunchConfiguration('map_file').perform(context)
    test_rewrites = {}
    if LaunchConfiguration('motion_test').perform(context).lower() == 'true':
        test_rewrites = {
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(
                get_package_share_directory('nav2_bt_navigator'), 'behavior_trees',
                'navigate_w_replanning_only_if_path_becomes_invalid.xml'),
            'controller_server.ros__parameters.FollowPath.min_approach_linear_velocity': '0.12',
            'controller_server.ros__parameters.FollowPath.regulated_linear_scaling_min_speed': '0.12',
        }
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(
                get_package_share_directory('tai_robot_one'), 'behavior_trees',
                'navigate_real_clearance_escape.xml'),
            **test_rewrites,
            'use_sim_time': 'false',
            'yaml_filename': map_file,
            'set_initial_pose': 'false',
            'amcl.ros__parameters.scan_topic': '/scan_filtered',
            'amcl.ros__parameters.update_min_d': '0.03',
            'amcl.ros__parameters.update_min_a': '0.02',
            'amcl.ros__parameters.max_beams': '120',
            # Wheel/EKF yaw agreed closely while AMCL invented 10--16 cm of
            # translation during pure turns. Trust rotational odometry more.
            # The real EKF now has stable BNO055 quaternion yaw, so straight
            # travel must not inject the simulation default's large yaw and
            # translation spread into the particle cloud.
            'amcl.ros__parameters.alpha2': '0.02',
            'amcl.ros__parameters.alpha3': '0.05',
            'amcl.ros__parameters.alpha4': '0.05',
            # The selected path is checked at each BT tick. 20 Hz is faster
            # than one 2.5 cm costmap cell at the robot's cruise speed and
            # avoids rebuilding two costmaps 100 times per second.
            'bt_navigator.ros__parameters.bt_loop_duration': '50',
            # Keep obstacle updates at 8 Hz; publish the 200x200 local grid
            # at 4 Hz for RViz and the route validator.
            'local_costmap.local_costmap.ros__parameters.publish_frequency': '4.0',
            'collision_monitor.ros__parameters.source_timeout': '0.5',
            # Real robot: keep the costmap footprint and swept-footprint
            # approach checking, but remove the extra red stop envelope.
            # Simulation continues to use VelocityStop from nav2_params.yaml.
            'collision_monitor.ros__parameters.VelocityStop.enabled': 'false',
            'collision_monitor.ros__parameters.VelocityStop.visualize': 'false',
            'collision_monitor.ros__parameters.rear_scan.enabled': 'false',
            'collision_monitor.ros__parameters.fork_scan.enabled': 'false',
            'collision_monitor.ros__parameters.camera_obstacle_scan.enabled':
                LaunchConfiguration('use_camera'),
            'local_costmap.local_costmap.ros__parameters.camera_obstacle_layer.enabled':
                LaunchConfiguration('use_camera'),
            'global_costmap.global_costmap.ros__parameters.camera_obstacle_layer.enabled':
                LaunchConfiguration('use_camera'),
            # There are no publishers for these optional sensors on the
            # LiDAR-only robot; do not run empty obstacle layers.
            'local_costmap.local_costmap.ros__parameters.fork_safety_layer.enabled': 'false',
            'local_costmap.local_costmap.ros__parameters.rear_safety_layer.enabled': 'false',
            'global_costmap.global_costmap.ros__parameters.fork_safety_layer.enabled': 'false',
        },
        convert_types=True,
    )
    common = {
        'use_sim_time': 'false',
        'autostart': 'true',
        'use_composition': 'False',
        'params_file': configured_params,
    }
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={**common, 'map': map_file}.items(),
    )
    # RViz NavigateToPose uses these six servers. The generic Nav2 bringup
    # starts four more independent processes (smoother, route, waypoint and
    # docking) that this robot's navigation tree never calls.
    nav_specs = [
        ('nav2_controller', 'controller_server', 'controller_server', True),
        ('nav2_planner', 'planner_server', 'planner_server', False),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', True),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', True),
        ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', False),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', False),
    ]
    nav_nodes = []
    for package, executable, name, sends_cmd_vel in nav_specs:
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        if sends_cmd_vel:
            remappings.append(('cmd_vel', 'cmd_vel_nav'))
        nav_nodes.append(Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=remappings,
        ))
    nav_nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': LaunchConfiguration('navigation_autostart')},
            {'node_names': [spec[2] for spec in nav_specs]},
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    ))
    navigation = GroupAction(
        actions=nav_nodes,
        condition=IfCondition(LaunchConfiguration('start_navigation')),
    )
    return [
        GroupAction([
            SetRemap(src='/scan', dst='/scan_filtered'),
            localization,
            navigation,
        ])
    ]


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    nav2_share = get_package_share_directory('nav2_bringup')
    lidar_share = get_package_share_directory('sllidar_ros2')
    astra_share = get_package_share_directory('astra_camera')
    params_file = os.path.join(package_share, 'config', 'nav2_params.yaml')

    real_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'real_hardware.launch.py')),
        launch_arguments={
            'use_lift': 'false',
            'use_rviz': 'false',
            'use_cm029_teleop': LaunchConfiguration('use_cm029_teleop'),
            'cmd_vel_input_topic': PythonExpression([
                "'/cmd_vel_safe' if '",
                LaunchConfiguration('start_navigation'),
                "'.lower() in ('1', 'true', 'yes', 'on') else '/cmd_vel'",
            ]),
            'cm029_linear_speed': '0.20',
            'cm029_angular_speed': '0.40',
        }.items(),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            lidar_share, 'launch', 'sllidar_a1_launch.py')),
        launch_arguments={
            'serial_port': '/dev/tai_lidar',
            'frame_id': 'lidar_link',
        }.items(),
    )

    scan_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'scan_self_filter.launch.py')),
    )

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            astra_share, 'launch', 'astra_pro.launch.xml')),
        launch_arguments={
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'false',
            # Never accumulate old depth frames while creating the point cloud.
            'queue_size': '1',
            'enable_ir': 'false',
            'tf_publish_rate': '0.0',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '15',
            'depth_width': '320',
            'depth_height': '240',
            # Astra only supports this depth mode at 30 Hz. Throttle the cloud
            # downstream so the driver does not silently fall back at runtime.
            'depth_fps': '30',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    camera_compressed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'camera_compressed.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    depth_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='depth_raw_throttle',
        output='screen',
        arguments=[
            'messages',
            '/camera/depth/image_raw',
            '10.0',
            '/camera/depth/image_raw_10fps',
        ],
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    camera_obstacle_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='camera_obstacle_scan',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/depth/points'),
            ('scan', '/camera/obstacle_scan'),
        ],
        parameters=[{
            'use_sim_time': False,
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.10,
            # Reject floor noise from the real camera pitched 20 degrees down.
            'min_height': 0.12,
            'max_height': 0.75,
            'angle_min': -0.5235987756,
            'angle_max': 0.5235987756,
            'angle_increment': 0.01745329252,
            'queue_size': 1,
            'scan_time': 0.10,
            'range_min': 0.60,
            'range_max': 6.00,
            'use_inf': True,
        }],
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    nav_path_display_filter = Node(
        package='tai_robot_one',
        executable='nav_path_display_filter',
        name='nav_path_display_filter',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'global_path_topic': PythonExpression([
                "'/plan' if '", LaunchConfiguration('motion_test'),
                "'.lower() in ('1', 'true', 'yes', 'on') else '/plan_selected'",
            ]),
        }],
        condition=IfCondition(LaunchConfiguration('start_navigation')),
    )

    nav2 = OpaqueFunction(
        function=_nav2_actions,
        args=[nav2_share, params_file],
    )

    pose_persistence = Node(
        package='tai_robot_one',
        executable='amcl_pose_persistence',
        name='amcl_pose_persistence',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('restore_saved_pose')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            description='Absolute path to the saved map YAML'),
        DeclareLaunchArgument(
            'start_navigation', default_value='false',
            description='Start Nav2 planning/control after AMCL'),
        DeclareLaunchArgument(
            'navigation_autostart', default_value='true',
            description='Set false until AMCL has an initial pose'),
        DeclareLaunchArgument(
            'use_cm029_teleop', default_value='false',
            description='Allow CM029 only while start_navigation is false'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Start Astra and enable camera obstacle sources'),
        DeclareLaunchArgument(
            'restore_saved_pose', default_value='true',
            description='Restore and persist the last AMCL pose'),
        DeclareLaunchArgument(
            'motion_test', default_value='false', choices=['true', 'false'],
            description='Single-goal diagnostic without automatic motion recovery'),
        DeclareLaunchArgument(
            'localization_delay', default_value='8.0',
            description='Seconds to wait for hardware and sensors'),
        OpaqueFunction(function=_validate_arguments),
        real_hardware,
        lidar,
        scan_filter,
        camera,
        camera_compressed,
        depth_throttle,
        camera_obstacle_scan,
        nav_path_display_filter,
        TimerAction(
            period=LaunchConfiguration('localization_delay'),
            actions=[nav2]),
        TimerAction(
            period=PythonExpression([
                LaunchConfiguration('localization_delay'), " + 2.0",
            ]),
            actions=[pose_persistence]),
    ])
