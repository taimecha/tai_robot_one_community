import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start_after_success(action, process_name):
    """Prevent later controllers from starting after an earlier failure."""
    def _on_exit(event, _context):
        if event.returncode == 0:
            return [action]
        return [EmitEvent(event=Shutdown(
            reason=f'{process_name} failed with exit code {event.returncode}'))]

    return _on_exit


def generate_launch_description():
    package_share = get_package_share_directory('tai_robot_one')
    xacro_file = os.path.join(
        package_share, 'description', 'robot.urdf.xacro')
    controllers_file = os.path.join(
        package_share, 'config', 'ros2_controllers_real.yaml')
    ekf_file = os.path.join(package_share, 'config', 'ekf.yaml')
    imu_config = os.path.join(package_share, 'config', 'imu_bridge.yaml')
    rviz_config = os.path.join(
        package_share, 'config', 'gazebo_robot.rviz')

    drive_port = LaunchConfiguration('drive_serial_port')
    lift_port = LaunchConfiguration('lift_serial_port')
    imu_port = LaunchConfiguration('imu_serial_port')
    use_lift = LaunchConfiguration('use_lift')
    home_lift = LaunchConfiguration('home_lift_on_activate')
    use_rviz = LaunchConfiguration('use_rviz')
    use_cm029_teleop = LaunchConfiguration('use_cm029_teleop')
    cmd_vel_input_topic = LaunchConfiguration('cmd_vel_input_topic')
    cm029_linear_speed = LaunchConfiguration('cm029_linear_speed')
    cm029_angular_speed = LaunchConfiguration('cm029_angular_speed')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' use_sim:=false',
            ' hardware_plugin:=tai_robot_one/TaiRobotSerialSystem',
            ' controllers_file:=', controllers_file,
            ' drive_serial_port:=', drive_port,
            ' lift_serial_port:=', lift_port,
            ' use_lift:=', use_lift,
            ' home_lift_on_activate:=', home_lift,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False},
            controllers_file,
        ],
    )

    cmd_vel_stamper = Node(
        package='tai_robot_one',
        executable='cmd_vel_stamper',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'input_topic': cmd_vel_input_topic,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file, {'use_sim_time': False}],
        remappings=[('odometry/filtered', '/odom')],
    )

    imu_bridge = Node(
        package='tai_robot_one',
        executable='esp32_imu_bridge',
        name='esp32_imu_bridge',
        output='screen',
        parameters=[imu_config, {
            'port': imu_port,
            'baud': 115200,
            'frame_id': 'imu_link',
            'topic': '/imu',
        }],
    )

    imu_visualizer = Node(
        package='tai_robot_one',
        executable='imu_visualizer',
        name='imu_visualizer',
        output='screen',
        # RViz attitude markers are diagnostic only; 5 Hz keeps them readable
        # without spending a Python process on 20 marker arrays each second.
        parameters=[{'publish_rate': 5.0}],
    )

    cm029_teleop = Node(
        package='tai_robot_one',
        executable='cm029_teleop',
        name='cm029_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'joy_topic': '/joy',
            'linear_speed': ParameterValue(
                cm029_linear_speed, value_type=float),
            'angular_speed': ParameterValue(
                cm029_angular_speed, value_type=float),
            'deadzone': 0.10,
            'joy_timeout': 0.30,
            'lift_speed': 0.05,
        }],
        condition=IfCondition(use_cm029_teleop),
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '90',
            '--switch-timeout', '90',
            '--service-call-timeout', '90',
        ],
    )
    base_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='base_controller_spawner',
        output='screen',
        arguments=[
            'base_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '90',
            '--switch-timeout', '90',
            '--service-call-timeout', '90',
            '--param-file', controllers_file,
        ],
    )
    lift_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='lift_controller_spawner',
        output='screen',
        arguments=[
            'lift_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '90',
            '--switch-timeout', '90',
            '--service-call-timeout', '90',
            '--param-file', controllers_file,
        ],
        condition=IfCondition(use_lift),
    )

    start_joint_state_broadcaster = TimerAction(
        period=2.0, actions=[joint_state_broadcaster])
    start_base_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=_start_after_success(
                base_controller, 'joint_state_broadcaster spawner'),
        )
    )
    start_lift_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=base_controller,
            on_exit=_start_after_success(
                lift_controller, 'base_controller spawner'),
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_serial_port',
            default_value='/dev/tai_drive',
            description='Stable udev symlink for the base ESP32'),
        DeclareLaunchArgument(
            'lift_serial_port',
            default_value='/dev/tai_lift',
            description='Stable udev symlink for the lift ESP32'),
        DeclareLaunchArgument(
            'imu_serial_port',
            default_value='/dev/tai_imu',
            description='Stable udev symlink for the BNO055 ESP32'),
        DeclareLaunchArgument(
            'use_lift',
            default_value='false',
            description=(
                'Keep false for the combined IMU/lift ESP32: esp32_imu_bridge '
                'owns /dev/tai_imu and publishes lift feedback')),
        DeclareLaunchArgument(
            'home_lift_on_activate',
            default_value='true',
            description='Home the lift automatically before arming it'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Open RViz on the Raspberry Pi'),
        DeclareLaunchArgument(
            'use_cm029_teleop',
            default_value='false',
            description=(
                'Run CM029 teleop on the Pi; joy_node remains on the laptop')),
        DeclareLaunchArgument(
            'cmd_vel_input_topic',
            default_value='/cmd_vel',
            description='Final unstamped velocity topic sent to ros2_control'),
        DeclareLaunchArgument(
            'cm029_linear_speed',
            default_value='0.50',
            description='CM029 maximum linear speed in m/s'),
        DeclareLaunchArgument(
            'cm029_angular_speed',
            default_value='1.00',
            description='CM029 maximum angular speed in rad/s'),
        robot_state_publisher,
        controller_manager,
        cmd_vel_stamper,
        imu_bridge,
        imu_visualizer,
        cm029_teleop,
        ekf,
        start_joint_state_broadcaster,
        start_base_controller,
        start_lift_controller,
        rviz,
    ])
