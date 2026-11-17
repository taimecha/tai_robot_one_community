"""Nav2 to a newly saved recognition A; reuses existing hardware and AMCL.

Only the collision monitor publishes /cmd_vel_safe. This launch must be stopped
before the A->C follower and its collision monitor are started.
"""
import math
from pathlib import Path
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


def navigation_parameters():
    root = Path(__file__).resolve().parents[3]
    params = yaml.safe_load((root/'ros2_ws/config/nav2_params.yaml').read_text())

    def real_time(value):
        if isinstance(value, dict):
            for key,item in value.items():
                if key == 'use_sim_time':
                    value[key] = False
                else:
                    real_time(item)
    real_time(params)
    bt=params['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml']=str(Path(get_package_share_directory('nav2_bt_navigator'))/
        'behavior_trees/navigate_w_replanning_only_if_path_becomes_invalid.xml')
    bt['bt_loop_duration']=50
    # The stock tree uses standard nodes, not the old route/recovery plugins.
    # Jazzy loads its built-in BT nodes by default. An empty YAML sequence
    # has no ROS parameter type and aborts bt_navigator during construction.
    bt.pop('plugin_lib_names',None)
    control=params['controller_server']['ros__parameters']
    # Coarse recognition A: leave margin inside the 10 cm / 5 degree region.
    # Leave room for braking and the final AMCL observation in both axes.
    control['general_goal_checker'].update(xy_goal_tolerance=.08,
        yaw_goal_tolerance=math.radians(4.),stateful=False,
        plugin='nav2_controller::StoppedGoalChecker',
        rot_stopped_velocity=.02,trans_stopped_velocity=.015)
    # A brief transform check keeps the same action alive while commands
    # brake to zero; it must not exhaust the normal five-second progress lease.
    control['progress_checker']['movement_time_allowance']=15.
    # Retain the real drivetrain's active speed floor. The previous 0.05 m/s
    # approach and 0.20 rad/s cap were below the existing controller's loaded
    # translation/rotation floors and could produce ineffective commands.
    control['FollowPath'].update(desired_linear_vel=.18,min_approach_linear_velocity=.10,
        regulated_linear_scaling_min_speed=.10,rotate_to_heading_angular_vel=.30,
        predictive_terminal_braking=True,terminal_command_latency=.25,
        terminal_angular_deceleration=1.5,allow_short_rear_goal_correction=True,
        allow_reverse_route_tracking=False,terminal_reverse_speed=.10,
        transient_tf_hold_timeout=.75)
    # Native Smac2D ends on a grid coordinate, which can differ from saved A.
    # Its opt-in wrapper collision-checks the short connector to exact A.
    planner=params['planner_server']['ros__parameters']
    for name in ('GridShortest','GridBased'):
        planner[name].update(plugin='tai_robot_one::ExactGoalSmac2D',
            tolerance=0.,use_final_approach_orientation=False)
    for layer in ('local_costmap','global_costmap'):
        costmap=params[layer][layer]['ros__parameters']
        for optional in ('camera_obstacle_layer','fork_safety_layer','rear_safety_layer'):
            if optional in costmap:
                costmap[optional]['enabled']=False
        costmap.update(update_frequency=5.,publish_frequency=2.)
    smoother=params['velocity_smoother']['ros__parameters']
    # Match the real drive's braking limits instead of adding a long final
    # ramp after Nav2 declares arrival. Keep a modest cap below the older
    # Nav2 cruise/turn speeds, but above the measured loaded speed floors.
    drive=yaml.safe_load((root/'ros2_ws/config/ros2_controllers_real.yaml').read_text())
    drive=drive['base_controller']['ros__parameters']
    smoother.update(max_velocity=[.18,0.,.30],min_velocity=[-.10,0.,-.30],
        max_accel=[.10,0.,.6],max_decel=[drive['linear.x.max_deceleration'],0.,
                                       drive['angular.z.max_deceleration']])
    behavior=params['behavior_server']['ros__parameters']
    behavior.update(max_rotational_vel=.30,min_rotational_vel=.22)
    monitor=params['collision_monitor']['ros__parameters']
    monitor.update(cmd_vel_in_topic='/cmd_vel_smoothed',cmd_vel_out_topic='/cmd_vel_safe',
        source_timeout=.5,observation_sources=['scan'])
    monitor['scan']['topic']='/scan_filtered'
    monitor['VelocityStop']['translation_reverse']['linear_min']=-.12
    # Retain the complete body/fork stopping envelope and footprint prediction.
    monitor['VelocityStop']['enabled']=True
    return params


def navigation_actions(context):
    params=navigation_parameters()
    # Nav2's nested costmap nodes need a YAML file, not a flat parameter dict.
    with tempfile.NamedTemporaryFile(mode='w',prefix='r3_nav2_',suffix='.yaml',delete=False) as file:
        yaml.safe_dump(params,file,sort_keys=False)
        filename=file.name
    specs=[('nav2_controller','controller_server'),('nav2_planner','planner_server'),
           ('nav2_behaviors','behavior_server'),('nav2_velocity_smoother','velocity_smoother'),
           ('nav2_collision_monitor','collision_monitor'),('nav2_bt_navigator','bt_navigator')]
    nodes=[]
    for package,name in specs:
        remaps=[('/scan','/scan_filtered')]
        if name in ('controller_server','behavior_server'):
            remaps.append(('cmd_vel','cmd_vel_nav'))
        if name=='velocity_smoother':
            remaps.append(('cmd_vel','cmd_vel_nav_checked'))
            remaps.append(('cmd_vel_smoothed','/cmd_vel_smoothed'))
        nodes.append(Node(package=package,executable=name,name=name,output='screen',
                          parameters=[filename],remappings=remaps))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',output='screen',parameters=[dict(use_sim_time=False,
            autostart=True,node_names=[name for _,name in specs],bond_timeout=4.)]))
    root = Path(__file__).resolve().parents[3]
    nodes.append(ExecuteProcess(cmd=[sys.executable,
        str(root/'simulation/r3_navigation_recovery_gate.py')],output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=navigation_actions)])
