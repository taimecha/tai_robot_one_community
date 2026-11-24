"""Nav2 to a newly saved recognition A; reuses existing hardware and AMCL.

Only the collision monitor publishes /cmd_vel_safe. This launch must be stopped
before the A->C follower and its collision monitor are started.
"""
import math
from pathlib import Path
import sys
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def navigation_parameters(loaded_delivery=False, camera_obstacles=False):
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
    bt['default_nav_to_pose_bt_xml']=str(root/'simulation/r3/nav_to_a_straight_first.xml')
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
        regulated_linear_scaling_min_speed=.10,rotate_to_heading_angular_vel=.35,
        rotate_to_heading_min_angle=.60,
        predictive_terminal_braking=True,terminal_command_latency=.25,
        terminal_angular_deceleration=1.5,allow_short_rear_goal_correction=True,
        allow_reverse_route_tracking=False,terminal_reverse_speed=.10,
        transient_tf_hold_timeout=.75)
    # Native Smac2D ends on a grid coordinate, which can differ from saved A.
    # Its opt-in wrapper collision-checks the short connector to exact A.
    planner=params['planner_server']['ros__parameters']
    planner['planner_plugins']=['GridShortest','GridBased']
    for name in ('GridShortest','GridBased'):
        planner[name].update(plugin='tai_robot_one::ExactGoalSmac2D',
            tolerance=0.,use_final_approach_orientation=False,
            prefer_straight_path=True,
            straight_goal_xy_tolerance=control['general_goal_checker']['xy_goal_tolerance'])
    for layer in ('local_costmap','global_costmap'):
        costmap=params[layer][layer]['ros__parameters']
        for optional in ('camera_obstacle_layer','fork_safety_layer','rear_safety_layer'):
            if optional in costmap:
                costmap[optional]['enabled']=bool(camera_obstacles) if optional=='camera_obstacle_layer' else False
        if camera_obstacles and 'camera_obstacle_layer' in costmap:
            cam_src = costmap['camera_obstacle_layer'].get('camera_obstacle_scan', {})
            # Clear the 0.78 m loaded pallet nose so carried pallet cannot self-mark.
            min_r = 0.85 if loaded_delivery else 0.60
            cam_src['obstacle_min_range'] = min_r
            cam_src['raytrace_min_range'] = min_r
        costmap.update(update_frequency=5.,publish_frequency=2.)
        if loaded_delivery:
            # Same chassis and front projection as at A. Widen the front
            # for the 355 mm pallet, its alignment tolerance and map padding.
            costmap['footprint']='[[0.78,0.22],[0.78,-0.22],[0.38,-0.25],[-0.33,-0.25],[-0.33,0.25],[0.38,0.25]]'
    smoother=params['velocity_smoother']['ros__parameters']
    # Match the real drive's braking limits instead of adding a long final
    # ramp after Nav2 declares arrival. Keep a modest cap below the older
    # Nav2 cruise/turn speeds, but above the measured loaded speed floors.
    drive=yaml.safe_load((root/'ros2_ws/config/ros2_controllers_real.yaml').read_text())
    drive=drive['base_controller']['ros__parameters']
    smoother.update(max_velocity=[.18,0.,.35],min_velocity=[-.10,0.,-.35],
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
    if loaded_delivery:
        stop=monitor['VelocityStop']
        stop['translation_forward']['points']='[[0.88,0.28],[0.88,-0.28],[0.48,-0.35],[-0.37,-0.29],[-0.37,0.29],[0.48,0.35]]'
        stop['translation_reverse']['points']='[[0.82,0.26],[0.82,-0.26],[0.42,-0.29],[-0.50,-0.35],[-0.50,0.35],[0.42,0.29]]'
        stop['rotation_and_rest']['points']=params['local_costmap']['local_costmap']['ros__parameters']['footprint']
    return params


def navigation_actions(context):
    shared_navigation=LaunchConfiguration('shared_navigation').perform(context)=='true'
    loaded_delivery=shared_navigation or LaunchConfiguration('loaded_delivery').perform(context)=='true'
    private_navigation=loaded_delivery or LaunchConfiguration('delivery_teaching').perform(context)=='true'
    camera_obstacles=LaunchConfiguration('camera_obstacles').perform(context)=='true'
    params=navigation_parameters(loaded_delivery=loaded_delivery, camera_obstacles=camera_obstacles)
    root = Path(__file__).resolve().parents[3]
    gate_args = []
    if loaded_delivery:
        sys.path.insert(0, str(root/'simulation'))
        from r3_delivery_topics import delivery_velocity

        gate_args = ['--ros-args', '-p', f'wait_for_delivery_handoff:={str(not shared_navigation).lower()}',
                     '-p', f'shared_navigation:={str(shared_navigation).lower()}',
                     '-r', f'/cmd_vel_nav:={delivery_velocity("cmd_vel_nav")}',
                     '-r', f'/cmd_vel_nav_checked:={delivery_velocity("cmd_vel_nav_checked")}']
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
        if name=='bt_navigator' and private_navigation:
            # Teaching forwards RViz requests. A loaded mission owns D and
            # uses this endpoint directly, preventing public RViz preemption.
            remaps.extend([('navigate_to_pose','/pallet/r3/delivery/navigate_to_pose'),
                ('navigate_through_poses','/pallet/r3/delivery/navigate_through_poses'),
                ('goal_pose','/pallet/r3/delivery/internal_goal_pose')])
        if name in ('controller_server','behavior_server'):
            remaps.append(('cmd_vel', delivery_velocity('cmd_vel_nav') if loaded_delivery else 'cmd_vel_nav'))
        if name=='velocity_smoother':
            remaps.append(('cmd_vel', delivery_velocity('cmd_vel_nav_checked') if loaded_delivery else 'cmd_vel_nav_checked'))
            remaps.append(('cmd_vel_smoothed', delivery_velocity('cmd_vel_smoothed') if loaded_delivery else '/cmd_vel_smoothed'))
        if loaded_delivery and name=='collision_monitor':
            remaps.extend((topic, delivery_velocity(topic)) for topic in ('/cmd_vel_smoothed', '/cmd_vel_safe'))
        nodes.append(Node(package=package,executable=name,name=name,output='screen',
                          parameters=[filename],remappings=remaps))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',output='screen',parameters=[dict(use_sim_time=False,
            autostart=True,node_names=[name for _,name in specs],bond_timeout=4.)]))
    nodes.append(ExecuteProcess(cmd=[sys.executable,
        str(root/'simulation/r3_navigation_recovery_gate.py'), *gate_args],output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('delivery_teaching',default_value='false'),
        DeclareLaunchArgument('loaded_delivery',default_value='false'),
        DeclareLaunchArgument('shared_navigation',default_value='false'),
        DeclareLaunchArgument('camera_obstacles',default_value='false'),
        OpaqueFunction(function=navigation_actions)])
