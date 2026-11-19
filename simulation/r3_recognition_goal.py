#!/usr/bin/env python3
"""Save a NEW map-bound recognition A, or ask Nav2 to navigate to that A.

No legacy goal or persisted AMCL position is imported. This process publishes
no motor command; navigation uses only the NavigateToPose action.
"""
import argparse
from collections import Counter,deque
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import ParticleCloud
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from r3_core import wrap,compose,inverse
from r3_amcl_bridge import GlobalParticleGate
from r3_scan_map_check import ScanMapCheck,LIVE_SCAN_MAX_SAMPLES,validate_recognition_scan_match
from r3_stationary_localization import StationaryEvidence
from r3_amcl_observation_pair import validate_pair_evidence
from r3_amcl_quality import validate_runtime_particles
from r3_delivery_topics import PRIVATE_ACTION, check_navigation_command_owners, delivery_velocity


SCHEMA='r3_recognition_A_v1'
DEFAULT_GOAL=Path(__file__).resolve().parent/'r3/recognition_A.json'
A_PREFERRED_XY_M=.03
ARRIVAL_XY_TOLERANCE_M=.10
ARRIVAL_YAW_TOLERANCE_RAD=math.radians(5.)
A_XY_RMS_LIMIT_M=.05
A_YAW_STD_LIMIT_DEG=2.
NAV_QUALITY_HOLD_S=.5
A_QUALITY_HOLD_S=2.
NAV_DATA_RECOVERY_S=2.
MAX_A_FINALIZATION_ATTEMPTS=1


class LocalizationDataPending(ValueError):
    """An input is temporarily late; no pose is accepted while waiting."""


class MotionPredictionPending(ValueError):
    """A bounded wait for the bridge to publish its motion-onset proof."""


class MotionPredictionLimitError(ValueError):
    """Preserve measured onset limits when cancellation protects navigation."""

    def __init__(self,message,evidence):
        super().__init__(message)
        self.evidence=evidence


def map_identity(path):
    path=Path(path).resolve()
    description=yaml.safe_load(path.read_text())
    image=Path(description['image'])
    if not image.is_absolute():
        image=path.parent/image
    # Bind the goal to both map geometry and the exact occupancy image.
    image_sha=hashlib.sha256(image.read_bytes()).hexdigest()
    geometry={k:description.get(k) for k in ('resolution','origin','negate','occupied_thresh','free_thresh','mode')}
    fingerprint=hashlib.sha256(json.dumps(dict(geometry=geometry,image_sha256=image_sha),
                              sort_keys=True,allow_nan=False).encode()).hexdigest()
    return dict(map_file=str(path),fingerprint_sha256=fingerprint,image_sha256=image_sha,
                geometry=geometry)


def validate_goal(value, selected_map):
    if value.get('schema')!=SCHEMA or value.get('purpose')!='pallet_recognition_A':
        raise ValueError('Point A has not been saved with the new workflow; save a new A first')
    if value.get('frame_id')!='map' or not value.get('goal_id') or not value.get('saved_at_utc'):
        raise ValueError('Point A lacks its new save identity/map frame')
    pose=value.get('pose_xyyaw')
    if (not isinstance(pose,list) or len(pose)!=3 or any(isinstance(v,bool)
            or not isinstance(v,(int,float)) or not math.isfinite(v) for v in pose)):
        raise ValueError('Point A contains an invalid map pose')
    if (value.get('map') or {}).get('fingerprint_sha256')!=selected_map['fingerprint_sha256']:
        raise ValueError('Point A belongs to a different map; save a new A for this map')
    return tuple(pose)


def pose_error(pose, goal):
    return math.dist(pose[:2],goal[:2]),abs(wrap(pose[2]-goal[2]))


def validate_arrival(pose, goal, *, label='A'):
    xy,yaw=pose_error(pose,goal)
    if (not math.isfinite(xy) or not math.isfinite(yaw)
            or xy>=ARRIVAL_XY_TOLERANCE_M-1e-12 or yaw>ARRIVAL_YAW_TOLERANCE_RAD+1e-9):
        raise RuntimeError(f'Nav2 reported success outside {label} arrival limits '
                           f'(<{ARRIVAL_XY_TOLERANCE_M:.2f} m / {math.degrees(ARRIVAL_YAW_TOLERANCE_RAD):g} deg): '
                           f'{xy:.3f} m / {math.degrees(yaw):.2f} deg')
    return xy,yaw


def validate_a_localization(covariance, particle_metrics, recent_poses, reference,*,teaching=True):
    """Require a narrow, stable estimate before teaching or accepting A.

    These are estimator consistency limits. Only an independent measurement
    of the robot's physical pose can establish absolute accuracy. The user
    prefers about 3 cm and accepts position error/spread strictly below 5 cm.
    """
    if len(covariance)<36 or len(reference)!=3 or not all(math.isfinite(v) for v in reference):
        raise ValueError('invalid AMCL pose or covariance at A')
    variances=(covariance[0],covariance[7],covariance[35])
    if any(not math.isfinite(v) or v<0 for v in variances):
        raise ValueError('invalid AMCL covariance at A')
    # Trace of the XY covariance is the RMS Euclidean position spread;
    # checking each axis separately could exceed the radial position limit.
    xy_rms=math.sqrt(variances[0]+variances[1])
    yaw_std_deg=math.degrees(math.sqrt(variances[2]))
    if xy_rms>=A_XY_RMS_LIMIT_M-1e-12 or yaw_std_deg>A_YAW_STD_LIMIT_DEG+1e-9:
        raise ValueError(f'AMCL uncertainty at A exceeds allowed limits (<{A_XY_RMS_LIMIT_M*100:g} cm / 2 deg) '
                         f'({xy_rms*100:.1f} cm / {yaw_std_deg:.2f} deg)')
    if (not isinstance(particle_metrics,dict)
            or not all(isinstance(particle_metrics.get(k),(int,float))
                       and math.isfinite(particle_metrics[k])
                       for k in ('xy_rms_m','yaw_std_deg'))
            or particle_metrics['xy_rms_m']>=A_XY_RMS_LIMIT_M-1e-12
            or particle_metrics['yaw_std_deg']>A_YAW_STD_LIMIT_DEG+1e-9):
        raise ValueError(f'AMCL particles at A require <{A_XY_RMS_LIMIT_M*100:g} cm / 2 deg spread')
    if teaching:
        if (particle_metrics.get('unique_pose_bins',0)<10
                or particle_metrics.get('largest_pose_bin_mass',1.)>.5):
            raise ValueError('AMCL particles collapsed to too few distinct poses; uncertainty is unreliable')
    else:
        validate_runtime_particles(particle_metrics)
    if len(recent_poses)<3:
        raise ValueError('need at least 3 fresh pose/scan observations before saving/starting')
    if any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in recent_poses):
        raise ValueError('invalid recent AMCL pose at A')
    if any(math.dist(p[:2],reference[:2])>=A_XY_RMS_LIMIT_M-1e-12
           or abs(wrap(p[2]-reference[2]))>math.radians(A_YAW_STD_LIMIT_DEG)+1e-9
           for p in recent_poses):
        raise ValueError(f'AMCL pose estimate must remain within <{A_XY_RMS_LIMIT_M*100:g} cm / {A_YAW_STD_LIMIT_DEG:g} deg')
    return dict(amcl_xy_rms_m=xy_rms,amcl_yaw_std_deg=yaw_std_deg,
                particle_xy_rms_m=particle_metrics['xy_rms_m'],
                particle_yaw_std_deg=particle_metrics['yaw_std_deg'])


def validate_a_scan_match(match,*,teaching=True):
    """Stricter map agreement for A than the basic tracking health gate."""
    if not teaching:
        return validate_recognition_scan_match(match)
    ratio=match.get('inlier_ratio') if isinstance(match,dict) else None
    median=match.get('median_distance_m') if isinstance(match,dict) else None
    if (not isinstance(ratio,(int,float)) or not math.isfinite(ratio) or ratio<.80
            or not isinstance(median,(int,float)) or not math.isfinite(median) or median>.05+1e-9):
        raise ValueError('A scan/map agreement requires >=80% inliers and median wall distance <=5 cm')


def validate_map_update(previous,current):
    if previous is None:
        return
    change=compose(inverse(previous),current)
    if math.hypot(*change[:2])>.15 or abs(change[2])>math.radians(5.):
        raise ValueError('AMCL map correction jumped during navigation; cancel and localize again')


def yaw_of(q):
    n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if not math.isfinite(n) or abs(n-1.)>.02:
        raise ValueError('invalid orientation')
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def atomic_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


def create_goal_json(path,value):
    """Publish a complete goal atomically without ever replacing another A."""
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.'+str(uuid.uuid4())+'.tmp')
    try:
        temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
        os.link(temporary,path)
    finally:
        temporary.unlink(missing_ok=True)


class RecognitionGoal(Node):
    def __init__(self,request_nomotion=False,node_name='r3_recognition_goal',
                 selected_map_file=None,*,context=None,continuous_localization=False,
                 navigation_action='/navigate_to_pose',shared_navigation=False):
        super().__init__(node_name,context=context)
        self.amcl=None
        self.odom=None
        self.stationary_since=None
        self.observations=deque(maxlen=30)
        self.tf=Buffer(cache_time=Duration(seconds=10.))
        self.listener=TransformListener(self.tf,self)
        self.client=ActionClient(self,NavigateToPose,navigation_action)
        self.goal_handle=None
        self.result_future=None
        self.last_map_odom=None
        self.cloud=None
        self.scan_stamp=None
        self.scan=None
        self.map_check=None
        self.map_match=None
        self.particle_gate=GlobalParticleGate()
        self.nomotion=self.create_client(Empty,'/request_nomotion_update')
        self.nomotion_future=None
        self.next_nomotion=0.
        self.continuous_localization=bool(continuous_localization)
        # A late subscriber must first drain map/TF startup work and establish
        # its own fresh sensor baseline. Capture continuity is enforced after
        # that point, rather than across messages queued before TF was usable.
        self.stationary_startup_pending=self.continuous_localization
        self.request_nomotion=request_nomotion and not self.continuous_localization
        self.a_quality=None
        self.selected_map_file=(str(Path(selected_map_file).resolve())
                                if selected_map_file is not None else None)
        self.stationary_mode=False
        self.stationary_status=None
        self.stationary_snapshot=None
        self.stationary_amcl=None
        self.stationary_error=None
        self.stationary_evidence=StationaryEvidence()
        self.stationary_validated_poses=deque(maxlen=60)
        self.map_signature=None
        self.stationary_fit_key=None
        self.last_validated_scan=None
        self.last_live_amcl=None
        self.navigation_run_id=None
        self.navigation_goal_attempted=False
        self.last_parked_validation=None
        self.motion_prediction_identity=None
        self.current_localization_mode=None
        self.own_motion_onset=None
        self.pending_prediction_identity=None
        self.readiness_diagnostics={}
        self.a_scan_window_start=None
        self.a_scan_window_last=None
        self.a_scan_window_count=0
        self.a_scan_quality_reason='waiting for continuous strict A scan/map evidence'
        self._spin_executor=None
        self.navigation_data_waits=[]
        self.navigation_action_active=False
        self.navigation_map_signature=None
        self.navigation_observation_stamp=None
        self.navigation_result=None
        self.navigation_finalization=[]
        self.shared_navigation=shared_navigation
        self.base_output=None
        self.motion_enabled=False
        self.forwarded_velocity_commands=0
        self.next_a_graph_check=0.
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        if not self.continuous_localization:
            self.create_subscription(ParticleCloud,'/particle_cloud',self.on_particles,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan_filtered',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/pallet/r3/localization/status',self.on_localization_status,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05,self.poll_stationary_scan)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def on_amcl(self,msg):
        try:
            if msg.header.frame_id!='map':
                raise ValueError('AMCL pose is not in map')
            c=msg.pose.covariance
            if (any(not math.isfinite(c[i]) or c[i]<0 for i in (0,7,35))
                    or max(c[0],c[7])>.10**2 or c[35]>math.radians(5.)**2):
                raise ValueError('AMCL has not converged')
            p=msg.pose.pose.position
            pose=(p.x,p.y,yaw_of(msg.pose.pose.orientation))
            if not all(math.isfinite(v) for v in pose):
                raise ValueError('invalid AMCL pose')
            if self.observations and self.stamp(msg)<=self.observations[-1][0]:
                return
            self.amcl=msg
            self.last_live_amcl=msg
            self.observations.append((self.stamp(msg),pose))
        except ValueError:
            self.amcl=None
            self.observations.clear()

    def on_odom(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id not in ('base_footprint','base_link'):
            self.stationary_evidence.invalidate('invalid stationary odom frame')
            self._reset_a_scan_quality('invalid stationary odom frame')
            self.odom=None;self.stationary_since=None;return
        twist=msg.twist.twist
        try:
            raw=msg.pose.pose
            pose=(raw.position.x,raw.position.y,yaw_of(raw.orientation))
            velocity=(twist.linear.x,twist.linear.y,twist.angular.z)
            if not all(math.isfinite(v) for v in (*pose,*velocity)):
                raise ValueError('nonfinite odom')
        except ValueError:
            self.stationary_evidence.invalidate('invalid stationary odom pose/twist')
            self._reset_a_scan_quality('invalid stationary odom pose/twist')
            self.odom=None;self.stationary_since=None;return
        stamp=self.stamp(msg)
        previous=self.stationary_evidence.odom_stamp
        if self.continuous_localization and previous is not None and stamp-previous>.5:
            self._restart_local_scan_window('local odom delivery interrupted; collect fresh scan evidence')
        self.odom=msg
        self.stationary_evidence.update_odom(pose,velocity,stamp,self.now(),
                                            check_capture_continuity=not self.continuous_localization)
        if self.stationary_evidence.fault:
            self._reset_a_scan_quality(self.stationary_evidence.fault)
        still=math.hypot(*velocity[:2])<.005 and abs(velocity[2])<.01
        if not still:
            if (self.continuous_localization and self.own_motion_onset is None
                    and self.stationary_snapshot is not None):
                self.own_motion_onset=dict(started_ros_s=self.now(),
                    source_amcl_stamp_s=self.stationary_snapshot['amcl_stamp_s'])
            self.stationary_since=None
        elif self.stationary_since is None:
            self.stationary_since=time.monotonic()

    def on_scan(self,msg):
        if msg.header.frame_id=='lidar_link':
            stamp=self.stamp(msg)
            previous=self.scan_stamp
            if (self.continuous_localization and self.stationary_snapshot is not None
                    and not getattr(self,'stationary_startup_pending',False)
                    and previous is not None):
                if stamp<previous:
                    reason='LiDAR timestamp moved backwards; a new AMCL observation is required'
                    self.stationary_evidence.invalidate(reason)
                    self._restart_local_scan_window(reason)
                elif stamp-previous>.5:
                    # Missing callbacks on this best-effort subscription do
                    # not establish that the supervisor's source was lost.
                    self._restart_local_scan_window('local LiDAR delivery interrupted; collect fresh scan evidence')
            self.scan_stamp=stamp
            self.scan=msg
            if not self.continuous_localization:
                self.poll_stationary_scan()
        elif self.stationary_mode:
            self.scan=None;self.scan_stamp=None
            self.stationary_evidence.invalidate('unexpected stationary LiDAR frame')
            self._reset_a_scan_quality('unexpected stationary LiDAR frame')

    def _restart_local_scan_window(self,reason):
        self.stationary_evidence.restart_scan_window(reason)
        self.stationary_validated_poses.clear()
        self.last_validated_scan=None
        self.stationary_fit_key=None
        self._reset_a_scan_quality(reason)

    def poll_stationary_scan(self):
        # TF can arrive a few milliseconds after a scan callback. Retry on a
        # short timer rather than repeatedly treating that scheduling order as
        # a scan mismatch or waiting for the slower display report interval.
        if (not self.navigation_action_active and self.stationary_mode
                and self.stationary_snapshot is not None):
            try:
                self._score_stationary_scan(self.now())
            except (ValueError,TransformException):
                pass

    def on_map(self,msg):
        try:
            check=ScanMapCheck(msg)
            signature=(check.width,check.height,check.resolution,check.origin,
                       check.cos_yaw,check.sin_yaw,check.data)
            if self.map_signature is not None and signature!=self.map_signature:
                self.stationary_evidence.invalidate('occupancy map changed; new AMCL snapshot required')
                self.stationary_validated_poses.clear()
                self._reset_a_scan_quality('occupancy map changed')
                self.last_validated_scan=None
            self.map_signature=signature
            self.map_check=check
        except ValueError:
            self.map_check=None
            self.stationary_evidence.invalidate('invalid selected occupancy map')
            self._reset_a_scan_quality('invalid selected occupancy map')

    def on_localization_status(self,msg):
        if self.request_nomotion:
            return
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):
                raise ValueError('invalid localization status')
            cache=value.get('stationary_cache')
            continuous=self.continuous_localization and value.get('continuous_localization') is True
            if not ((value.get('stationary_inspection') is True or continuous)
                    and isinstance(cache,dict) and cache.get('enabled') is True):
                if self.stationary_mode:
                    raise ValueError('stationary localization session changed; new snapshot required')
                return
            self.stationary_mode=True
            self.stationary_status=value
            snapshot=self._validate_stationary_snapshot(value)
            previous=self.stationary_snapshot
            if previous is not None:
                same_session=previous['run_id']==snapshot['run_id']
                if same_session and snapshot['amcl_stamp_s']<previous['amcl_stamp_s']:
                    raise ValueError('stationary AMCL snapshot timestamp moved backwards')
                if (same_session and snapshot['amcl_stamp_s']==previous['amcl_stamp_s']
                        and snapshot!=previous):
                    raise ValueError('stationary AMCL snapshot changed without a new observation')
                if snapshot==previous:
                    self.stationary_error=None
                    return
                if not same_session:
                    self.last_validated_scan=None
                    self.stationary_startup_pending=self.continuous_localization
                    self.stationary_evidence=StationaryEvidence()
                    if self.odom is not None:
                        raw=self.odom.pose.pose;twist=self.odom.twist.twist
                        self.stationary_evidence.update_odom(
                            (raw.position.x,raw.position.y,yaw_of(raw.orientation)),
                            (twist.linear.x,twist.linear.y,twist.angular.z),self.stamp(self.odom),self.now())
            self.stationary_snapshot=copy.deepcopy(snapshot)
            self.stationary_fit_key=None
            self.stationary_validated_poses.clear()
            self._reset_a_scan_quality('new stationary AMCL snapshot')
            if not getattr(self,'stationary_startup_pending',False):
                self.stationary_evidence.capture(snapshot['amcl_stamp_s'],
                                                  snapshot['odom_anchor_xyyaw'],snapshot['particle_metrics'])
            original=PoseWithCovarianceStamped()
            original.header.frame_id='map'
            original.header.stamp=Time(nanoseconds=round(snapshot['amcl_stamp_s']*1e9)).to_msg()
            x,y,yaw=snapshot['map_pose_xyyaw']
            original.pose.pose.position.x,original.pose.pose.position.y=x,y
            original.pose.pose.orientation.z=math.sin(yaw/2.)
            original.pose.pose.orientation.w=math.cos(yaw/2.)
            original.pose.covariance=snapshot['covariance_6x6']
            self.stationary_amcl=original
            self.stationary_error=None
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            if self.stationary_mode:
                self.stationary_error=str(exc)
                self.stationary_evidence.invalidate(str(exc))
                self._reset_a_scan_quality(str(exc))

    def _validate_stationary_snapshot(self,status):
        snapshot=status.get('stationary_snapshot')
        if not isinstance(snapshot,dict):
            raise ValueError('waiting for stationary AMCL snapshot')
        if (not isinstance(snapshot.get('run_id'),str) or not snapshot['run_id']
                or snapshot['run_id']!=status.get('run_id')
                or not isinstance(snapshot.get('map_file'),str)
                or snapshot['map_file']!=status.get('map_file')):
            raise ValueError('stationary snapshot does not belong to the active map/session')
        if (self.selected_map_file is not None
                and str(Path(snapshot['map_file']).resolve())!=self.selected_map_file):
            raise ValueError('stationary snapshot belongs to a different selected map')
        for name,length in (('map_pose_xyyaw',3),('odom_anchor_xyyaw',3),
                            ('map_odom_xyyaw',3),('covariance_6x6',36)):
            data=snapshot.get(name)
            if (not isinstance(data,list) or len(data)!=length
                    or not all(isinstance(v,(int,float)) and not isinstance(v,bool)
                               and math.isfinite(v) for v in data)):
                raise ValueError('invalid stationary snapshot '+name)
        covariance=snapshot['covariance_6x6']
        if (any(covariance[i]<0 for i in (0,7,14,21,28,35))
                or max(covariance[0],covariance[7])>.10**2
                or covariance[35]>math.radians(5.)**2):
            raise ValueError('stationary AMCL covariance has not converged')
        for name in ('amcl_stamp_s','cloud_stamp_s'):
            stamp=snapshot.get(name)
            if (not isinstance(stamp,(int,float)) or isinstance(stamp,bool)
                    or not math.isfinite(stamp) or stamp<0 or stamp>self.now()+.05):
                raise ValueError('invalid stationary '+name)
        validate_pair_evidence(snapshot)
        metrics=snapshot.get('particle_metrics')
        if (not isinstance(metrics,dict) or metrics.get('converged') is not True
                or not all(isinstance(metrics.get(name),(int,float))
                    and not isinstance(metrics[name],bool) and math.isfinite(metrics[name])
                    and metrics[name]>=0 for name in ('xy_rms_m','yaw_std_deg'))):
            raise ValueError('stationary AMCL particle evidence is invalid/ambiguous')
        reconstructed=compose(snapshot['map_odom_xyyaw'],snapshot['odom_anchor_xyyaw'])
        reference=snapshot['map_pose_xyyaw']
        if (math.dist(reconstructed[:2],reference[:2])>1e-6
                or abs(wrap(reconstructed[2]-reference[2]))>1e-6):
            raise ValueError('stationary snapshot map/odom transforms disagree')
        cache=status['stationary_cache']
        if (cache.get('original_covariance_preserved') is not True
                or cache.get('repeated_nomotion_updates') is not False):
            raise ValueError('stationary snapshot requires preserved covariance and no repeated AMCL updates')
        return snapshot

    def _require_stationary_status(self,now):
        if self.stationary_error:
            raise ValueError(self.stationary_error)
        status=self.stationary_status
        stamp=status.get('published_ros_s') if isinstance(status,dict) else None
        if (not isinstance(stamp,(int,float)) or not math.isfinite(stamp)
                or not -.05<=now-stamp<=.5):
            raise LocalizationDataPending('stationary localization status is missing/stale')
        if self.stationary_snapshot is None or self.stationary_amcl is None:
            raise ValueError('waiting for stationary AMCL snapshot')
        if (self.continuous_localization and self.navigation_run_id is not None
                and status.get('run_id')!=self.navigation_run_id):
            raise ValueError('localization session changed during navigation')
        if (self.selected_map_file is not None
                and str(Path(self.stationary_snapshot['map_file']).resolve())!=self.selected_map_file):
            raise ValueError('stationary snapshot belongs to a different selected map')

    def _score_stationary_scan(self,now):
        self._require_stationary_status(now)
        if self.odom is None or not -.05<=now-self.stamp(self.odom)<=.20:
            raise LocalizationDataPending('fresh EKF odom unavailable')
        if self.scan is None or self.scan_stamp is None or not -.05<=now-self.scan_stamp<=.5:
            raise LocalizationDataPending('fresh LiDAR scan unavailable')
        if self.map_check is None:
            raise ValueError('selected occupancy map unavailable for scan validation')
        map_odom=self.stationary_snapshot['map_odom_xyyaw']
        scan=self.scan
        try:
            lidar=self.tf.lookup_transform('odom',scan.header.frame_id,Time.from_msg(scan.header.stamp))
            t,q=lidar.transform.translation,lidar.transform.rotation
            odom_sensor=(t.x,t.y,yaw_of(q))
        except TransformException as exc:
            previous=self.last_validated_scan
            if (not self.continuous_localization or previous is None
                    or not -.05<=now-self.stamp(previous[0])<=.5):
                raise LocalizationDataPending('stationary scan/map transform unavailable: '+str(exc)) from exc
            # An exact capture-time TF may arrive just after its scan. The
            # previous already transformed scan remains evidence only within
            # its normal 0.5 s freshness bound. It is re-scored if AMCL changed.
            scan,odom_sensor=previous
        except ValueError as exc:
            raise ValueError('stationary scan/map transform unavailable: '+str(exc)) from exc
        scan_stamp=self.stamp(scan)
        key=(scan_stamp,self.stationary_snapshot['amcl_stamp_s'],id(self.map_check))
        if key!=self.stationary_fit_key:
            sensor_pose=compose(map_odom,odom_sensor)
            try:
                options=({'max_samples':LIVE_SCAN_MAX_SAMPLES} if self.continuous_localization else {})
                self.map_match=self.map_check.score(scan,sensor_pose,**options)
            except ValueError:
                self.last_validated_scan=None
                self._reset_a_scan_quality('invalid LiDAR/map geometry')
                raise
            self.stationary_fit_key=key
            self._observe_a_scan_quality(scan_stamp,self.map_match)
            if self.map_match.get('ready',False):
                self.last_validated_scan=(scan,odom_sensor)
            else:
                # A real mismatch must not be hidden by the older good scan.
                self.last_validated_scan=None
        self.map_match['scan_stamp_s']=scan_stamp
        if getattr(self,'stationary_startup_pending',False):
            status=self.stationary_status
            if (status.get('ready') is not True
                    or status.get('stationary_cache',{}).get('validated') is not True):
                raise ValueError('waiting for bridge-validated stationary startup evidence')
            if not self.map_match.get('ready',False):
                raise ValueError(self.map_match.get('reason','startup scan does not fit the map'))
            # _require_stationary_status, exact scan TF, the current map fit
            # and fresh own odom/scan have all passed. Only now adopt the
            # original AMCL observation as this reader's continuity baseline.
            # No stamp/covariance is changed and no AMCL refresh is requested.
            self.stationary_evidence.capture(self.stationary_snapshot['amcl_stamp_s'],
                self.stationary_snapshot['odom_anchor_xyyaw'],
                self.stationary_snapshot['particle_metrics'])
            self.stationary_startup_pending=False
        scan_options=({'check_capture_continuity':False} if self.continuous_localization else {})
        self.stationary_evidence.observe_scan(scan_stamp,self.map_match,now,**scan_options)
        if not self.map_match.get('ready',False):
            self.stationary_validated_poses.clear()
            raise ValueError(self.map_match['reason'])
        raw=self.odom.pose.pose
        pose=compose(map_odom,(raw.position.x,raw.position.y,yaw_of(raw.orientation)))
        if (not self.stationary_validated_poses
                or scan_stamp>self.stationary_validated_poses[-1][0]):
            self.stationary_validated_poses.append((scan_stamp,pose))
        return pose

    def _current_continuous_pose(self,now,require_still):
        if require_still:
            return self._current_stationary_pose(now,True)
        self._require_stationary_status(now)
        status=self.stationary_status
        if status.get('continuous_localization') is not True:
            raise ValueError('continuous localization bridge is unavailable')
        pose=self._score_stationary_scan(now)
        cache=status.get('stationary_cache',{})
        local_parked=self.stationary_evidence.ready(now)[0]
        if cache.get('validated') is True and local_parked:
            return self._current_stationary_pose(now,False)
        metrics=self.stationary_snapshot['particle_metrics']
        validate_runtime_particles(metrics)
        original=self.stationary_snapshot['amcl_stamp_s']
        prediction=False
        if status.get('motion_prediction_validated') is True:
            self._validate_motion_prediction(now)
            prediction=True
        if not prediction and not -.05<=now-original<=2.:
            self._wait_for_motion_prediction(now)
            raise LocalizationDataPending('live AMCL observation is missing/stale during navigation')
        if (status.get('ready') is not True
                or (not prediction and status.get('live_observation_validated') is not True)):
            self._wait_for_motion_prediction(now)
            raise LocalizationDataPending('bridge is not validating live AMCL during navigation')
        map_odom=tuple(self.stationary_snapshot['map_odom_xyyaw'])
        if (self.goal_handle is not None and self.goal_handle.accepted
                and self.result_future is not None and not self.result_future.done()):
            validate_map_update(self.last_map_odom,map_odom)
        self.last_map_odom=map_odom
        self.amcl=self.stationary_amcl
        self.particle_gate.metrics=copy.deepcopy(metrics)
        self.current_localization_mode='bounded_motion_prediction' if prediction else 'fresh_live_amcl'
        return pose

    def _wait_for_motion_prediction(self,now):
        """Wait at most 150 ms for a peer's next status, never accept a pose."""
        status=self.stationary_status
        parked=self.last_parked_validation
        onset=self.own_motion_onset
        if (status.get('ready') is not True
                or status.get('stationary_cache',{}).get('validated') is not True
                or parked is None or onset is None):
            return
        original=self.stationary_snapshot['amcl_stamp_s']
        if (parked['run_id']!=status.get('run_id')
                or abs(parked['source_amcl_stamp_s']-original)>1e-6
                or abs(onset['source_amcl_stamp_s']-original)>1e-6
                or not -.05<=onset['started_ros_s']-parked['validated_ros_s']<=.25):
            return
        fault=self.stationary_evidence.fault
        if fault is not None and not fault.startswith(('robot motion','robot displacement')):
            return
        raw=self.odom.pose.pose
        actual=(raw.position.x,raw.position.y,yaw_of(raw.orientation))
        if (math.dist(actual[:2],parked['odom_xyyaw'][:2])>.08
                or abs(wrap(actual[2]-parked['odom_xyyaw'][2]))>.08):
            return
        identity=(status['run_id'],original,onset['started_ros_s'],tuple(parked['odom_xyyaw']))
        if self.pending_prediction_identity is None:
            self.pending_prediction_identity=identity
        elif self.pending_prediction_identity!=identity:
            return
        if -.05<=now-onset['started_ros_s']<.15:
            raise MotionPredictionPending('waiting for the bridge motion-onset proof (maximum 0.15 s)')
        raise ValueError('bridge motion-onset proof did not arrive within 0.15 seconds')

    def _validate_motion_prediction(self,now):
        """Independently bound the bridge's one parked-to-moving prediction."""
        status=self.stationary_status
        proof=status.get('motion_prediction')
        if not isinstance(proof,dict) or proof.get('validated') is not True:
            raise ValueError('motion prediction has no validated transition proof')
        if proof.get('ended') is True:
            raise ValueError('motion prediction transition was already ended')
        fault=self.stationary_evidence.fault
        if fault is not None and not fault.startswith(('robot motion','robot displacement')):
            raise ValueError('motion prediction blocked by '+fault)
        numeric=('started_ros_s','source_amcl_stamp_s','elapsed_s','displacement_m','yaw_change_rad',
                 'max_duration_s','max_distance_m','max_yaw_rad')
        if not all(isinstance(proof.get(k),(int,float)) and not isinstance(proof[k],bool)
                   and math.isfinite(proof[k]) for k in numeric):
            raise ValueError('motion prediction contains invalid numeric evidence')
        for key,limit in (('max_duration_s',2.),('max_distance_m',.08),('max_yaw_rad',.08)):
            if not math.isclose(proof[key],limit,rel_tol=0.,abs_tol=1e-12):
                raise ValueError('motion prediction fixed bounds were changed')
        anchor=proof.get('anchor_odom_xyyaw')
        if (not isinstance(anchor,list) or len(anchor)!=3 or not all(
                isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in anchor)):
            raise ValueError('motion prediction odom anchor is invalid')
        original=self.stationary_snapshot['amcl_stamp_s']
        if abs(proof['source_amcl_stamp_s']-original)>1e-6:
            raise ValueError('motion prediction source differs from the original AMCL observation')
        parked=self.last_parked_validation
        if (parked is None or parked['run_id']!=status.get('run_id')
                or abs(parked['source_amcl_stamp_s']-original)>1e-6
                or not -.05<=proof['started_ros_s']-parked['validated_ros_s']<=.25):
            raise ValueError('motion prediction lacks a recent independently validated parked pose')
        if (math.dist(anchor[:2],parked['odom_xyyaw'][:2])>.005
                or abs(wrap(anchor[2]-parked['odom_xyyaw'][2]))>math.radians(.5)):
            raise ValueError('motion prediction anchor differs from the validated parked odom')
        elapsed=now-proof['started_ros_s']
        if not -.05<=elapsed<=2. or not 0.<=proof['elapsed_s']<=2.:
            raise ValueError('motion prediction exceeded its two-second transition window')
        if abs((status['published_ros_s']-proof['started_ros_s'])-proof['elapsed_s'])>.05:
            raise ValueError('motion prediction elapsed time does not match the status timestamp')
        raw=self.odom.pose.pose
        actual=(raw.position.x,raw.position.y,yaw_of(raw.orientation))
        distance=math.dist(actual[:2],anchor[:2]);angle=abs(wrap(actual[2]-anchor[2]))
        if (distance>.08 or angle>.08 or not 0.<=proof['displacement_m']<=.08
                or not 0.<=proof['yaw_change_rad']<=.08):
            evidence=dict(run_id=status.get('run_id'),current_ros_s=now,
                source_amcl_stamp_s=original,source_amcl_age_s=now-original,
                status_stamp_ros_s=status['published_ros_s'],
                status_age_s=now-status['published_ros_s'],
                odom_stamp_ros_s=self.stamp(self.odom),
                anchor_odom_xyyaw=list(anchor),current_odom_xyyaw=list(actual),
                own_displacement_m=distance,own_yaw_change_rad=angle,
                own_yaw_change_deg=math.degrees(angle),
                elapsed_s=elapsed,max_duration_s=2.,max_distance_m=.08,
                max_yaw_rad=.08,max_yaw_deg=math.degrees(.08),
                amcl_pipeline=copy.deepcopy(status.get('amcl_pipeline')),
                bridge_transition_proof=copy.deepcopy(proof))
            raise MotionPredictionLimitError(
                'motion prediction exceeded its distance or heading transition bound '
                f'(odom {distance:.3f} m / {math.degrees(angle):.2f} deg; '
                f'limits 0.080 m / {math.degrees(.08):.2f} deg; '
                f'AMCL source age {now-original:.2f} s)',evidence)
        identity=(status['run_id'],original,proof['started_ros_s'],tuple(anchor))
        previous=self.motion_prediction_identity
        if previous is not None and previous[:2]==identity[:2] and previous!=identity:
            raise ValueError('motion prediction was rearmed without a new AMCL observation')
        self.motion_prediction_identity=identity
        return True

    def _current_stationary_pose(self,now,require_still):
        pose=self._score_stationary_scan(now)
        if (self.stationary_status.get('ready') is not True
                or self.stationary_status['stationary_cache'].get('validated') is not True):
            raise ValueError('bridge is not validating the stationary AMCL snapshot')
        ready,reason=self.stationary_evidence.ready(now)
        if not ready:
            raise ValueError(reason)
        if require_still and (self.stationary_since is None or time.monotonic()-self.stationary_since<.5):
            raise ValueError('robot must stand still for at least 0.5 seconds')
        self.amcl=self.stationary_amcl
        self.last_map_odom=tuple(self.stationary_snapshot['map_odom_xyyaw'])
        self.particle_gate.metrics=copy.deepcopy(self.stationary_snapshot['particle_metrics'])
        if self.continuous_localization:
            raw=self.odom.pose.pose
            self.last_parked_validation=dict(validated_ros_s=now,run_id=self.stationary_status['run_id'],
                source_amcl_stamp_s=self.stationary_snapshot['amcl_stamp_s'],
                odom_xyyaw=[raw.position.x,raw.position.y,yaw_of(raw.orientation)])
            self.current_localization_mode='validated_parked_snapshot'
            self.own_motion_onset=None
            self.pending_prediction_identity=None
        return pose

    def recent_a_poses(self):
        values=(self.stationary_validated_poses if self.stationary_mode
                else self.observations)
        return [pose for stamp,pose in values if -.05<=self.now()-stamp<=2.]

    def _reset_a_scan_quality(self,reason):
        self.a_scan_window_start=None
        self.a_scan_window_last=None
        self.a_scan_window_count=0
        self.a_scan_quality_reason=reason

    def _observe_a_scan_quality(self,stamp,fit):
        previous=self.a_scan_window_last
        if previous is not None and stamp==previous:
            return
        if previous is not None and (stamp<previous or stamp-previous>.5):
            self._reset_a_scan_quality('strict A scan sequence had a gap/clock change')
        self.a_scan_window_last=stamp
        try:
            if not fit.get('ready',False):
                raise ValueError('scan/map tracking gate rejected the scan')
            validate_a_scan_match(fit,teaching=not getattr(self,'continuous_localization',False))
        except ValueError as exc:
            self.a_scan_window_start=None
            self.a_scan_window_count=0
            self.a_scan_quality_reason=str(exc)
            return
        if self.a_scan_window_start is None:
            self.a_scan_window_start=stamp
        self.a_scan_window_count+=1
        self.a_scan_quality_reason='strict A scan/map agreement maintained on distinct live scans'

    def a_scan_quality_duration(self):
        if not self.stationary_mode:
            return None
        if self.stationary_evidence.fault:
            self._reset_a_scan_quality(self.stationary_evidence.fault)
            return 0.
        last=self.a_scan_window_last
        if last is None or not -.05<=self.now()-last<=.5:
            self._reset_a_scan_quality('strict A scan sequence is missing/stale')
            return 0.
        if self.a_scan_window_start is None or self.a_scan_window_count<3:
            return 0.
        return max(0.,last-self.a_scan_window_start)

    def a_scan_quality_ready(self):
        return not self.stationary_mode or self.a_scan_quality_duration()>=self.required_quality_hold_s()-1e-9

    def required_quality_hold_s(self):
        # Navigation already requires the bridge's independent one-second
        # parked evidence window. A short additional stable quality hold
        # avoids blocking a robot already at A on isolated noisy scan fits.
        # Teaching A and its standalone watcher retain the longer hold.
        return NAV_QUALITY_HOLD_S if getattr(self,'continuous_localization',False) else A_QUALITY_HOLD_S

    def on_particles(self,msg):
        if self.continuous_localization:
            # Continuous A readers use the bridge's original paired cloud
            # evidence. Parsing a second 2000..4000-particle cloud here was
            # redundant work on the Pi, including throughout native travel.
            return
        try:
            if msg.header.frame_id!='map':
                raise ValueError('particle cloud is not in map')
            self.cloud=(self.stamp(msg),[(p.pose.position.x,p.pose.position.y,
                        yaw_of(p.pose.orientation),p.weight) for p in msg.particles])
        except ValueError:
            self.cloud=None
            self.particle_gate.healthy=False

    def spin_once(self):
        wall=time.monotonic()
        if self.nomotion_future is not None and self.nomotion_future.done():
            self.nomotion_future=None
        if (self.request_nomotion and self.stationary_since is not None and wall>=self.next_nomotion
                and self.nomotion_future is None and self.nomotion.service_is_ready()):
            self.nomotion_future=self.nomotion.call_async(Empty.Request())
            self.next_nomotion=wall+.5
        if self._spin_executor is None:
            self._spin_executor=SingleThreadedExecutor(context=self.context)
            self._spin_executor.add_node(self)
        # Keep the executor registered and service ready subscriptions before
        # inspecting their ages. One callback followed by a fatal health check
        # could inspect old odom while a fresh message was already queued
        # behind TF, a particle cloud or a scan-scoring callback.
        self._spin_executor.spin_once(timeout_sec=.01)
        drain_until=time.monotonic()+.02
        for _ in range(16):
            if time.monotonic()>=drain_until:
                break
            self._spin_executor.spin_once(timeout_sec=0.)

    def destroy_node(self):
        if self._spin_executor is not None:
            self._spin_executor.remove_node(self)
            self._spin_executor.shutdown(timeout_sec=1.)
            self._spin_executor=None
        return super().destroy_node()

    def current_pose(self,require_still=False):
        now=self.now()
        if self.continuous_localization:
            return self._current_continuous_pose(now,require_still)
        if not self.request_nomotion and self.stationary_mode:
            return self._current_stationary_pose(now,require_still)
        if self.amcl is None or not -.05<=now-self.stamp(self.amcl)<=2.:
            raise ValueError('fresh converged AMCL pose unavailable')
        if self.odom is None or not -.05<=now-self.stamp(self.odom)<=.20:
            raise ValueError('fresh EKF odom unavailable')
        if self.scan_stamp is None or not -.05<=now-self.scan_stamp<=.5:
            raise ValueError('fresh LiDAR scan unavailable')
        if self.cloud is None or abs(self.cloud[0]-self.stamp(self.amcl))>.25:
            raise ValueError('fresh particle cloud matching AMCL pose unavailable')
        p=self.amcl.pose.pose.position
        reference=(p.x,p.y,yaw_of(self.amcl.pose.pose.orientation))
        self.particle_gate.accept(self.cloud[1],reference,self.cloud[0],now)
        ready,reason=self.particle_gate.ready(now)
        if not ready:
            raise ValueError(reason)
        if require_still and (self.stationary_since is None or time.monotonic()-self.stationary_since<.5):
            raise ValueError('robot must stand still for at least 0.5 seconds')
        # Derive map->odom from AMCL's exact observation time; its broadcast
        # transform is future-dated and may otherwise interpolate old data.
        transform=self.tf.lookup_transform('odom','base_footprint',Time.from_msg(self.amcl.header.stamp))
        t,q=transform.transform.translation,transform.transform.rotation
        map_odom=compose(reference,inverse((t.x,t.y,yaw_of(q))))
        if (self.goal_handle is not None and self.goal_handle.accepted
                and self.result_future is not None and not self.result_future.done()):
            validate_map_update(self.last_map_odom,map_odom)
        self.last_map_odom=map_odom
        raw=self.odom.pose.pose
        pose=compose(map_odom,(raw.position.x,raw.position.y,yaw_of(raw.orientation)))
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('nonfinite map transform')
        if self.map_check is None:
            raise ValueError('selected occupancy map unavailable for scan validation')
        lidar=self.tf.lookup_transform('odom',self.scan.header.frame_id,Time.from_msg(self.scan.header.stamp))
        t,q=lidar.transform.translation,lidar.transform.rotation
        sensor_pose=compose(map_odom,(t.x,t.y,yaw_of(q)))
        self.map_match=self.map_check.score(self.scan,sensor_pose)
        if not self.map_match['ready']:
            raise ValueError(self.map_match['reason'])
        return pose

    def wait_pose(self,timeout,require_still=True):
        deadline=time.monotonic()+timeout
        self.last_stationary_candidate=None
        hold_s=self.required_quality_hold_s()
        issue='waiting for localization'
        stable_since=None
        failures=Counter()
        longest_good_s=0.
        last_failure=None
        while rclpy.ok() and time.monotonic()<deadline:
            self.spin_once()
            try:
                pose=self.current_pose(require_still)
                if require_still:
                    self.last_stationary_candidate=dict(pose_xyyaw=list(pose),
                        captured_ros_s=self.now(),scan_map_match=copy.deepcopy(self.map_match))
                    validate_a_scan_match(self.map_match,
                        teaching=not getattr(self,'continuous_localization',False))
                    recent=self.recent_a_poses()
                    self.a_quality=validate_a_localization(self.amcl.pose.covariance,
                        self.particle_gate.metrics,recent,pose,
                        teaching=not getattr(self,'continuous_localization',False))
                    if stable_since is None:
                        stable_since=time.monotonic()
                    longest_good_s=max(longest_good_s,time.monotonic()-stable_since)
                    if not self.a_scan_quality_ready():
                        issue=f'waiting for strict A scan/map agreement on every distinct scan for {hold_s:g} seconds'
                        continue
                    if time.monotonic()-stable_since<hold_s:
                        issue=f'waiting for A quality to remain valid for {hold_s:g} seconds'
                        continue
                return pose
            except (ValueError,TransformException) as exc:
                if stable_since is not None:
                    longest_good_s=max(longest_good_s,time.monotonic()-stable_since)
                stable_since=None
                issue=str(exc)
                failures[issue]+=1
                last_failure=issue
        self.readiness_diagnostics=dict(longest_continuous_quality_s=longest_good_s,
            last_failed_check=last_failure,failure_counts=dict(failures),last_wait_reason=issue)
        if getattr(self,'last_stationary_candidate',None) is not None:
            self.readiness_diagnostics['last_stationary_candidate']=self.last_stationary_candidate
            goal=getattr(self,'requested_A_pose',None)
            if goal is not None:
                xy,yaw=pose_error(self.last_stationary_candidate['pose_xyyaw'],goal)
                self.readiness_diagnostics.update(position_error_m=xy,heading_error_deg=math.degrees(yaw))
        details=(f'; longest valid window {longest_good_s:.2f} s; last failed check: {last_failure}'
                 if last_failure is not None else f'; longest valid window {longest_good_s:.2f} s')
        raise RuntimeError('localization readiness timeout: '+issue+details)

    def save(self,map_file,output,timeout):
        if output.exists():
            raise ValueError('New Point A file already exists; choose a new output path or explicitly remove it')
        self.selected_map_file=str(Path(map_file).resolve())
        pose=self.wait_pose(timeout)
        evidence=self.localization_evidence()
        value=dict(schema=SCHEMA,purpose='pallet_recognition_A',frame_id='map',
            goal_id=str(uuid.uuid4()),saved_at_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            pose_xyyaw=list(pose),map=map_identity(map_file),
            source=('stationary_AMCL_snapshot_validated_by_live_scan_odom'
                if evidence.get('stationary_snapshot_validation') is not None
                else 'fresh_AMCL_map_pose_at_stationary_robot'),
            pose=dict(position=dict(x=pose[0],y=pose[1],z=0.),
                orientation=dict(x=0.,y=0.,z=math.sin(pose[2]/2),w=math.cos(pose[2]/2))),
            localization_evidence=evidence,
            arrival_limits=dict(position_m=ARRIVAL_XY_TOLERANCE_M,
                                heading_deg=math.degrees(ARRIVAL_YAW_TOLERANCE_RAD)),
            physical_alignment_verified=False)
        try:
            create_goal_json(output,value)
        except FileExistsError as exc:
            raise ValueError('Point A was saved by another process; existing A was preserved') from exc
        return dict(status='NEW_A_SAVED',goal=str(output),pose_xyyaw=list(pose),goal_id=value['goal_id'])

    def localization_evidence(self):
        p=self.amcl.pose.pose
        raw=self.odom.pose.pose
        evidence=dict(captured_ros_s=self.now(),amcl_stamp_ros_s=self.stamp(self.amcl),
            amcl_pose_xyyaw=[p.position.x,p.position.y,yaw_of(p.orientation)],
            amcl_covariance_6x6=list(self.amcl.pose.covariance),
            odom_stamp_ros_s=self.stamp(self.odom),
            odom_pose_xyyaw=[raw.position.x,raw.position.y,yaw_of(raw.orientation)],
            map_odom_xyyaw=list(self.last_map_odom),
            scan_stamp_ros_s=self.scan_stamp,scan_map_match=self.map_match,
            particle_metrics=self.particle_gate.metrics,a_quality=self.a_quality,
            stationary_duration_s=(time.monotonic()-self.stationary_since
                                   if self.stationary_since is not None else 0.),
            recent_amcl_poses=[dict(stamp_ros_s=s,pose_xyyaw=list(p))
                               for s,p in self.observations if self.now()-s<=2.],
            uncertainty_is_physical_accuracy_proof=False)
        if getattr(self,'continuous_localization',False):
            evidence['localization_mode']=self.current_localization_mode
            evidence['live_amcl_observation_fresh']=self.current_localization_mode=='fresh_live_amcl'
            if self.current_localization_mode=='bounded_motion_prediction':
                evidence['motion_prediction']=copy.deepcopy(self.stationary_status.get('motion_prediction'))
        if getattr(self,'stationary_mode',False):
            ready,reason=self.stationary_evidence.ready(self.now())
            evidence['stationary_snapshot_validation']=dict(
                run_id=self.stationary_snapshot['run_id'],
                original_amcl_stamp_ros_s=self.stationary_snapshot['amcl_stamp_s'],
                original_amcl_age_s=self.now()-self.stationary_snapshot['amcl_stamp_s'],
                original_cloud_stamp_ros_s=self.stationary_snapshot['cloud_stamp_s'],
                particle_pairing=copy.deepcopy(self.stationary_snapshot.get('particle_pairing')),
                original_covariance_preserved=True,repeated_nomotion_updates=False,
                live_scan_odom_validated=ready,reason=reason,
                distinct_scan_count=len(self.stationary_evidence.scans),
                strict_a_scan_duration_s=self.a_scan_quality_duration(),
                physical_accuracy_verified=False)
            evidence['recent_scan_validated_poses']=[dict(scan_stamp_ros_s=s,pose_xyyaw=list(p))
                for s,p in self.stationary_validated_poses if -.05<=self.now()-s<=2.]
        return evidence

    def await_future(self,future,deadline,check_localization=False):
        pending_since=None
        pending_record=None
        while rclpy.ok() and not future.done():
            if time.monotonic()>deadline:
                raise RuntimeError('Nav2 action timed out')
            self.spin_once()
            if check_localization:
                if getattr(self,'navigation_action_active',False):
                    self.monitor_navigation_session()
                    continue
                try:
                    self.current_pose(False)
                except MotionPredictionPending:
                    # This dedicated ordering wait never returns an accepted
                    # pose and cannot extend beyond the original 150 ms onset.
                    continue
                except LocalizationDataPending as exc:
                    wall=time.monotonic()
                    if pending_since is None:
                        pending_since=wall
                        pending_record=dict(reason=str(exc),recovered=False,duration_s=0.)
                        if hasattr(self,'now'):
                            ros_now=self.now()
                            pending_record['started_ros_s']=ros_now
                            odom=getattr(self,'odom',None)
                            if odom is not None:
                                pending_record['odom_age_s']=ros_now-self.stamp(odom)
                            scan_stamp=getattr(self,'scan_stamp',None)
                            if scan_stamp is not None:
                                pending_record['scan_age_s']=ros_now-scan_stamp
                        if not hasattr(self,'navigation_data_waits'):
                            self.navigation_data_waits=[]
                        self.navigation_data_waits.append(pending_record)
                        if hasattr(self,'get_logger'):
                            self.get_logger().warn('Keeping Nav2 goal while waiting for localization data: '+str(exc))
                    pending_record['duration_s']=wall-pending_since
                    pending_record['last_reason']=str(exc)
                    if wall-pending_since>=NAV_DATA_RECOVERY_S:
                        raise RuntimeError(f'Nav2 localization data did not recover within '
                            f'{NAV_DATA_RECOVERY_S:g} seconds: {exc}') from exc
                    # Compatibility for callers requesting a checked pose:
                    # waiting does not accept stale evidence. Native A
                    # actions use the session-monitor branch above instead.
                    continue
                if pending_since is not None:
                    pending_record.update(recovered=True,duration_s=time.monotonic()-pending_since)
                    if hasattr(self,'get_logger'):
                        self.get_logger().info('Localization data recovered; continuing the same Nav2 goal')
                    pending_since=None
                    pending_record=None
        if not future.done():
            raise RuntimeError('ROS stopped during Nav2 action')
        return future.result()

    def open_navigation_relay(self, callback):
        self.base_output=self.create_publisher(Twist,'/cmd_vel_safe',
            QoSProfile(depth=1,lifespan=Duration(seconds=.2)))
        self.create_subscription(Twist,delivery_velocity('/cmd_vel_safe'),callback,
                                 qos_profile_sensor_data)

    def wait_check(self,check,timeout):
        deadline=time.monotonic()+timeout
        reason='đang chờ dữ liệu'
        while rclpy.ok() and time.monotonic()<deadline:
            self.spin_once()
            try:
                return check()
            except (ValueError,TransformException) as exc:
                reason=str(exc)
        raise RuntimeError(reason)

    def check_navigation_relay(self):
        check_navigation_command_owners(self,relay=self.get_fully_qualified_name(),navigation=True)
        receivers=self.get_subscriptions_info_by_topic('/cmd_vel_safe')
        if sum((p.node_namespace,p.node_name)==('/','cmd_vel_stamper') for p in receivers)!=1:
            raise ValueError('Đầu nhận lệnh bánh chưa sẵn sàng.')

    def on_a_safe_velocity(self,message):
        if not self.motion_enabled:
            return
        status=self.stationary_status or {}
        stamp=status.get('published_ros_s')
        command=Twist()
        if (status.get('run_id')==self.navigation_run_id and status.get('map_file')==self.selected_map_file
                and status.get('navigation_monitor') is True and status.get('trial_handoff') is None
                and not status.get('localization_fault') and isinstance(stamp,(int,float))
                and -.05<=self.now()-stamp<=.75 and self.map_signature==self.navigation_map_signature
                and all(math.isfinite(v) for v in (message.linear.x,message.angular.z))):
            command.linear.x,command.angular.z=message.linear.x,message.angular.z
        self.base_output.publish(command)
        self.forwarded_velocity_commands+=1

    def monitor_navigation_session(self):
        """Keep identity faults fatal; the bridge owns jump recovery.

        A delayed Python quality report is not a velocity interlock. The
        native controller, costmaps and collision monitor use current TF and
        LiDAR; complete independent quality checks bracket the A action.
        """
        if self.map_signature!=self.navigation_map_signature:
            raise ValueError('occupancy map changed during navigation')
        status=self.stationary_status
        if not isinstance(status,dict):
            return
        if status.get('run_id')!=self.navigation_run_id:
            raise ValueError('localization session changed during navigation')
        if status.get('map_file')!=self.selected_map_file:
            raise ValueError('selected localization map changed during navigation')
        if status.get('navigation_monitor') is not True:
            raise ValueError('localization role changed during navigation')
        if status.get('localization_fault'):
            raise ValueError('localization fault during navigation: '+str(status['localization_fault']))
        if getattr(self,'shared_navigation',False) and self.motion_enabled:
            if status.get('trial_handoff') is not None:
                raise ValueError('quyền điều khiển chặng A đã được bàn giao')
            wall=time.monotonic()
            if wall>=self.next_a_graph_check:
                self.check_navigation_relay()
                self.next_a_graph_check=wall+.5
        recovery=status.get('navigation_recovery') or {}
        holding=recovery.get('hold_commands') is True
        if holding != getattr(self,'navigation_recovery_hold',False):
            self.navigation_recovery_hold=holding
            if hasattr(self,'get_logger'):
                self.get_logger().info(
                    'Đang kiểm tra lại định vị bằng scan mới; tạm giữ lệnh, giữ nguyên goal A.'
                    if holding else 'Định vị đã nhất quán; tiếp tục cùng goal A.')
        if recovery.get('started_ros_s') is not None:
            self.last_navigation_recovery=copy.deepcopy(recovery)
        snapshot=self.stationary_snapshot
        if snapshot is None:
            return
        stamp=snapshot['amcl_stamp_s']
        if self.navigation_observation_stamp is None or stamp>self.navigation_observation_stamp:
            map_odom=tuple(snapshot['map_odom_xyyaw'])
            # The navigation-only bridge checks anomalous corrections with
            # new scans while holding the command chain. A validated release
            # must not trigger this second permanent cancellation afterwards.
            # The saved-A quality gate still checks the final stationary pose.
            self.last_map_odom=map_odom
            self.navigation_observation_stamp=stamp

    def cancel(self):
        if self.goal_handle is not None and self.result_future is not None and not self.result_future.done():
            future=self.goal_handle.cancel_goal_async()
            until=time.monotonic()+5.
            while rclpy.ok() and not future.done() and time.monotonic()<until:
                self.spin_once()

    def navigate(self,map_file,goal_path,timeout,check_only=False,recognize_here=False):
        if check_only and recognize_here:
            raise ValueError('choose check-only or recognize-here')
        selected_map=map_identity(map_file)
        value=json.loads(goal_path.read_text())
        goal=validate_goal(value,selected_map)
        self.requested_A_pose=goal
        self.wait_pose(30.)
        if recognize_here:
            # This explicit operator-selected mode uses the current measured
            # recognition place. It never claims that the saved A was reached,
            # never replaces its pose, and needs no Nav2 action server.
            pose=self.wait_pose(10.)
            xy,yaw=pose_error(pose,goal)
            try:
                validate_arrival(pose,goal)
                within_limits=True
            except RuntimeError:
                within_limits=False
            return dict(status='RECOGNIZING_HERE',recognize_here=True,
                reason='user_confirmed_current_recognition_place',
                navigation_goal_attempted=False,direct_velocity_commands_from_this_process=0,
                robot_commands_from_this_process=0,goal_id=value['goal_id'],goal=str(goal_path),
                goal_pose_xyyaw=list(goal),current_pose_xyyaw=list(pose),
                position_error_m=xy,heading_error_deg=math.degrees(yaw),
                within_saved_A_limits=within_limits,
                arrival_limits=dict(position_m=ARRIVAL_XY_TOLERANCE_M,
                                    heading_deg=math.degrees(ARRIVAL_YAW_TOLERANCE_RAD)),
                map=selected_map,stationary=True,scan_map_match=self.map_match,
                particle_metrics=self.particle_gate.metrics,
                localization_evidence=self.localization_evidence(),
                physical_alignment_verified=False)
        if not self.client.wait_for_server(timeout_sec=30.):
            raise RuntimeError('NavigateToPose action server is unavailable')
        # Server discovery may have taken time; refresh the stationary pose
        # instead of trusting the readiness observation from before discovery.
        start_pose=self.wait_pose(10.)
        if check_only:
            return dict(status='READY_FOR_NAVIGATION',check_only=True,
                arrival_limits=dict(position_m=ARRIVAL_XY_TOLERANCE_M,
                                    heading_deg=math.degrees(ARRIVAL_YAW_TOLERANCE_RAD)),
                robot_commands_from_this_process=0,goal_id=value['goal_id'],goal=str(goal_path),
                navigation_goal_attempted=False,direct_velocity_commands_from_this_process=0,
                goal_pose_xyyaw=list(goal),current_pose_xyyaw=list(start_pose),
                map=selected_map,localization_evidence=self.localization_evidence(),
                physical_alignment_verified=False)
        # A stationary robot already within the saved recognition region can
        # proceed using its measured pose. Sending a redundant goal would not
        # improve that pose and must not manufacture another AMCL update.
        try:
            start_xy,start_yaw=validate_arrival(start_pose,goal)
        except RuntimeError:
            pass
        else:
            return dict(status='AT_NEW_A',already_at_A=True,
                navigation_goal_attempted=False,direct_velocity_commands_from_this_process=0,
                robot_commands_from_this_process=0,goal_id=value['goal_id'],goal=str(goal_path),
                goal_pose_xyyaw=list(goal),arrival_pose_xyyaw=list(start_pose),
                position_error_m=start_xy,heading_error_deg=math.degrees(start_yaw),
                arrival_limits=dict(position_m=ARRIVAL_XY_TOLERANCE_M,
                                    heading_deg=math.degrees(ARRIVAL_YAW_TOLERANCE_RAD)),
                map=selected_map,stationary=True,scan_map_match=self.map_match,
                particle_metrics=self.particle_gate.metrics,
                localization_evidence=self.localization_evidence(),
                physical_alignment_verified=False)
        if self.shared_navigation:
            check_navigation_command_owners(self,navigation=True)
            self.open_navigation_relay(self.on_a_safe_velocity)
            self.wait_check(self.check_navigation_relay,5.)
            start_pose=self.wait_pose(10.)
        if getattr(self,'continuous_localization',False):
            self.navigation_run_id=self.stationary_status['run_id']
            self.navigation_map_signature=self.map_signature
            self.navigation_observation_stamp=self.stationary_snapshot['amcl_stamp_s']
        message=NavigateToPose.Goal()
        message.pose.header.frame_id='map';message.pose.header.stamp=self.get_clock().now().to_msg()
        message.pose.pose.position.x,message.pose.pose.position.y=goal[:2]
        message.pose.pose.orientation.z=math.sin(goal[2]/2)
        message.pose.pose.orientation.w=math.cos(goal[2]/2)
        start_xy,start_yaw=pose_error(start_pose,goal)
        heading_only=start_xy<ARRIVAL_XY_TOLERANCE_M-1e-12
        if heading_only:
            # The user accepts this position inside A's region. Correct its
            # heading in place instead of driving a short loop for a few cm.
            message.pose.pose.position.x,message.pose.pose.position.y=start_pose[:2]
        initial_turn=(wrap(math.atan2(goal[1]-start_pose[1],goal[0]-start_pose[0])-start_pose[2])
                      if start_xy>1e-9 else 0.)
        self.navigation_start_evidence=dict(current_pose_xyyaw=list(start_pose),
            goal_pose_xyyaw=list(goal),position_error_m=start_xy,
            heading_error_deg=math.degrees(start_yaw),
            goal_bearing_turn_deg=math.degrees(initial_turn),
            heading_only_within_A=heading_only,
            command_goal_pose_xyyaw=[message.pose.pose.position.x,message.pose.pose.position.y,goal[2]],
            goal_id=value['goal_id'],localization_evidence=self.localization_evidence())
        print(json.dumps(dict(status='NAVIGATION_START',
            **{key:self.navigation_start_evidence[key] for key in (
                'current_pose_xyyaw','goal_pose_xyyaw','position_error_m',
                'heading_error_deg','goal_bearing_turn_deg','heading_only_within_A',
                'command_goal_pose_xyyaw')}),
            ensure_ascii=False,indent=2),flush=True)
        deadline=time.monotonic()+timeout
        try:
            self.navigation_finalization=[]
            for attempt in range(1,MAX_A_FINALIZATION_ATTEMPTS+1):
                if time.monotonic()>=deadline:
                    raise RuntimeError('Nav2 action timed out')
                self.navigation_action_active=getattr(self,'continuous_localization',False)
                self.navigation_goal_attempted=True
                message.pose.header.stamp=self.get_clock().now().to_msg()
                future=self.client.send_goal_async(message)
                self.goal_handle=self.await_future(future,deadline)
                if not self.goal_handle.accepted:
                    raise RuntimeError('Nav2 rejected the saved Point A')
                self.result_future=self.goal_handle.get_result_async()
                self.motion_enabled=self.shared_navigation
                try:
                    result=self.await_future(self.result_future,deadline,True)
                finally:
                    self.motion_enabled=False
                    if self.base_output is not None and rclpy.ok():
                        self.base_output.publish(Twist())
                        self.forwarded_velocity_commands+=1
                self.navigation_action_active=False
                self.navigation_result=dict(action_status=result.status,attempt=attempt,
                    reported_ros_s=self.get_clock().now().nanoseconds*1e-9
                    if hasattr(self.get_clock().now(),'nanoseconds') else None)
                if result.status!=GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError(f'Nav2 did not reach Point A: action status {result.status}')
                print('Nav2 đã báo tới đích; xác nhận vị trí A thực tế để bắt đầu nhận diện...',flush=True)
                pose=self.wait_pose(min(20.,max(.01,deadline-time.monotonic())))
                xy,yaw=pose_error(pose,goal)
                self.navigation_finalization.append(dict(attempt=attempt,pose_xyyaw=list(pose),
                    position_error_m=xy,heading_error_deg=math.degrees(yaw),
                    command_goal_pose_xyyaw=[message.pose.pose.position.x,message.pose.pose.position.y,goal[2]],
                    localization_evidence=self.localization_evidence()))
                try:
                    validate_arrival(pose,goal)
                    break
                except RuntimeError:
                    # A is a coarse recognition region. The operator disabled
                    # repeated finalization; the single attempt limit prevents
                    # heading/position oscillations around the saved point.
                    if attempt>=MAX_A_FINALIZATION_ATTEMPTS or xy>.10 or yaw>math.radians(10.):
                        raise
                    target_xy=pose[:2] if xy<ARRIVAL_XY_TOLERANCE_M-1e-12 else goal[:2]
                    message.pose.pose.position.x,message.pose.pose.position.y=target_xy
                    print(json.dumps(dict(status='FINALIZING_A',attempt=attempt,
                        position_error_m=xy,heading_error_deg=math.degrees(yaw),
                        command_goal_pose_xyyaw=[*target_xy,goal[2]],
                        goal_pose_xyyaw=list(goal)),indent=2),flush=True)
            if getattr(self,'continuous_localization',False):
                self.monitor_navigation_session()
            return dict(status='AT_NEW_A',already_at_A=False,
                        navigation_goal_attempted=True,
                        direct_velocity_commands_from_this_process=self.forwarded_velocity_commands,
                        goal_id=value['goal_id'],goal=str(goal_path),
                        goal_pose_xyyaw=list(goal),arrival_pose_xyyaw=list(pose),
                        position_error_m=xy,heading_error_deg=math.degrees(yaw),
                        arrival_limits=dict(position_m=ARRIVAL_XY_TOLERANCE_M,
                                            heading_deg=math.degrees(ARRIVAL_YAW_TOLERANCE_RAD)),
                        map=selected_map,stationary=True,scan_map_match=self.map_match,
                        particle_metrics=self.particle_gate.metrics,
                        localization_evidence=self.localization_evidence(),
                        arrival_attempts=self.navigation_finalization,
                        localization_recovery=getattr(self,'last_navigation_recovery',None),
                        physical_alignment_verified=False)
        finally:
            self.navigation_action_active=False
            self.cancel()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    save=sub.add_parser('save')
    save.add_argument('--map',type=Path,required=True)
    save.add_argument('--output',type=Path,default=DEFAULT_GOAL)
    save.add_argument('--timeout',type=float,default=120.)
    inspect=sub.add_parser('inspect')
    inspect.add_argument('--map',type=Path,required=True)
    inspect.add_argument('--goal',type=Path,default=DEFAULT_GOAL)
    navigate=sub.add_parser('navigate')
    navigate.add_argument('--map',type=Path,required=True)
    navigate.add_argument('--goal',type=Path,default=DEFAULT_GOAL)
    navigate.add_argument('--output',type=Path,required=True)
    navigate.add_argument('--timeout',type=float,default=900.)
    navigate.add_argument('--continuous-localization',action='store_true',
                         help='Use continuously validated live/parked AMCL snapshots without repeated no-motion updates')
    navigate.add_argument('--shared-navigation',action='store_true',
                          help='Use the parked-between-actions Nav2 instance for the pallet trip')
    purpose=navigate.add_mutually_exclusive_group()
    purpose.add_argument('--check-only',action='store_true',
                         help='Check localization and Nav2 readiness; never send a navigation goal')
    purpose.add_argument('--recognize-here',action='store_true',
                         help='Recognize from the current verified stationary place; never discover or send a navigation goal')
    args=parser.parse_args()
    if args.command!='inspect' and (not math.isfinite(args.timeout) or args.timeout<=0):
        parser.error('timeout must be finite and positive')
    # Missing or old-schema A is rejected before creating any action client.
    if args.command in ('navigate','inspect'):
        if not args.goal.is_file():
            parser.error(f'New Point A has not been saved: {args.goal}')
        validate_goal(json.loads(args.goal.read_text()),map_identity(args.map))
    if args.command=='inspect':
        value=json.loads(args.goal.read_text())
        print(json.dumps(dict(status='NEW_A_VALID',goal=str(args.goal),
            pose_xyyaw=value['pose_xyyaw'],goal_id=value['goal_id']),indent=2))
        return 0
    continuous=args.command=='navigate' and args.continuous_localization
    shared=args.command=='navigate' and args.shared_navigation
    if shared and (not continuous or args.recognize_here):
        parser.error('--shared-navigation requires continuous navigation to A')
    rclpy.init();node=RecognitionGoal(request_nomotion=args.command=='navigate' and not continuous,
                                   selected_map_file=args.map,continuous_localization=continuous,
                                   navigation_action=PRIVATE_ACTION if shared else '/navigate_to_pose',
                                   shared_navigation=shared);result=None
    try:
        if args.command=='save':
            result=node.save(args.map,args.output,args.timeout)
        else:
            result=node.navigate(args.map,args.goal,args.timeout,check_only=args.check_only,
                                 recognize_here=args.recognize_here)
        if node.navigation_data_waits:
            result['navigation_data_waits']=node.navigation_data_waits
        if node.navigation_result is not None:
            result['navigation_result']=node.navigation_result
        print(json.dumps(result,indent=2))
    except (KeyboardInterrupt,RuntimeError,ValueError,TransformException) as exc:
        result=dict(status='BLOCKED',reason=str(exc),
            navigation_goal_attempted=node.navigation_goal_attempted,
            direct_velocity_commands_from_this_process=node.forwarded_velocity_commands)
        if isinstance(exc,MotionPredictionLimitError):
            result['motion_prediction_failure']=exc.evidence
        if getattr(node,'navigation_start_evidence',None) is not None:
            result['navigation_start_evidence']=node.navigation_start_evidence
        if not node.navigation_goal_attempted:
            result['robot_commands_from_this_process']=0
        if node.readiness_diagnostics:
            result['readiness_diagnostics']=node.readiness_diagnostics
        if node.navigation_result is not None:
            result['navigation_result']=node.navigation_result
        if node.navigation_finalization:
            result['arrival_attempts']=node.navigation_finalization
        if node.navigation_data_waits:
            result['navigation_data_waits']=node.navigation_data_waits
        print(json.dumps(result,indent=2))
    finally:
        node.cancel()
        if args.command=='navigate' and result is not None:
            atomic_json(args.output,result)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result and result['status'] in ('NEW_A_SAVED','AT_NEW_A','READY_FOR_NAVIGATION',
                                             'RECOGNIZING_HERE') else 2


if __name__=='__main__':
    raise SystemExit(main())
