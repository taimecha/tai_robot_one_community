#!/usr/bin/env python3
"""Enable bounded A-to-C localization on the existing navigation monitor.

Only an explicitly rejected, non-mutating role request may be retried (three
attempts at most); uncertain responses are never repeated. The same snapshot
and fresh control odometry are observed after success. This publishes no pose,
transform, navigation goal or velocity.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from r3_scan_map_check import validate_recognition_scan_match
from r3_amcl_quality import validate_runtime_particles
from r3_delivery_topics import VELOCITY_TOPICS


MAX_SERVICE_ATTEMPTS=3
MIN_RETRY_INTERVAL_S=.25
SOURCE_FIELDS=('run_id','map_file','amcl_stamp_s','cloud_stamp_s','map_pose_xyyaw',
               'map_odom_xyyaw','odom_anchor_xyyaw','covariance_6x6','particle_metrics',
               'particle_pairing','occupancy_map_signature')


def finite_number(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def fresh(value,now,limit):
    return finite_number(value) and finite_number(now) and -.05<=now-value<=limit


def _pose(value,name):
    if not isinstance(value,(list,tuple)) or len(value)!=3 or not all(finite_number(v) for v in value):
        raise ValueError('invalid '+name)
    return tuple(value)


def snapshot_digest(snapshot):
    source={key:snapshot.get(key) for key in SOURCE_FIELDS}
    return hashlib.sha256(json.dumps(source,sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_source(status,map_file,now,*,validate_scan=validate_recognition_scan_match):
    """Bind the request to a fresh parked monitor in the selected map/run."""
    if not isinstance(status,dict) or not fresh(status.get('published_ros_s'),now,.5):
        raise ValueError('localization status is missing/stale')
    if (status.get('ready') is not True or status.get('continuous_localization') is not True
            or status.get('frame_id')!='r3_trial' or not status.get('run_id')):
        raise ValueError('continuous localization is not ready for handoff: '+str(status.get('reason','unknown')))
    if status.get('map_file')!=str(Path(map_file).resolve()):
        raise ValueError('localization handoff belongs to a different map')
    if status.get('robot_stationary') is not True or status.get('stationary_cache',{}).get('validated') is not True:
        raise ValueError('handoff requires a currently validated stationary localization snapshot')
    snapshot=status.get('stationary_snapshot')
    if (not isinstance(snapshot,dict) or snapshot.get('run_id')!=status['run_id']
            or snapshot.get('map_file')!=status['map_file']
            or not finite_number(snapshot.get('amcl_stamp_s'))
            or not finite_number(snapshot.get('cloud_stamp_s'))):
        raise ValueError('handoff lacks its original map/run-bound AMCL snapshot')
    covariance=snapshot.get('covariance_6x6')
    if (not isinstance(covariance,list) or len(covariance)!=36
            or not all(finite_number(v) for v in covariance)
            or any(covariance[i]<0 for i in (0,7,35))):
        raise ValueError('handoff AMCL covariance is invalid')
    if (math.sqrt(covariance[0]+covariance[7])>=.05-1e-12
            or math.degrees(math.sqrt(covariance[35]))>2.+1e-9):
        raise ValueError('handoff AMCL uncertainty requires <5 cm / 2 deg')
    metrics=snapshot.get('particle_metrics')
    validate_runtime_particles(metrics)
    fit=status.get('scan_map_fit')
    if (not isinstance(fit,dict) or fit.get('ready') is not True
            or not fresh(fit.get('scan_stamp_s'),now,.5)):
        raise ValueError('handoff requires fresh scan/map agreement')
    validate_scan(fit)
    _pose(snapshot.get('map_odom_xyyaw'),'original map/odom pose')
    _pose(status.get('current_map_odom_xyyaw'),'current map/odom pose')
    snapshot_digest(snapshot)
    count=status['stationary_cache'].get('nomotion_request_count')
    if isinstance(count,bool) or not isinstance(count,int) or count<0:
        raise ValueError('handoff no-motion request count is invalid')
    return snapshot


def validate_request_identity(initial,status):
    """A rejected request cannot justify changing or replacing its source."""
    if (not isinstance(status,dict) or status.get('run_id')!=initial['run_id']
            or status.get('map_file')!=initial['map_file']):
        raise RuntimeError('localization process or map changed after the handoff request')
    snapshot=status.get('stationary_snapshot')
    if not isinstance(snapshot,dict) or snapshot_digest(snapshot)!=snapshot_digest(initial['stationary_snapshot']):
        raise RuntimeError('original AMCL snapshot changed after the handoff request')
    if (status.get('stationary_cache',{}).get('nomotion_request_count')
            !=initial['stationary_cache']['nomotion_request_count']):
        raise RuntimeError('AMCL was refreshed after the handoff request')


def validate_rejected_request(initial,node,now):
    """Retry only an explicit rejection with an unchanged healthy monitor."""
    status=node.status
    validate_request_identity(initial,status)
    if (not fresh(status.get('published_ros_s'),now,.5)
            or status.get('navigation_monitor') is not True
            or status.get('role') not in (None,'navigation_monitor')
            or status.get('continuous_localization') is not True or status.get('ready') is not True
            or status.get('robot_stationary') is not True
            or status.get('stationary_cache',{}).get('validated') is not True
            or status.get('trial_handoff') is not None):
        raise RuntimeError('rejected handoff cannot be retried after a role or localization fault')
    owners=node.velocity_publishers()
    if owners:
        raise RuntimeError('rejected handoff cannot be retried while velocity publishers are present: '+json.dumps(owners,sort_keys=True))


def validate_handoff(initial,status,odom,now,map_file):
    """Verify the role changed while the original localization evidence stayed."""
    source=validate_source(status,map_file,now)
    if status['run_id']!=initial['run_id'] or status['map_file']!=initial['map_file']:
        raise ValueError('localization process or map changed during handoff')
    if status.get('navigation_monitor') is not False or status.get('role')!='trial':
        raise ValueError('localization has not entered the trial role')
    handoff=status.get('trial_handoff')
    if not isinstance(handoff,dict) or handoff.get('enabled') is not True:
        raise ValueError('trial handoff has not been confirmed')
    enabled=handoff.get('at_ros_s')
    if not finite_number(enabled) or enabled>now+.05:
        raise ValueError('trial handoff timestamp is invalid')
    original=initial['stationary_snapshot']
    if snapshot_digest(source)!=snapshot_digest(original):
        raise ValueError('original AMCL snapshot changed during role handoff')
    if handoff.get('source_amcl_stamp_s')!=original['amcl_stamp_s']:
        raise ValueError('trial handoff changed the AMCL observation timestamp')
    before=initial['stationary_cache']['nomotion_request_count']
    after=status['stationary_cache']['nomotion_request_count']
    if before!=after or handoff.get('nomotion_request_count')!=before:
        raise ValueError('AMCL was refreshed during role handoff')
    baseline=_pose(handoff.get('baseline_map_odom'),'trial handoff baseline')
    final_baseline=_pose(status.get('baseline_map_odom'),'bounded trial baseline')
    if baseline!=final_baseline:
        raise ValueError('trial baseline differs from confirmed handoff')
    if initial.get('navigation_monitor') is True:
        requested=_pose(initial.get('current_map_odom_xyyaw'),'request map/odom pose')
        if any(abs(a-b)>1e-8 for a,b in zip(baseline,requested)):
            raise ValueError('trial baseline does not anchor the validated current pose')
    elif initial.get('navigation_monitor') is False:
        previous=initial.get('trial_handoff')
        if previous!=handoff or initial.get('baseline_map_odom')!=status.get('baseline_map_odom'):
            raise ValueError('idempotent handoff changed the already frozen trial baseline')
    else:
        raise ValueError('request had an invalid localization role')
    if (odom is None or odom.header.frame_id!='r3_trial'
            or odom.child_frame_id not in ('base_footprint','base_link')
            or not fresh(stamp(odom),now,.2) or stamp(odom)<enabled-.05):
        raise ValueError('fresh bounded trial odometry is not available')
    pose=odom.pose.pose;quaternion=pose.orientation
    values=(pose.position.x,pose.position.y,quaternion.x,quaternion.y,quaternion.z,quaternion.w)
    if (not all(finite_number(v) for v in values)
            or abs(sum(v*v for v in values[2:])-1.)>.02):
        raise ValueError('bounded trial odometry pose is invalid')
    return dict(status='TRIAL_LOCALIZATION_READY',run_id=status['run_id'],map_file=status['map_file'],
        frame_id='r3_trial',published_ros_s=now,baseline_map_odom=list(baseline),
        source_amcl_stamp_s=original['amcl_stamp_s'],source_cloud_stamp_s=original['cloud_stamp_s'],
        original_snapshot_sha256=snapshot_digest(original),
        original_covariance_preserved=True,original_particle_evidence_preserved=True,
        nomotion_request_count_before=before,nomotion_request_count_after=after,
        localization_status=copy.deepcopy(status),trial_handoff=copy.deepcopy(handoff),
        localized_odom_stamp_s=stamp(odom),actuator_commands=0,navigation_goals_sent=0,
        transform_publications=0,
        successful_role_changes=1 if initial.get('navigation_monitor') is True else 0,
        physical_alignment_verified=False)


class TrialEnableClient(Node):
    def __init__(self):
        super().__init__('r3_enable_trial')
        self.status=None;self.odom=None;self.odom_stamps=[]
        self.control_status=None
        self.service=self.create_client(Trigger,'/pallet/r3/localization/enable_trial')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/pallet/r3/localization/status',self.on_status,qos)
        self.create_subscription(Odometry,'/pallet/r3/localized_odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(String,'/pallet/r3/control_odom/status',self.on_control_status,qos)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_status(self,msg):
        try:self.status=json.loads(msg.data)
        except (ValueError,TypeError):self.status=None

    def on_odom(self,msg):
        self.odom=msg
        value=stamp(msg)
        if (msg.header.frame_id=='r3_trial' and msg.child_frame_id in ('base_footprint','base_link')
                and (not self.odom_stamps or value>self.odom_stamps[-1])):
            self.odom_stamps.append(value)
            self.odom_stamps=self.odom_stamps[-20:]

    def on_control_status(self,msg):
        try:
            value=json.loads(msg.data)
            if isinstance(value,dict):self.control_status=value
        except (ValueError,TypeError):
            pass

    def velocity_publishers(self):
        owners={}
        for topic in VELOCITY_TOPICS:
            publishers=self.get_publishers_info_by_topic(topic)
            if publishers:owners[topic]=[info.node_name for info in publishers]
        return owners

    def spin_once(self):rclpy.spin_once(self,timeout_sec=.05)


def enable_trial(node,map_file,timeout,*,clock=time.monotonic,ok=rclpy.ok):
    deadline=clock()+timeout;initial=None;future=None;acknowledged=False
    attempts=0;last_request_wall=None
    node.role_change_service_calls=0;node.role_change_acknowledged=False
    issue='waiting for the current localization monitor'
    while ok() and clock()<deadline:
        node.spin_once()
        now=node.now()
        if future is None:
            if initial is not None:
                validate_request_identity(initial,node.status)
            try:
                validate_source(node.status,map_file,now)
                owners=node.velocity_publishers()
                if owners:
                    issue='waiting for Nav2 velocity publishers to disappear: '+json.dumps(owners,sort_keys=True)
                    continue
                if not node.service.service_is_ready():
                    issue='waiting for the localization role-change service';continue
                if last_request_wall is not None and clock()-last_request_wall<MIN_RETRY_INTERVAL_S:
                    issue='waiting for the bounded role-change retry interval';continue
                if initial is None:initial=copy.deepcopy(node.status)
                attempts+=1;node.role_change_service_calls=attempts
                last_request_wall=clock()
                future=node.service.call_async(Trigger.Request())
                issue='waiting for the localization role-change response'
            except ValueError as exc:issue=str(exc)
            continue
        if not acknowledged:
            if not future.done():continue
            response=future.result()
            if response is None or not isinstance(getattr(response,'success',None),bool):
                raise RuntimeError('localization role change returned an uncertain response; request was not repeated')
            if response.success is False:
                validate_rejected_request(initial,node,now)
                if attempts>=MAX_SERVICE_ATTEMPTS:
                    raise RuntimeError(f'localization role change rejected after {attempts} attempts: '+str(response.message))
                issue='explicit role-change rejection; waiting to retry unchanged monitor: '+str(response.message)
                future=None
                continue
            acknowledged=True
            node.role_change_acknowledged=True
            issue='waiting for fresh bounded trial localization status and odometry'
        try:
            health=getattr(node,'control_status',None)
            if (isinstance(node.status,dict) and node.status.get('external_control_odom') is True
                    and isinstance(health,dict)
                    and fresh(health.get('published_ros_s'),now,.5)
                    and health.get('integrity_fault')):
                raise RuntimeError('control odometry rejected handoff: '+str(health['integrity_fault']))
            result=validate_handoff(initial,node.status,node.odom,now,map_file)
            # Confirm a stream from the enabled role, not one queued sample.
            enabled=result['trial_handoff']['at_ros_s']
            samples=[s for s in node.odom_stamps if enabled-.05<=s<=now+.05 and now-s<=.2]
            if len(samples)<3 or samples[-1]-samples[0]<.08:
                issue='waiting for three distinct fresh bounded odometry samples';continue
            result['confirmed_odom_samples']=len(samples)
            result['confirmed_odom_span_s']=samples[-1]-samples[0]
            result['role_change_service_calls']=attempts
            result['role_change_acknowledged']=True
            result['control_odometry_status']=copy.deepcopy(health)
            return result
        except ValueError as exc:
            issue=str(exc)
            health=getattr(node,'control_status',None)
            if 'fresh bounded trial odometry' in issue and isinstance(health,dict):
                issue+='; control_odom: '+str(health.get('reason','unknown'))
    raise RuntimeError('localization role-change timeout/stopped: '+issue)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=30.)
    args=parser.parse_args()
    if not args.map.is_absolute() or not args.map.is_file():parser.error('--map must be an existing absolute map YAML')
    if not args.output.is_absolute():parser.error('--output must be an absolute JSON path')
    if not finite_number(args.timeout) or args.timeout<=0:parser.error('--timeout must be finite and positive')
    from r3_recognition_goal import atomic_json,map_identity
    selected=map_identity(args.map)
    rclpy.init();node=TrialEnableClient();result=None
    try:
        result=enable_trial(node,args.map,args.timeout)
        if selected['fingerprint_sha256']!=map_identity(args.map)['fingerprint_sha256']:
            raise RuntimeError('selected map changed during localization handoff')
        result['map']=selected
    except (KeyboardInterrupt,ExternalShutdownException,RuntimeError,ValueError) as exc:
        result=dict(status='BLOCKED',reason=str(exc) or 'localization role change interrupted',actuator_commands=0,
                    navigation_goals_sent=0,transform_publications=0,
                    role_change_service_calls=getattr(node,'role_change_service_calls',0),
                    role_change_acknowledged=getattr(node,'role_change_acknowledged',False),
                    control_odometry_status=copy.deepcopy(node.control_status),
                    localization_role=(node.status or {}).get('role'))
    finally:
        if result is not None:
            atomic_json(args.output,result)
            summary={key:result[key] for key in ('status','reason','run_id','baseline_map_odom',
                'source_amcl_stamp_s','original_covariance_preserved','nomotion_request_count_before',
                'nomotion_request_count_after','confirmed_odom_samples','actuator_commands') if key in result}
            summary['report']=str(args.output)
            print(json.dumps(summary,indent=2,allow_nan=False),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result and result['status']=='TRIAL_LOCALIZATION_READY' else 2


if __name__=='__main__':raise SystemExit(main())
