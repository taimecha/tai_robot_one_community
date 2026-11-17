#!/usr/bin/env python3
"""Apply validated AMCL corrections to each fresh EKF sample in its own process.

No scan scoring, particle processing, TF or motion commands run here. Original
EKF capture stamps and AMCL observation stamps are preserved. The supervisor's
0.5-second validation lease governs adoption of new AMCL corrections. After
the initial frame is validated, delayed/rejected AMCL updates leave that frame
unchanged and fresh EKF tracking continues, as requested by the operator.
The 0.2-second EKF limit and frame/odom integrity checks remain mandatory.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from r3_core import compose,wrap
from r3_localization_contract import checked_status
from r3_amcl_quality import validate_runtime_particles
from r3_scan_map_check import validate_recognition_scan_match
from r3_amcl_observation_pair import validate_pair_evidence


ODOM_MAX_AGE_S=.2
SUPERVISOR_MAX_AGE_S=.5


class IgnoredOdomSample(ValueError):
    """A queued/duplicate capture cannot replace a newer valid sample."""


def number(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def pose_value(value):
    if not isinstance(value,(list,tuple)) or len(value)!=3 or not all(number(v) for v in value):
        raise ValueError('invalid control odom transform')
    return tuple(value)


def rotate_covariance(covariance,yaw):
    """Same 6D rotation as the supervisor, omitting zero matrix products."""
    if len(covariance)!=36 or not all(number(v) for v in covariance):
        raise ValueError('invalid EKF covariance')
    c,s=math.cos(yaw),math.sin(yaw)
    rows=(((0,c),(1,-s)),((0,s),(1,c)),((2,1.),),
          ((3,c),(4,-s)),((3,s),(4,c)),((5,1.),))
    return [sum(a*b*covariance[k*6+l] for k,a in rows[i] for l,b in rows[j])
            for i in range(6) for j in range(6)]


class ControlOdomGuard:
    def __init__(self,map_file):
        self.map_file=str(Path(map_file).resolve())
        self.run_id=None
        self.expected=None
        self.fault=None
        self.last_stamp=None
        self.last_pose=None
        self.invalid_after=None
        self.parked_witness=None
        self.onset=None
        self.reference_state=None
        self.raw_stamp=None
        self.raw_pose=None
        self.raw_valid=False
        self.mode='waiting_for_initial_localization'
        self.update_issue='waiting for a validated trial reference'
        self.applied_correction=None
        self.target_correction=None
        self.last_output_capture=None
        self.insertion_lock=None
        self.insertion_pending=None
        self.insertion_capture_target=None

    def lock_for_insertion(self,request,now):
        """Hold the current frame, or slew to the plan's camera-capture frame.

        The optional capture target is immutable for this request/plan. It
        never changes the current output immediately, and an acknowledgement
        is withheld until fresh captures have completed the normal slew.
        """
        if (not isinstance(request,dict) or request.get('action')!='lock_for_insertion'
                or not self.fresh(request.get('published_ros_s'),now,1.)
                or not isinstance(request.get('plan_id'),str) or len(request['plan_id'])!=64):
            raise ValueError('invalid/stale insertion frame request')
        if self.expected is None or any(request.get(k)!=self.expected[k]
                                       for k in ('run_id','map_file','baseline_map_odom')):
            raise ValueError('insertion lock belongs to a different localization session/map')
        capture=(pose_value(request['capture_correction_xyyaw'])
                 if 'capture_correction_xyyaw' in request else None)
        if self.insertion_lock is not None:
            if request['plan_id']!=self.insertion_lock['plan_id']:
                raise ValueError('insertion frame is already held by another plan')
            if capture is not None and capture!=self.insertion_capture_target:
                raise ValueError('insertion capture frame changed after confirmation')
            return self.insertion_lock
        if (self.fault or not self.raw_valid or self.reference_state is None
                or self.applied_correction is None
                or not self.fresh(self.last_output_capture,now,ODOM_MAX_AGE_S)):
            raise ValueError('fresh validated EKF control frame required before insertion lock')
        if self.insertion_pending is not None:
            if (request['plan_id']!=self.insertion_pending['plan_id']
                    or capture!=self.insertion_capture_target):
                raise ValueError('pending insertion plan/capture frame changed')
            return dict(self.insertion_pending)
        if capture is not None:
            if (math.dist(capture[:2],self.applied_correction[:2])>.15
                    or abs(wrap(capture[2]-self.applied_correction[2]))>math.radians(5.)):
                raise ValueError('insertion capture frame exceeds 15 cm / 5 deg settling bounds')
            self.insertion_capture_target=capture
            self.target_correction=capture
            self.insertion_pending=dict(ready=False,plan_id=request['plan_id'],
                run_id=self.expected['run_id'],map_file=self.expected['map_file'],
                capture_correction_xyyaw=list(capture),requested_at_ros_s=now)
            self.mode='insertion_frame_settling'
            self.update_issue='settling the shared odom/LiDAR frame to camera capture at A'
            if self.applied_correction!=capture:
                return dict(self.insertion_pending)
        return self._confirm_insertion_lock(request,now)

    def _confirm_insertion_lock(self,request,now):
        self.insertion_lock=dict(ready=True,plan_id=request['plan_id'],
            run_id=self.expected['run_id'],map_file=self.expected['map_file'],
            correction_xyyaw=list(self.applied_correction),
            locked_at_ros_s=now,locked_capture_stamp_s=self.last_output_capture)
        if self.insertion_capture_target is not None:
            self.insertion_lock['capture_correction_xyyaw']=list(self.insertion_capture_target)
        self.insertion_pending=None
        self.target_correction=self.applied_correction
        self.mode='insertion_locked_ekf'
        return self.insertion_lock

    def _slew_correction(self,target,stamp):
        """Apply the same bounded interpolation to AMCL and capture targets."""
        dt=max(0.,min(.1,stamp-self.last_output_capture))
        current=self.applied_correction
        dx,dy=target[0]-current[0],target[1]-current[1]
        scale=min(1.,.03*dt/max(math.hypot(dx,dy),1e-12))
        delta=wrap(target[2]-current[2])
        angle=max(-math.radians(.5)*dt,min(math.radians(.5)*dt,delta))
        self.applied_correction=(current[0]+scale*dx,current[1]+scale*dy,
            target[2] if abs(delta)<=math.radians(.5)*dt else wrap(current[2]+angle))

    def transform(self,status,message,now):
        """Keep control on fresh EKF; adopt only validated AMCL updates."""
        self.raw_valid=False
        if self.fault:raise ValueError(self.fault)
        if message.header.frame_id!='odom' or message.child_frame_id not in ('base_footprint','base_link'):
            raise ValueError('invalid EKF frame for control odom')
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not self.fresh(stamp,now,ODOM_MAX_AGE_S):
            raise IgnoredOdomSample('EKF odom is stale for control odometry')
        p,q=message.pose.pose.position,message.pose.pose.orientation
        values=(p.x,p.y,q.x,q.y,q.z,q.w)
        twist=message.twist.twist
        if (not all(number(v) for v in (*values,twist.linear.x,twist.linear.y,twist.angular.z))
                or abs(sum(v*v for v in values[2:])-1.)>.02):
            raise ValueError('invalid EKF pose/twist for control odometry')
        raw=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        if self.raw_stamp is not None:
            if stamp<self.raw_stamp:
                self.fault='EKF capture clock reversed; restart control odometry'
            elif stamp-self.raw_stamp>.5:
                self.fault='EKF capture gap; restart control odometry'
            elif math.dist(raw[:2],self.raw_pose[:2])>.10 or abs(wrap(raw[2]-self.raw_pose[2]))>.15:
                self.fault='EKF pose jumped; restart control odometry'
            if self.fault:raise ValueError(self.fault)
            if stamp==self.raw_stamp:raise IgnoredOdomSample('duplicate EKF capture is not new control odometry')
        self.raw_stamp,self.raw_pose=stamp,raw
        if self.expected is not None and isinstance(status,dict):
            for key in ('run_id','map_file','baseline_map_odom'):
                if status.get(key) is not None and status[key]!=self.expected[key]:
                    self.fault='localization map/reference changed; restart control odometry'
                    raise ValueError(self.fault)
            if status.get('role') not in (None,'trial') or status.get('external_control_odom') is False:
                self.fault='control odom publisher ownership changed'
                raise ValueError(self.fault)
        if self.insertion_lock is not None:
            # Global AMCL remains available for observation. Its later changes
            # cannot turn a fork already entering the frozen pallet corridor.
            self.mode='insertion_locked_ekf'
            self.update_issue='AMCL frame updates deferred during C-to-B insertion'
            self.target_correction=self.applied_correction=tuple(self.insertion_lock['correction_xyyaw'])
            self.last_output_capture=stamp
            self.reference_state['correction_xyyaw']=list(self.applied_correction)
            output=self._apply(self.reference_state,message,raw)
            self.raw_valid=True
            return output
        if self.insertion_pending is not None:
            # The frozen RGB-D target and both pose consumers must agree
            # before heading alignment. Ignore new global corrections during
            # this bounded return; each output still uses its true EKF stamp.
            self.target_correction=self.insertion_capture_target
            self._slew_correction(self.target_correction,stamp)
            self.mode='insertion_frame_settling'
            self.last_output_capture=stamp
            if (math.dist(self.applied_correction[:2],self.target_correction[:2])<=1e-12
                    and abs(wrap(self.applied_correction[2]-self.target_correction[2]))<=1e-12):
                self._confirm_insertion_lock(self.insertion_pending,now)
            self.reference_state['correction_xyyaw']=list(self.applied_correction)
            output=self._apply(self.reference_state,message,raw)
            self.raw_valid=True
            return output
        try:
            candidate=ControlOdomGuard(self.map_file)
            candidate.expected=self.expected
            candidate._validate_reference(status,message,now)
            self.reference_state=copy.deepcopy(status)
            self.target_correction=pose_value(status['correction_xyyaw'])
            self.expected=copy.deepcopy(candidate.expected)
            self.run_id=candidate.run_id
            self.update_issue=''
            self.mode=('amcl_corrected_ekf' if status.get('live_observation_validated') is True
                       else 'validated_parked_frame' if status.get('stationary_cache',{}).get('validated') is True
                       else 'ekf_tracking')
        except (ValueError,TypeError,KeyError) as exc:
            self.update_issue=str(exc)
            if self.reference_state is None:raise ValueError(self.update_issue) from exc
            self.mode='ekf_tracking'
            self.target_correction=self.applied_correction
        if self.applied_correction is None:
            self.applied_correction=self.target_correction
        else:
            self._slew_correction(self.target_correction,stamp)
        self.last_output_capture=stamp
        self.reference_state['correction_xyyaw']=list(self.applied_correction)
        output=self._apply(self.reference_state,message,raw)
        self.raw_valid=True
        return output

    @staticmethod
    def fresh(value,now,limit):
        return number(value) and -.05<=now-value<=limit

    def _onset_ready(self,proof,source,pose,now):
        if (not isinstance(proof,dict) or proof.get('ended') is True
                or proof.get('source_amcl_stamp_s')!=source):
            raise ValueError('invalid control odom motion-onset proof')
        anchor=pose_value(proof.get('anchor_odom_xyyaw'))
        started=proof.get('started_ros_s')
        if (not self.fresh(started,now,2.) or math.dist(pose[:2],anchor[:2])>.08+1e-12
                or abs(wrap(pose[2]-anchor[2]))>.08+1e-12):
            raise ValueError('control odom motion onset exceeded 2 s / 8 cm / 0.08 rad')

    def _validate_reference(self,status,message,now):
        if self.fault:
            raise ValueError(self.fault)
        state=checked_status(status,now,self.expected)
        if state['map_file']!=self.map_file:
            raise ValueError('control odom belongs to a different selected map')
        if self.run_id is not None and state['run_id']!=self.run_id:
            self.fault='localization session changed; restart control odometry'
            raise ValueError(self.fault)
        self.run_id=state['run_id']
        if (state.get('external_control_odom') is not True or state.get('role')!='trial'
                or state.get('navigation_monitor') is not False
                or state.get('stationary_inspection') is True):
            raise ValueError('control odom is disabled until the trial handoff')
        if state.get('localization_fault'):
            raise ValueError(str(state['localization_fault']))
        if message.header.frame_id!='odom' or message.child_frame_id not in ('base_footprint','base_link'):
            raise ValueError('invalid EKF frame for control odom')
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not self.fresh(stamp,now,ODOM_MAX_AGE_S):
            raise ValueError('EKF odom is stale for control odometry')
        p,q=message.pose.pose.position,message.pose.pose.orientation
        values=(p.x,p.y,q.x,q.y,q.z,q.w)
        norm=sum(v*v for v in values[2:])
        twist=message.twist.twist
        if (not all(number(v) for v in (*values,twist.linear.x,twist.linear.y,twist.angular.z))
                or abs(norm-1.)>.02):
            raise ValueError('invalid EKF pose/twist for control odometry')
        raw=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        if self.last_stamp is not None:
            if stamp<self.last_stamp:
                self.fault='EKF capture clock reversed; restart control odometry'
                raise ValueError(self.fault)
            if stamp==self.last_stamp:
                raise ValueError('duplicate EKF capture is not new control odometry')
            if stamp-self.last_stamp>.5:
                self.invalid_after=now
            if (math.dist(raw[:2],self.last_pose[:2])>.10
                    or abs(wrap(raw[2]-self.last_pose[2]))>.15):
                self.fault='EKF pose jumped; restart control odometry'
                raise ValueError(self.fault)
        self.last_stamp,self.last_pose=stamp,raw
        fit=state.get('scan_map_fit')
        validate_recognition_scan_match(fit)
        if not self.fresh(fit.get('scan_stamp_s'),now,.5):
            raise ValueError('validated LiDAR scan is stale for control odometry')
        snapshot=state.get('stationary_snapshot') or {}
        source=state.get('amcl_stamp_s')
        if self.invalid_after is not None and (not number(source) or source<=self.invalid_after):
            raise ValueError('new AMCL observation required after an EKF capture gap')
        validate_runtime_particles(snapshot.get('particle_metrics'))
        if source!=snapshot.get('amcl_stamp_s'):
            raise ValueError('control odom AMCL source identity is inconsistent')
        if snapshot.get('run_id')!=state['run_id'] or snapshot.get('map_file')!=self.map_file:
            raise ValueError('control odom snapshot belongs to a different map/session')
        validate_pair_evidence(snapshot)
        if self.onset is not None and self.onset['source_amcl_stamp_s']!=source:
            self.onset=self.parked_witness=None
        if state.get('live_observation_validated') is True:
            if not all(self.fresh(s,now,2.) for s in (source,snapshot.get('cloud_stamp_s'))):
                raise ValueError('live AMCL observation is stale for control odometry')
            self.onset=self.parked_witness=None
        elif state.get('motion_prediction_validated') is True:
            self._onset_ready(state.get('motion_prediction'),source,raw,now)
            self.parked_witness=None
        elif state.get('stationary_cache',{}).get('validated') is True:
            anchor=pose_value(snapshot.get('odom_anchor_xyyaw'))
            stopped=(math.hypot(twist.linear.x,twist.linear.y)<.005
                and abs(twist.angular.z)<.01 and math.dist(raw[:2],anchor[:2])<=.005
                and abs(wrap(raw[2]-anchor[2]))<=math.radians(.5))
            if stopped and self.onset is None:
                self.parked_witness=dict(stamp=state['published_ros_s'],source=source,pose=raw)
            else:
                if self.onset is None:
                    witness=self.parked_witness
                    if (witness is None or witness['source']!=source
                            or not self.fresh(witness['stamp'],now,.25)):
                        raise ValueError('no recent parked validation for control odom motion onset')
                    self.onset=dict(started_ros_s=stamp,source_amcl_stamp_s=source,
                                    anchor_odom_xyyaw=witness['pose'])
                    self.parked_witness=None
                self._onset_ready(self.onset,source,raw,now)
        else:
            raise ValueError('supervisor has no validated live or parked localization')
        xy,yaw=state.get('xy_std_m'),state.get('yaw_std_deg')
        if not number(xy) or not 0.<=xy<=.10 or not number(yaw) or not 0.<=yaw<=5.:
            raise ValueError('invalid AMCL covariance bound for control odom')
        correction=pose_value(state['correction_xyyaw'])
        if self.expected is None:
            self.expected={key:copy.deepcopy(state[key]) for key in ('run_id','map_file','baseline_map_odom')}
        return self._apply(state,message,raw)

    @staticmethod
    def _apply(state,message,raw):
        correction=pose_value(state['correction_xyyaw'])
        xy,yaw=state['xy_std_m'],state['yaw_std_deg']
        output=copy.deepcopy(message)
        output.header.frame_id='r3_trial'
        output.child_frame_id='base_footprint'
        pose=compose(correction,raw)
        output.pose.pose.position.x,output.pose.pose.position.y=pose[:2]
        output.pose.pose.orientation.x=output.pose.pose.orientation.y=0.
        output.pose.pose.orientation.z=math.sin(pose[2]/2.)
        output.pose.pose.orientation.w=math.cos(pose[2]/2.)
        output.pose.covariance=rotate_covariance(message.pose.covariance,correction[2])
        output.pose.covariance[0]+=xy**2
        output.pose.covariance[7]+=xy**2
        output.pose.covariance[35]+=math.radians(yaw)**2
        return output


class ControlOdomNode(Node):
    def __init__(self,map_file,*,context=None):
        super().__init__('r3_control_odom',context=context)
        self.guard=ControlOdomGuard(map_file)
        self.control_stream_id=str(uuid.uuid4())
        self.state=None
        self.reason='waiting for localization trial handoff and fresh EKF'
        self.count=0
        self.last_output_stamp=None
        self.last_output_wall=None
        self.maximum_output_gap_s=0.
        self.insertion_lock_issue=''
        self.output=self.create_publisher(Odometry,'/pallet/r3/localized_odom',qos_profile_sensor_data)
        reliable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.health=self.create_publisher(String,'/pallet/r3/control_odom/status',reliable)
        self.create_subscription(String,'/pallet/r3/localization/status',self.on_state,reliable)
        self.create_subscription(String,'/pallet/r3/control_odom/insertion_request',self.on_insertion_request,reliable)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_timer(.05,self.publish_health)

    def on_insertion_request(self,message):
        try:
            self.guard.lock_for_insertion(json.loads(message.data),
                self.get_clock().now().nanoseconds*1e-9)
            self.insertion_lock_issue=''
            self.publish_health()
        except (ValueError,TypeError,KeyError) as exc:
            self.insertion_lock_issue=str(exc)

    def on_state(self,message):
        try:
            value=json.loads(message.data)
            if not isinstance(value,dict):raise ValueError('invalid localization status')
            self.state=value
        except (ValueError,TypeError):
            self.state=None
            self.reason='invalid localization status'

    def on_odom(self,message):
        now=self.get_clock().now().nanoseconds*1e-9
        previously_valid=self.guard.raw_valid
        try:
            output=self.guard.transform(self.state or {},message,now)
        except IgnoredOdomSample as exc:
            self.guard.raw_valid=bool(previously_valid and self.last_output_stamp is not None
                and -.05<=now-self.last_output_stamp<=ODOM_MAX_AGE_S)
            self.reason=str(exc)
            return
        except (ValueError,TypeError,KeyError) as exc:
            self.reason=str(exc)
            return
        wall=time.monotonic()
        if self.last_output_wall is not None:
            self.maximum_output_gap_s=max(self.maximum_output_gap_s,wall-self.last_output_wall)
        self.output.publish(output)
        self.count+=1
        self.last_output_stamp=output.header.stamp.sec+output.header.stamp.nanosec*1e-9
        self.last_output_wall=wall
        self.reason=('AMCL update delayed/rejected; continuing in the locked trial frame with fresh EKF'
                     if self.guard.mode=='ekf_tracking' else
                     'insertion frame held; fresh EKF/IMU tracks the straight C-to-B segment'
                     if self.guard.mode=='insertion_locked_ekf' else
                     'settling shared odom/LiDAR frame to the camera capture at A'
                     if self.guard.mode=='insertion_frame_settling' else
                     'fresh EKF sample transformed with a validated AMCL correction')

    def publish_health(self):
        now=self.get_clock().now().nanoseconds*1e-9
        ready=bool(self.guard.reference_state is not None and self.guard.raw_valid and not self.guard.fault
                   and self.last_output_stamp is not None and -.05<=now-self.last_output_stamp<=ODOM_MAX_AGE_S)
        state=dict(published_ros_s=now,ready=ready,output_samples=self.count,reason=self.guard.fault or self.reason,
            integrity_fault=self.guard.fault,control_stream_id=self.control_stream_id,
            latest_output_capture_stamp_s=self.last_output_stamp,
            output_capture_age_s=(now-self.last_output_stamp if self.last_output_stamp is not None else None),
            supervisor_validation_age_s=(now-self.state['published_ros_s']
                if isinstance(self.state,dict) and number(self.state.get('published_ros_s')) else None),
            maximum_output_gap_s=self.maximum_output_gap_s,original_capture_stamp_preserved=True,
            control_status_topic='/pallet/r3/control_odom/status',control_mode=self.guard.mode,
            amcl_update_issue=self.guard.update_issue,amcl_update_validated=self.guard.mode=='amcl_corrected_ekf',
            insertion_lock=self.guard.insertion_lock,insertion_lock_issue=self.insertion_lock_issue,
            insertion_pending=self.guard.insertion_pending,
            covariance_is_accuracy_proof=False)
        reference=self.guard.reference_state
        if reference is not None:
            for key in ('run_id','map_file','baseline_map_odom','correction_xyyaw','amcl_stamp_s','xy_std_m','yaw_std_deg'):
                state[key]=reference[key]
            state['frame_id']='r3_trial'
            state['supervisor_validated_ros_s']=reference['published_ros_s']
            state['amcl_source_age_s']=now-reference['amcl_stamp_s']
            state['raw_correction_xyyaw']=list(self.guard.target_correction)
            # A held correction is already settled in this control frame;
            # an unaccepted supervisor target never moves a frozen plan.
            distance=math.dist(self.guard.applied_correction[:2],self.guard.target_correction[:2])
            angle=abs(math.degrees(wrap(self.guard.applied_correction[2]-self.guard.target_correction[2])))
            state['correction_pending_translation_m']=distance
            state['correction_pending_yaw_deg']=angle
            state['correction_settled']=distance<=.005 and angle<=.2
        self.health.publish(String(data=json.dumps(state,allow_nan=False)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    args=parser.parse_args()
    if not args.map.is_absolute() or not args.map.is_file():
        parser.error('--map must name the selected existing absolute map YAML')
    rclpy.init()
    node=ControlOdomNode(args.map)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
