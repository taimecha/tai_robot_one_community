"""Validate a parked AMCL snapshot using live odom and distinct LiDAR scans.

Motion and clock changes invalidate reuse until a new observation. The sensor
supervisor also invalidates capture gaps. A downstream reader can discard its
local evidence window after missed messages while the supervisor validates the
unchanged observation against the source streams.
"""
from collections import deque
import math


def _pose_ok(pose):
    return len(pose)==3 and all(math.isfinite(v) for v in pose)


class StationaryEvidence:
    def __init__(self):
        self.observation_stamp=None
        self.anchor=None
        self.metrics=None
        self.odom_stamp=None
        self.odom_pose=None
        self.twist=None
        self.scan_stamp=None
        self.scans=deque(maxlen=40)
        self.fault=None
        self.reason='waiting for an AMCL observation and live stationary evidence'

    def invalidate(self,reason):
        self.fault=str(reason)
        self.scans.clear()

    def restart_scan_window(self,reason):
        self.scans.clear()
        self.reason=reason

    def capture(self,stamp,odom_pose,metrics=None):
        if not math.isfinite(stamp) or not _pose_ok(odom_pose):
            self.invalidate('invalid stationary AMCL snapshot');return False
        if self.observation_stamp is not None and stamp<=self.observation_stamp:
            return False
        self.observation_stamp=stamp
        self.anchor=tuple(odom_pose)
        self.metrics=metrics
        self.fault=None
        self.scans.clear()
        self.reason='waiting for distinct fresh scans spanning one second'
        self._check_motion()
        return self.fault is None

    def _check_motion(self):
        if self.anchor is None or self.odom_pose is None or self.twist is None:return
        if math.hypot(*self.twist[:2])>=.005 or abs(self.twist[2])>=.01:
            self.invalidate('robot motion invalidated stationary AMCL snapshot')
        angle=math.atan2(math.sin(self.odom_pose[2]-self.anchor[2]),
                         math.cos(self.odom_pose[2]-self.anchor[2]))
        if math.dist(self.odom_pose[:2],self.anchor[:2])>.005 or abs(angle)>math.radians(.5):
            self.invalidate('robot displacement invalidated stationary AMCL snapshot')

    def update_odom(self,pose,twist,stamp,now,*,check_capture_continuity=True):
        if (not _pose_ok(pose) or not _pose_ok(twist)
                or not all(math.isfinite(v) for v in (stamp,now))
                or now-stamp<-.05):
            self.invalidate('invalid/future odom for stationary AMCL snapshot');return
        if self.observation_stamp is not None and self.odom_stamp is not None:
            if stamp<self.odom_stamp or (check_capture_continuity and stamp-self.odom_stamp>.5):
                self.invalidate('odom gap/clock change invalidated stationary AMCL snapshot')
            elif stamp-self.odom_stamp>.5:
                self.restart_scan_window('local odom delivery interrupted; collect fresh scan evidence')
        self.odom_stamp=stamp;self.odom_pose=tuple(pose);self.twist=tuple(twist)
        self._check_motion()
        if now-stamp>.25:
            self.restart_scan_window('fresh odom unavailable; waiting for a new fresh scan evidence window')

    def observe_scan(self,stamp,fit,now,*,check_capture_continuity=True):
        if not all(math.isfinite(v) for v in (stamp,now)) or now-stamp<-.05:
            self.invalidate('invalid/future LiDAR invalidated stationary AMCL snapshot');return
        if self.observation_stamp is not None and self.scan_stamp is not None:
            if stamp<self.scan_stamp or stamp-self.scan_stamp>.5:
                if check_capture_continuity:
                    self.invalidate('LiDAR gap/clock change invalidated stationary AMCL snapshot')
                else:
                    self.restart_scan_window('scan validation interrupted; collect a new fresh evidence window')
        previous=self.scan_stamp;self.scan_stamp=stamp
        if now-stamp>.5:
            self.scans.clear();self.reason='fresh LiDAR scan unavailable';return
        ratio=fit.get('inlier_ratio') if isinstance(fit,dict) else None
        median=fit.get('median_distance_m') if isinstance(fit,dict) else None
        if (not isinstance(fit,dict) or fit.get('ready') is not True
                or not isinstance(ratio,(int,float)) or not math.isfinite(ratio) or ratio<.65
                or not isinstance(median,(int,float)) or not math.isfinite(median) or median>.12):
            self.scans.clear();self.reason='fresh scan no longer fits the stationary AMCL snapshot';return
        if self.anchor is None or self.fault:return
        if previous is not None and stamp<=previous:return
        self.scans.append(stamp)
        while len(self.scans)>3 and stamp-self.scans[0]>1.5:self.scans.popleft()
        self.reason='stationary snapshot checked against fresh LiDAR and odom; original covariance retained'

    def ready(self,now):
        if not math.isfinite(now):return False,'invalid stationary evidence clock'
        if self.fault:return False,self.fault
        if self.anchor is None:return False,'waiting for a valid stationary AMCL snapshot'
        if self.odom_stamp is None or not -.05<=now-self.odom_stamp<=.25:
            return False,'stationary odom is missing/stale'
        if self.scan_stamp is None or not -.05<=now-self.scan_stamp<=.5:
            return False,'stationary LiDAR scan is missing/stale'
        if not self.scans:
            return False,self.reason
        if len(self.scans)<3 or self.scans[-1]-self.scans[0]<1.-1e-9:
            return False,'waiting for distinct matching scans spanning one second'
        return True,self.reason
