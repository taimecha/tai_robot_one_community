"""Path tracking and the planning-time motion prediction used by the real robot.

Predictions validate a proposed path before motion; they are not measured poses.
"""
import csv
import itertools
import math

from r3_core import (R3_MIN_STRAIGHT_CLEARANCE_M, collision, compose, dense,
                     errors, fork_for_base, wrap)
from r3_insertion_clearance import insertion_motion_clearance
from r3_longitudinal_control import MeasuredSpeedControl, profiled_speed


def stationary_heading_command(current_yaw, goal_yaw, max_angular_rps):
    return max(-max_angular_rps, min(max_angular_rps,
                                   1.2 * wrap(goal_yaw-current_yaw)))


class PalletHeadingPID:
    """Track the pallet axis with heading and measured chassis-rate feedback.

    The feedback yaw stays in the camera-at-A frame. Map corrections must
    never move this angular reference after the pallet has been measured.
    Output saturation and conditional integration prevent windup at B.
    """
    kp = 1.2
    ki = .10
    kd = .10
    integral_limit_rad_s = math.radians(3.)
    rate_kp = .6
    rate_ki = 1.
    rate_integral_limit_rps = .04
    max_trim_rps = .06
    max_trim_curvature = 1.2
    settled_angle_rad = math.radians(.20)
    release_angle_rad = math.radians(.35)

    def __init__(self):
        self.integral = 0.
        self.rate_integral = 0.
        self.previous_error = None
        self.settled = False
        self.report = None

    def step(self, yaw, goal, yaw_rate, dt, speed):
        if (not all(math.isfinite(v) for v in (yaw, goal, yaw_rate, dt, speed))
                or dt <= 0 or speed < 0):
            raise ValueError('invalid pallet heading feedback')
        error = wrap(goal-yaw)
        limit = self.trim_limit(speed)
        if self.previous_error is not None and error*self.previous_error < 0:
            # Learned effort for tire scrub in one direction must not keep
            # steering past zero or be reused after reversing direction.
            self.reset_effort()
        self.previous_error = error
        if self.settled and (abs(error) > self.release_angle_rad or abs(yaw_rate) > .01):
            self.settled = False
        if abs(error) <= self.settled_angle_rad and abs(yaw_rate) <= .005:
            self.settled = True
        if self.settled or limit <= 1e-12:
            self.reset_effort()
            self.report = dict(target_yaw_rad=goal, feedback_yaw_rad=yaw,
                yaw_error_deg=math.degrees(wrap(yaw-goal)), yaw_rate_rps=yaw_rate,
                target_relative_to_pallet_deg=0., integral_rad_s=0.,
                target_yaw_rate_rps=0., rate_error_rps=-yaw_rate,
                kp=self.kp, ki=self.ki, kd=self.kd, rate_kp=self.rate_kp, rate_ki=self.rate_ki,
                rate_integral_rps=0., trim_limit_rps=limit, requested_omega_rps=0.,
                settled=self.settled, settled_band_deg=math.degrees(self.settled_angle_rad),
                release_band_deg=math.degrees(self.release_angle_rad))
            return 0.
        candidate = max(-self.integral_limit_rad_s, min(self.integral_limit_rad_s,
                        self.integral + error*min(dt, .1)))
        target_rate = self.kp*error + self.ki*candidate
        rate_error = target_rate-yaw_rate
        rate_candidate = max(-self.rate_integral_limit_rps,
            min(self.rate_integral_limit_rps,
                self.rate_integral+self.rate_ki*rate_error*min(dt, .1)))
        requested = target_rate+self.rate_kp*rate_error+rate_candidate-self.kd*yaw_rate
        if limit <= 1e-12:
            self.integral = 0.
            self.rate_integral = 0.
        elif abs(requested) <= limit or error*requested < 0:
            self.integral = candidate
        if limit > 1e-12 and (abs(requested) <= limit or rate_error*requested < 0):
            self.rate_integral = rate_candidate
        target_rate = self.kp*error+self.ki*self.integral
        requested = (target_rate+self.rate_kp*(target_rate-yaw_rate)
                     +self.rate_integral-self.kd*yaw_rate)
        self.report = dict(target_yaw_rad=goal, feedback_yaw_rad=yaw,
            yaw_error_deg=math.degrees(wrap(yaw-goal)), yaw_rate_rps=yaw_rate,
            target_relative_to_pallet_deg=0., integral_rad_s=self.integral,
            kp=self.kp, ki=self.ki, kd=self.kd, trim_limit_rps=limit,
            target_yaw_rate_rps=target_rate, rate_error_rps=target_rate-yaw_rate,
            rate_kp=self.rate_kp, rate_ki=self.rate_ki,
            rate_integral_rps=self.rate_integral,
            settled=self.settled, settled_band_deg=math.degrees(self.settled_angle_rad),
            release_band_deg=math.degrees(self.release_angle_rad),
            requested_omega_rps=requested)
        return max(-limit, min(limit, requested))

    @classmethod
    def trim_limit(cls, speed):
        return min(cls.max_trim_rps, cls.max_trim_curvature*speed)

    def reset_effort(self):
        self.integral = self.rate_integral = 0.


class PathFollower:
    """Pure Pursuit steering with an explicit start and a latched stop."""
    def __init__(self, path, *, max_speed=.10, acceleration=.08, dt=.05,
                 pre_dock_completion=True):
        self.path=path
        self.state='READY'
        self.index=0
        self.v=0.
        self.reason='operator start required'
        self.max_speed=min(.15, max(0., max_speed))
        self.acceleration=acceleration
        self.dt=dt
        self.pre_dock_completion=pre_dock_completion

    def start(self, operator_start):
        if operator_start and self.state=='READY':
            self.state='FOLLOW_CURVE'

    def stop(self, reason):
        self.state='STOP_LATCHED'
        self.v=0.
        self.reason=reason

    def step(self, x, y, yaw, restored=False):
        if self.state not in ('FOLLOW_CURVE','FOLLOW_STRAIGHT'):
            return 0.,0.
        nearest=min(range(self.index,min(self.index+35,len(self.path))),
                    key=lambda i:(self.path[i][0]-x)**2+(self.path[i][1]-y)**2)
        self.index=max(self.index,nearest)
        target=min(len(self.path)-1,self.index+12)
        tx,ty,_,_=self.path[target]
        dist=math.hypot(tx-x,ty-y)
        alpha=math.atan2(ty-y,tx-x)-yaw
        curvature=2*math.sin(alpha)/max(.10,dist)
        self.v=min(self.max_speed,self.v+self.acceleration*self.dt)
        w=max(-.20,min(.20,self.v*curvature))
        if self.index>=200:
            self.state='FOLLOW_STRAIGHT'
        if self.pre_dock_completion and math.hypot(self.path[-1][0]-x,self.path[-1][1]-y)<.02:
            self.state='WAIT_INSERTION_VERIFICATION'
            self.reason='pre-dock reached; insertion remains unverified'
            return 0.,0.
        return self.v,w


class FixedPlanFollower:
    """Follow a frozen path, then hold the pallet axis during pickup.

    Approach steering and stationary alignment retain their existing laws.
    Continuous pickup uses a bounded heading PID in the last straight lead-in
    and C->B; its pallet target is the camera measurement already locked at A.
    """
    # The drive cannot resolve a one-micrometre endpoint. Keep this below the
    # real trial's 5 mm terminal gap check and verify the actual odom there.
    endpoint_tolerance_m = .003
    min_straight_entry_clearance_m = R3_MIN_STRAIGHT_CLEARANCE_M
    approach_brake_m = .05
    alignment_done_deg = .25
    alignment_settle_s = .30
    c_position_tolerance_m = .02
    c_heading_tolerance_deg = 2.
    c_stop_trigger_m = .005
    c_command_latency_s = .20
    b_command_latency_s = .40
    clearance_stationary_time_s = .25

    def __init__(self, phases, target, geometry, approach_only=False,
                 continuous_insertion=False):
        self.phases, self.target, self.g = phases, target, geometry
        self.approach_only = approach_only
        self.continuous_insertion = continuous_insertion
        self.insertion_started = False
        self.insertion_heading = None
        self.insertion_heading_pid = PalletHeadingPID()
        self.insertion_heading_active = False
        self.insertion_heading_lead_in_m = 0.
        self.insertion_report = None
        self.insertion_camera_frame_report = None
        self.clearance_holds = []
        self.clearance_stationary_s = 0.
        self.c_join_pose = None
        if continuous_insertion:
            if approach_only or [p['type'] for p in phases] != ['APPROACH', 'STRAIGHT']:
                raise ValueError('continuous insertion requires an APPROACH and STRAIGHT plan')
            a, b = phases
            self.c_join_pose = a['poses'][-1]
            self.insertion_heading_lead_in_m = a.get('straight_lead_in_m', 0.)
            if (math.dist(self.c_join_pose[:2], b['poses'][0][:2]) > 1e-7
                    or abs(wrap(self.c_join_pose[2]-b['poses'][0][2])) > 1e-7):
                raise ValueError('C join is discontinuous')
            # Keep the same frozen geometry and tangents, with one speed
            # profile and one tracking index through C. Never stop to pivot.
            self.phases = [dict(a, poses=[*a['poses'], *b['poses'][1:]],
                                tracking_controller='pose_feedback',
                                straight_lead_in_m=0.)]
        if approach_only and (not phases or phases[0]['type'] != 'APPROACH'):
            raise ValueError('arrival at C requires an initial APPROACH phase')
        self.phase = 0
        self.state = 'PREVIEW_WAIT'
        self.follower = None
        self.v = 0.
        self.reference_v = 0.
        self.speed_control = MeasuredSpeedControl()
        self.braking_overspeed_reserve_m = 0.
        self.terminal_goal_report = None
        self.reason = ''
        self.align_goal = None
        self.w = 0.
        self.join_stopped_s = 0.
        self.align_stable_s = 0.
        self.curvature = []

    def reset_motion(self):
        """Restart from a real zero-command hold without retained speed effort."""
        self.v = self.w = self.reference_v = 0.
        self.speed_control.reset()
        self.insertion_heading_pid.reset_effort()
        self.clearance_stationary_s = 0.
        if self.follower is not None:
            self.follower.v = 0.

    def check_insertion_motion(self, pose, insertion_pose, speed, omega, latency, source):
        frames = [('control', pose)]
        if self.insertion_heading_active and insertion_pose is not None:
            frames.append(('camera-at-A', insertion_pose))
        blocked = None
        for frame, frame_pose in frames:
            report = insertion_motion_clearance(frame_pose, self.target, self.g,
                speed, omega, latency_s=latency)
            report['velocity_source'] = source
            if frame == 'camera-at-A':
                self.insertion_camera_frame_report = report
            else:
                self.insertion_report = report
            if not report['ready']:
                hit = report['first_intersection']
                if hit['time_s'] == 0. or hit['obstacle'] != 'tine/pillar':
                    return frame, report
                if blocked is None:
                    blocked = frame, report
        return blocked

    def clearance_hold_step(self, pose, insertion_pose, speed, omega, wheels_stopped, dt):
        hold = self.clearance_holds[-1]
        hold['duration_s'] += dt
        blocked = self.check_insertion_motion(pose, insertion_pose, abs(speed or 0.), omega,
                                             self.b_command_latency_s, 'measured_motion')
        if blocked:
            frame, report = blocked
            hit = report['first_intersection']
            if hit['time_s'] == 0. or hit['obstacle'] != 'tine/pillar':
                self.state, self.reason = 'STOP_LATCHED', frame+' '+report['reason']
                hold['outcome'] = 'unsafe_during_braking'
                return 0., 0.
        if (blocked is None and wheels_stopped and speed is not None
                and abs(speed) < .005 and abs(omega) < .005):
            self.clearance_stationary_s += dt
            if self.clearance_stationary_s >= self.clearance_stationary_time_s:
                self.state, self.reason = 'STRAIGHT', ''
                hold.update(outcome='resumed', resumed_pose_xyyaw=list(pose))
                return 0., 0.  # Check the first new command on the next tick.
        else:
            self.clearance_stationary_s = 0.
        if hold['duration_s'] >= hold['wait_limit_s']:
            self.state = 'STOP_LATCHED'
            self.reason = 'could not confirm a safe stop after '+hold['reason']
            hold['outcome'] = 'braking_timeout'
        return 0., 0.

    def align(self, pose, goal_yaw, dt):
        if errors(pose, self.target, self.g)['gap_m'] <= self.g['fork_length']:
            self.state, self.reason = 'STOP_LATCHED', 'rotation would occur after fork entry'
            return 0., 0.
        w = stationary_heading_command(pose[2], goal_yaw, self.g['max_omega'])
        dw = self.g['angular_acceleration']*dt
        w = max(self.w-dw, min(self.w+dw, w))
        # Check the full predicted sweep, including both tines.
        for p in dense([pose, (*pose[:2], wrap(pose[2]+w*dt))]):
            if collision(p, self.target, self.g):
                self.state, self.reason = 'STOP_LATCHED', 'alignment sweep occupied'
                return 0., 0.
        self.w = w
        return 0., w

    def start(self, operator_start):
        if not operator_start or self.state != 'PREVIEW_WAIT':
            return False
        self.state = self.phases[0]['type']
        return True

    def step(self, pose, dt=.05, obstacle=False, odom_valid=True, c_verified=True,
             insertion_yaw=None, insertion_yaw_rate=0., insertion_pose=None,
             measured_speed=None, wheels_stopped=False):
        if obstacle or not odom_valid:
            self.state = 'STOP_LATCHED'
            self.reason = 'obstacle stop' if obstacle else 'invalid odom'
        if self.state in ('PREVIEW_WAIT', 'STOP_LATCHED', 'STOPPED', 'C_REACHED'):
            return 0., 0.
        if measured_speed is not None and not math.isfinite(measured_speed):
            self.state, self.reason = 'STOP_LATCHED', 'invalid measured speed'
            return 0., 0.
        if self.state == 'CLEARANCE_HOLD':
            return self.clearance_hold_step(pose, insertion_pose, measured_speed,
                                            insertion_yaw_rate, wheels_stopped, dt)
        if self.align_goal is not None:
            if abs(wrap(self.align_goal-pose[2])) < math.radians(self.alignment_done_deg):
                self.w = 0.
                self.align_stable_s += dt
                if self.align_stable_s < self.alignment_settle_s:
                    return 0., 0.
                e = errors(pose, self.target, self.g)
                if (self.phase+1 < len(self.phases)
                        and self.phases[self.phase+1]['type'] == 'STRAIGHT'
                        and (abs(e['ey_m']) > self.g['max_lateral']
                             or abs(e['yaw_error_rad']) > math.radians(self.g['max_yaw_deg'])
                             or e['gap_m']-self.g['fork_length']
                             < self.min_straight_entry_clearance_m)):
                    self.state, self.reason = 'STOP_LATCHED', 'C alignment or fork-tip clearance failed'
                    return 0., 0.
                self.align_goal = None
                self.align_stable_s = 0.
                self.w = 0.
                if not c_verified:
                    self.state = 'WAIT_C_RGBD'
                    return 0., 0.
                self.phase += 1
                self.follower = None
                self.state = self.phases[self.phase]['type']
                return 0., 0.
            self.align_stable_s = 0.
            return self.align(pose, self.align_goal, dt)
        phase = self.phases[self.phase]
        if phase['type'] == 'ALIGN_YAW':
            goal_yaw = phase['poses'][-1][2]
            if abs(wrap(goal_yaw-pose[2])) < math.radians(self.alignment_done_deg):
                self.phase += 1
                self.follower = None
                self.state = self.phases[self.phase]['type']
                self.w = 0.
                return 0., 0.
            return self.align(pose, goal_yaw, dt)
        if self.follower is None:
            # Uniform spacing preserves the demo's 12-sample lookahead meaning.
            path = [(*p, 0.) for p in dense(phase['poses'], step=.01)]
            self.follower = PathFollower(path, max_speed=self.g['max_speed'],
                                           acceleration=self.g['acceleration'], dt=dt,
                                           pre_dock_completion=False)
            self.follower.start(True)
            self.curvature = [abs(wrap(b[2]-a[2]))/max(1e-9, math.dist(a[:2], b[:2]))
                              for a, b in zip(path, path[1:])]
            self.curvature.append(0.)
        e = errors(pose, self.target, self.g)
        end = phase['poses'][-1]
        dx, dy = end[0]-pose[0], end[1]-pose[1]
        remaining = dx*math.cos(end[2])+dy*math.sin(end[2])
        # At rest, use the SAME B tolerance as the post-braking/lift gate.
        # Chasing the tighter moving endpoint from an already valid stopped
        # pose can stall the drive and falsely end as "no odom progress".
        b_tolerance = min(.005, self.g['stop_gap_tolerance'])
        stationary_b = (self.continuous_insertion and measured_speed is not None
            and abs(measured_speed) < .005 and abs(insertion_yaw_rate) < .005
            and abs(remaining) <= b_tolerance
            and abs(e['gap_m']-self.g['stop_gap']) <= b_tolerance)
        if self.continuous_insertion:
            if collision(pose, self.target, self.g):
                self.state, self.reason = 'STOP_LATCHED', 'pickup footprint intersects pallet'
                return 0., 0.
            c = self.c_join_pose
            to_c = (c[0]-pose[0])*math.cos(c[2])+(c[1]-pose[1])*math.sin(c[2])
            # The planned last lead-in and C->B are already parallel to the
            # pallet axis. Start angular settling there while the tips are
            # still clear; keep the same PID state across C without a pivot.
            self.insertion_heading_active = (self.insertion_heading_active or self.insertion_started
                or to_c <= self.insertion_heading_lead_in_m)
            if self.insertion_heading_active:
                self.insertion_heading = wrap(self.target[2]-self.g['base_fork'][2])
            if to_c <= .005:
                self.insertion_started = True
                self.state = 'STRAIGHT'
        if self.approach_only:
            # C is a staging pose outside the pallet. The insertion-grade
            # lateral gate belongs to C->B, not to the approach's last 12 cm.
            if e['gap_m']-self.g['fork_length'] < self.min_straight_entry_clearance_m:
                self.state, self.reason = 'STOP_LATCHED', 'fork-tip clearance lost approaching C'
                return 0., 0.
            if remaining <= self.c_stop_trigger_m:
                self.v = self.w = self.follower.v = 0.
                self.state = 'C_REACHED'
                return 0., 0.
        if phase['type'] == 'STRAIGHT':
            if abs(e['ey_m']) > self.g['max_lateral'] or abs(e['yaw_error_rad']) > math.radians(self.g['max_yaw_deg']):
                self.state, self.reason = 'STOP_LATCHED', 'terminal yaw/lateral limit exceeded'
                return 0., 0.
        if remaining < -self.endpoint_tolerance_m and not stationary_b:
            self.state, self.reason = 'STOP_LATCHED', 'phase endpoint overshot'
            return 0., 0.
        if (not self.approach_only and phase['type'] == 'APPROACH' and self.phase+1 < len(self.phases)
                and self.phases[self.phase+1]['type'] == 'STRAIGHT'
                and remaining <= self.approach_brake_m):
            # Stop before C so Pure Pursuit cannot chase millimetres of
            # lateral error with a large yaw command at the path endpoint.
            self.v = self.w = 0.
            if self.follower is not None:
                self.follower.v = 0.
            if e['gap_m']-self.g['fork_length'] < self.min_straight_entry_clearance_m:
                self.state, self.reason = 'STOP_LATCHED', 'fork-tip clearance lost before C'
                return 0., 0.
            self.join_stopped_s += dt
            if self.join_stopped_s < .5:
                self.state = 'C_SETTLE'
                return 0., 0.
            if abs(wrap(end[2]-pose[2])) >= math.radians(self.g['max_yaw_deg']):
                self.align_goal = end[2]
                self.align_stable_s = 0.
                self.state = 'ALIGN_YAW'
                return 0., 0.
            if abs(e['ey_m']) > self.g['max_lateral']:
                self.state, self.reason = 'STOP_LATCHED', 'C lateral limit exceeded'
                return 0., 0.
            if not c_verified:
                self.state = 'WAIT_C_RGBD'
                return 0., 0.
            self.phase += 1
            self.follower = None
            self.join_stopped_s = 0.
            self.state = self.phases[self.phase]['type']
            return 0., 0.
        if abs(remaining) <= self.endpoint_tolerance_m or stationary_b:
            self.v = 0.
            if self.phase+1 < len(self.phases):
                # Existing settle interval: do not switch curvature/align
                # while the previous angular command is still decelerating.
                self.w = 0.
                self.join_stopped_s += dt
                if self.join_stopped_s < .5:
                    return 0., 0.
                self.join_stopped_s = 0.
                # Correct yaw while the fork tips are still clear of the
                # pallet; yaw error alone can shift the fork-root lateral
                # position past the 9 mm insertion tolerance.
                if abs(wrap(end[2]-pose[2])) >= math.radians(self.g['max_yaw_deg']):
                    self.align_goal = end[2]
                    self.align_stable_s = 0.
                    self.state = 'ALIGN_YAW'
                    return 0., 0.
                if (abs(e['ey_m']) > self.g['max_lateral']
                        or e['gap_m']-self.g['fork_length']
                        < self.min_straight_entry_clearance_m):
                    self.state, self.reason = 'STOP_LATCHED', 'C lateral or fork-tip clearance failed'
                    return 0., 0.
                self.phase += 1
                self.follower = None
                self.state = self.phases[self.phase]['type']
                return 0., 0.
            if not self.continuous_insertion and abs(e['ey_m']) > self.g['max_lateral']:
                self.state, self.reason = 'STOP_LATCHED', 'phase endpoint lateral limit exceeded'
                return 0., 0.
            if (abs(e['gap_m'] - self.g['stop_gap']) > .005
                    or (not self.continuous_insertion
                        and abs(e['yaw_error_rad']) > math.radians(self.g['max_yaw_deg']))):
                self.state, self.reason = 'STOP_LATCHED', 'terminal odom goal criteria failed'
                return 0., 0.
            self.state = 'STOPPED'
            self.w = 0.
            self.terminal_goal_report = dict(stationary_tolerance_used=stationary_b,
                gap_m=e['gap_m'], target_gap_m=self.g['stop_gap'],
                gap_tolerance_m=b_tolerance,
                measured_speed_mps=measured_speed, measured_yaw_rate_rps=insertion_yaw_rate)
            return 0., 0.
        previous_v, previous_w = self.v, self.w
        speed_feedback = (self.approach_only or self.continuous_insertion) and measured_speed is not None
        previous_reference = self.reference_v if speed_feedback else previous_v
        v, w = self.follower.step(*pose)
        # Brake by available deceleration. A linear remaining-distance cap
        # drove the real robot below 1 mm/s near C, where its motors stalled.
        # The endpoint check above still sends exact zero at the odom target.
        if self.approach_only or self.continuous_insertion:
            # Reserve the measured command pipeline delay as well as braking
            # distance; keep the terminal speed high enough to reach C rather
            # than exponentially creeping toward an unreachable zero.
            a = self.g['acceleration']
            budget = max(0., remaining)
            # Include a delayed control cycle as well as transport at B,
            # whose longitudinal stopping tolerance is tighter than C's.
            latency = self.b_command_latency_s if self.continuous_insertion else self.c_command_latency_s
            limit, self.braking_overspeed_reserve_m = profiled_speed(
                budget, a, latency+(dt if speed_feedback else 0.), measured_speed, previous_reference)
            v = min(v, limit)
        else:
            v = min(v, math.sqrt(2*self.g['acceleration']*max(0., remaining)))
        # Same PP curvature, with speed profiling applied consistently to omega.
        tx, ty, _, _ = self.follower.path[min(len(self.follower.path)-1, self.follower.index+12)]
        dist = math.hypot(tx-pose[0], ty-pose[1])
        alpha = math.atan2(ty-pose[1], tx-pose[0])-pose[2]
        kappa = 2*math.sin(alpha)/max(.10, dist)
        # Brake ahead of curvature rather than saturating steering at cruise.
        # 80% of the existing omega limit reserves feedback correction room.
        ahead = 0.
        path = self.follower.path
        for j in range(self.follower.index, min(len(path), self.follower.index+80)):
            if j > self.follower.index:
                ahead += math.dist(path[j-1][:2], path[j][:2])
            curve_speed = min(self.g['max_speed'], .8*self.g['max_omega']/max(self.curvature[j], 1e-9))
            v = min(v, math.sqrt(curve_speed**2+2*self.g['acceleration']*ahead))
        if not ((self.approach_only or self.continuous_insertion)
                and phase.get('tracking_controller') == 'pose_feedback'):
            # Only Pure Pursuit commands this point-chasing curvature. Near
            # C its endpoint curvature can diverge despite small tangent
            # feedback, unnecessarily starving the actual drive of speed.
            v = min(v, .8*self.g['max_omega']/max(abs(kappa), 1e-9))
        # Profiling may ask for a sharper slowdown than the actuator allows.
        # Honor the existing acceleration/deceleration limit and let steering
        # saturation/error checks report any infeasible tracking transition.
        dv=self.g['acceleration']*dt
        v=max(previous_reference-dv,min(previous_reference+dv,v))
        v=max(0.,min(self.g['max_speed'],v))
        self.follower.v = self.reference_v = v
        if speed_feedback:
            v = self.speed_control.step(v, measured_speed, previous_v, dt,
                                       self.g['acceleration'], self.g['max_speed'])
        self.v = v
        tracking_speed = self.reference_v
        straight_hold = self.continuous_insertion and self.insertion_heading_active
        if straight_hold:
            # Geometry/collision checks retain the shared localized pose.
            # Angular feedback uses odometry in the camera-at-A frame, so
            # an AMCL yaw offset cannot masquerade as physical fork alignment.
            feedback_yaw = pose[2] if insertion_yaw is None else insertion_yaw
            try:
                desired_w = self.insertion_heading_pid.step(feedback_yaw,
                    self.insertion_heading, insertion_yaw_rate, dt, tracking_speed)
            except ValueError as exc:
                self.state, self.reason = 'STOP_LATCHED', str(exc)
                return 0., 0.
        elif phase.get('tracking_controller') == 'pose_feedback':
            # Follow the local path tangent rather than a point 12 cm ahead.
            # A fixed-distance Pure Pursuit target cuts across a shallow S and
            # responds late to its change of turn direction at this low speed.
            i = min(self.follower.index, len(path)-2)
            ref, nxt = path[i], path[i+1]
            ds = math.dist(ref[:2], nxt[:2])
            signed_curvature = wrap(nxt[2]-ref[2])/max(ds, 1e-9)
            lateral = (-(pose[0]-ref[0])*math.sin(ref[2])
                       +(pose[1]-ref[1])*math.cos(ref[2]))
            heading = wrap(pose[2]-ref[2])
            desired_w = (tracking_speed*signed_curvature - 2.*math.sin(heading)
                         - 16.*tracking_speed*lateral)
            # The same feedback law runs through the straight lead-in, so
            # lateral feedback is not discarded at its boundary.
            if (not self.approach_only and phase['type'] == 'APPROACH'
                    and remaining <= phase.get('straight_lead_in_m', 0.)
                    and abs(e['ey_m']) > self.g['max_lateral']):
                self.state, self.reason = 'STOP_LATCHED', 'straight lead-in lateral limit exceeded'
                return 0., 0.
        elif (phase['type'] == 'APPROACH' and phase.get('straight_lead_in_m', 0.) > 0
                and remaining <= phase['straight_lead_in_m']):
            # The last approach samples are a straight lead-in. Pure Pursuit
            # can continue steering for a few millimetres of lateral error
            # even after yaw reaches the pallet axis, reversing the final
            # heading. Hold the terminal heading here instead. A large
            # lateral miss cannot be repaired safely beside the pallet.
            if not self.approach_only and abs(e['ey_m']) > self.g['max_lateral']:
                self.state, self.reason = 'STOP_LATCHED', 'straight lead-in lateral limit exceeded'
                return 0., 0.
            desired_w = 1.2*wrap(end[2]-pose[2])
        else:
            desired_w = tracking_speed*kappa
        w = max(-self.g['max_omega'], min(self.g['max_omega'], desired_w))
        dw = self.g['angular_acceleration']*dt
        w = max(self.w-dw, min(self.w+dw, w))
        if straight_hold:
            # Small heading trims while translating; reducing speed at B
            # must also reduce steering, never turn into a stationary pivot.
            trim_limit = self.insertion_heading_pid.trim_limit(tracking_speed)
            w = max(-trim_limit, min(trim_limit, w))
        self.w = w
        if self.continuous_insertion:
            latency = self.b_command_latency_s if self.insertion_started else self.c_command_latency_s
            # Check both frames for previous and proposed commands before
            # treating a projected conflict in measured motion as recoverable.
            commands = [('previous_command', previous_v, previous_w), ('next_command', v, w)]
            if straight_hold or measured_speed is not None:
                commands.append(('measured_motion', max(previous_v, v, abs(measured_speed or 0.)),
                                 insertion_yaw_rate))
            checked = set()
            for source, speed, omega in commands:
                if (speed, omega) in checked:
                    continue
                checked.add((speed, omega))
                blocked = self.check_insertion_motion(pose, insertion_pose, speed, omega, latency, source)
                if blocked is None:
                    continue
                frame, report = blocked
                hit = report['first_intersection']
                self.reason = frame+' '+report['reason']
                last_resume = self.clearance_holds[-1].get('resumed_pose_xyyaw') if self.clearance_holds else None
                if (self.insertion_started and source == 'measured_motion' and measured_speed is not None
                        and hit['obstacle'] == 'tine/pillar' and hit['time_s'] > 0.
                        and (last_resume is None or math.dist(pose[:2], last_resume[:2]) >= .005)):
                    self.clearance_holds.append(dict(reason=self.reason, pose_xyyaw=list(pose),
                        projected_intersection=hit, outcome='braking', duration_s=0.,
                        wait_limit_s=latency+report['braking_time_s']+1.))
                    self.state = 'CLEARANCE_HOLD'
                    self.reset_motion()
                else:
                    self.state = 'STOP_LATCHED'
                    self.v = self.w = self.follower.v = 0.
                return 0., 0.
        return v, w


def predict_tracking(phases, target, g, start):
    follower = FixedPlanFollower(phases, target, g)
    follower.start(True)  # Operator event is simulated, never sent to a real robot.
    pose = tuple(start)
    rows = []
    for k in itertools.count():
        t = k*.05
        v, w = follower.step(pose)
        hit = next((collision(p, target, g) for p in dense([pose, compose(pose, (v*.05, 0., w*.05))])
                    if collision(p, target, g)), None)
        if hit:
            follower.state, follower.reason = 'STOP_LATCHED', 'executed collision: '+hit
            v = w = 0.
        e = errors(pose, target, g)
        f = fork_for_base(pose, g)
        rows.append(dict(time_s=t, state=follower.state, x_m=pose[0], y_m=pose[1], yaw_rad=pose[2],
                         fork_x_m=f[0], fork_y_m=f[1], fork_yaw_rad=f[2],
                         cmd_v_mps=v, cmd_w_rps=w, lift_m=0., **e))
        if follower.state in ('STOP_LATCHED', 'STOPPED'):
            # Confirm actual synthetic plant rest for the existing settle interval.
            for j in range(1, 11):
                rows.append(dict(rows[-1], time_s=t+j*.05))
            break
        # Exact constant-twist unicycle integration; synthetic odom equals plant
        # by construction. This cannot demonstrate Gazebo slip/contact accuracy.
        if abs(w) < 1e-10:
            pose = (pose[0]+v*.05*math.cos(pose[2]), pose[1]+v*.05*math.sin(pose[2]), pose[2])
        else:
            yaw = pose[2]+w*.05
            pose = (pose[0]+v/w*(math.sin(yaw)-math.sin(pose[2])),
                    pose[1]-v/w*(math.cos(yaw)-math.cos(pose[2])), wrap(yaw))
    return rows, follower


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
