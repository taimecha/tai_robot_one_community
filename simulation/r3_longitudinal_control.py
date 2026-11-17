"""Bounded measured-speed feedback for the frozen-path follower; no ROS I/O."""
import math


def profiled_speed(remaining, acceleration, latency_s, measured_speed, previous_reference):
    """Reserve the measured overspeed before choosing the next reference.

    A reference-only brake curve cannot reserve the extra distance travelled
    while a real drive catches up with a lower velocity request.
    """
    values = (remaining, acceleration, latency_s, previous_reference)
    if (not all(math.isfinite(v) for v in values) or acceleration <= 0
            or latency_s < 0 or previous_reference < 0
            or measured_speed is not None and not math.isfinite(measured_speed)):
        raise ValueError('invalid measured braking profile')
    if measured_speed is None:
        # Planning/legacy callers have no observed velocity response.
        return (math.sqrt((acceleration*latency_s)**2
                +2*acceleration*max(0., remaining))-acceleration*latency_s, 0.)
    actual = max(0., measured_speed)
    brake_reserve = max(0., (actual**2-previous_reference**2)/(2*acceleration))
    # The old quadratic used the SMALLER next request for the delay term.
    # The drive still follows its current velocity during that delay.
    budget = max(0., remaining-brake_reserve)
    requested_limit = math.sqrt((acceleration*latency_s)**2+2*acceleration*budget)-acceleration*latency_s
    measured_limit = math.sqrt(2*acceleration*max(0., budget-actual*latency_s))
    # Enforce both delay envelopes: max(actual, next_reference)*latency.
    # At rest this retains a smooth feasible reference, rather than feeding
    # the preceding reference back into an oscillating distance budget.
    speed = min(requested_limit, measured_limit)
    reserve = max(0., actual-speed)*latency_s+brake_reserve
    return speed, reserve


class MeasuredSpeedControl:
    """Small PI trim with anti-windup, command slew and an exact zero stop.

    Trim is bounded independently of the speed reference. It compensates the
    observed tracking error, including a drive that barely moves on a small
    request, rather than imposing an unmeasured minimum running speed.
    Collision/braking checks still evaluate the complete resulting command.
    """
    kp = .8
    ki = .8
    integral_limit_mps = .04

    def __init__(self):
        self.reset()

    def reset(self):
        self.integral = 0.
        self.report = None

    def step(self, reference, measured, previous_command, dt, acceleration, max_speed):
        if (not all(math.isfinite(v) for v in
                    (reference, measured, previous_command, dt, acceleration, max_speed))
                or min(reference, previous_command, max_speed) < 0
                or max(reference, previous_command) > max_speed+1e-12
                or dt <= 0 or acceleration <= 0):
            raise ValueError('invalid longitudinal speed feedback')
        measured = max(0., measured)
        error = reference-measured
        if reference <= 1e-12:
            self.reset()
            self.report = dict(reference_speed_mps=0., measured_speed_mps=measured,
                error_mps=-measured, integral_mps=0., requested_speed_mps=0.,
                command_speed_mps=0., kp=self.kp, ki=self.ki,
                integral_limit_mps=self.integral_limit_mps)
            return 0.
        lower = max(0., previous_command-acceleration*dt)
        upper = min(max_speed, previous_command+acceleration*dt)
        candidate = max(-self.integral_limit_mps, min(self.integral_limit_mps,
            self.integral+self.ki*error*min(dt, .1)))
        requested = reference+self.kp*error+candidate
        limited = max(lower, min(upper, requested))
        if abs(requested-limited) < 1e-12 or error*(requested-limited) < 0:
            self.integral = candidate
        requested = reference+self.kp*error+self.integral
        command = max(lower, min(upper, requested))
        self.report = dict(reference_speed_mps=reference, measured_speed_mps=measured,
            error_mps=error, integral_mps=self.integral, requested_speed_mps=requested,
            command_speed_mps=command, kp=self.kp, ki=self.ki,
            integral_limit_mps=self.integral_limit_mps)
        return command
