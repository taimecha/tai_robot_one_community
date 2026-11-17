"""Geometric clearance for the real pickup, in the pallet frame."""
import math

from r3_core import collision, compose, fork_for_base, inverse

MIN_INSERTION_CLEARANCE_M = .010


def insertion_clearance(base, target, geometry, *, project_to_b=True):
    """Check both complete tines against the inner faces of all four legs.

    Include a constant-heading continuation to B. All lateral extrema along
    that continuation occur at its endpoints, so no dense rollout is needed.
    The feedback controller can still steer; its next command is checked
    separately. These are estimated clearances, not surveyed measurements.
    """
    fork = compose(inverse(target), fork_for_base(base, geometry))
    if not all(math.isfinite(v) for v in fork):
        raise ValueError('nonfinite insertion pose')
    half_leg = geometry['leg_size']/2
    left_legs = [y-half_leg for _, y in geometry['pillar_centers_xy'] if y > 0]
    right_legs = [y+half_leg for _, y in geometry['pillar_centers_xy'] if y < 0]
    if len(left_legs) != 2 or len(right_legs) != 2:
        raise ValueError('insertion needs two explicit legs on each side')
    left_inner, right_inner = min(left_legs), max(right_legs)
    if left_inner <= right_inner:
        raise ValueError('pallet opening is invalid')
    width = geometry['tine_width']
    ys = [compose(fork, (x, y+edge*width/2, 0.))[1]
          for y in geometry['tine_centers_y']
          for x in (0., geometry['fork_length']) for edge in (-1, 1)]
    yaw = fork[2]
    forward = math.cos(yaw)
    shift = 0.
    distance = 0.
    if project_to_b and forward > .1:
        distance = max(0., -geometry['stop_gap']-fork[0])/forward
        shift = distance*math.sin(yaw)
    current_left = left_inner-max(ys)
    current_right = min(ys)-right_inner
    projected_left = left_inner-max(y+shift for y in ys)
    projected_right = min(y+shift for y in ys)-right_inner
    minimum = min(current_left, current_right, projected_left, projected_right)
    hit = collision(base, target, geometry)
    if hit is None and distance > 0:
        hit = collision(compose(base, (distance, 0., 0.)), target, geometry)
    ready = forward > .1 and hit is None and minimum >= MIN_INSERTION_CLEARANCE_M
    if forward <= .1:
        reason = 'forks do not face the pallet opening'
    elif hit:
        reason = 'insertion footprint intersects '+hit
    elif not ready:
        reason = (f'insertion clearance {minimum*1000:.1f} mm below '
                  f'{MIN_INSERTION_CLEARANCE_M*1000:.1f} mm')
    else:
        reason = 'both tines fit the pallet corridor with the required margin'
    return dict(ready=ready, reason=reason,
                minimum_clearance_m=minimum, required_clearance_m=MIN_INSERTION_CLEARANCE_M,
                left_clearance_m=current_left, right_clearance_m=current_right,
                projected_B_left_clearance_m=projected_left,
                projected_B_right_clearance_m=projected_right,
                fork_root_lateral_m=fork[1], heading_error_deg=math.degrees(yaw),
                gap_m=-fork[0], project_to_b=project_to_b,
                physical_clearance_independently_verified=False)


def braking_trajectory(base, speed, omega, acceleration, angular_acceleration, latency_s):
    """Yield (pose, time) while holding for latency, then braking each axis.

    Angular velocity reaches zero on its own schedule; a slow linear stop
    must not stretch a short gyro observation into a long constant-curvature
    turn. Break integration at the command delay and both zero crossings.
    Midpoint twists keep spatial steps <= 2 mm and angular steps <= 0.1 deg.
    """
    pose = tuple(base)
    yield pose, 0.
    if speed == 0. and omega == 0.:
        return
    linear_stop_s = speed/acceleration
    angular_stop_s = abs(omega)/angular_acceleration
    t = 0.
    for boundary in sorted({latency_s, latency_s+linear_stop_s,
                            latency_s+angular_stop_s}):
        while boundary-t > 1e-10:
            elapsed = max(0., t-latency_s)
            v = max(0., speed-acceleration*elapsed)
            w = max(0., abs(omega)-angular_acceleration*elapsed)
            dt = min(.05, boundary-t, .002/max(v, 1e-9),
                     math.radians(.1)/max(w, 1e-9))
            middle = max(0., t+dt/2-latency_s)
            v = max(0., speed-acceleration*middle)
            w = math.copysign(max(0., abs(omega)-angular_acceleration*middle), omega)
            angle = w*dt
            travel = ((v*dt, 0., angle) if abs(w) < 1e-9 else
                      (v/w*math.sin(angle), v/w*(1-math.cos(angle)), angle))
            pose = compose(pose, travel)
            t += dt
            yield pose, t


def insertion_motion_clearance(base, target, geometry, speed, omega, *, latency_s):
    """Check the commanded arc and its braking sweep against finite obstacles.

    A constant-heading projection all the way to B is not the motion of the
    heading controller. In particular, it must not veto its first correction
    while the forks are still outside the pallet. Instead, reserve command
    latency and a complete stop with separate linear/angular braking. Inflate all four
    legs by the same 10 mm margin used for static insertion checks. This also
    checks the body, deck, floor and rear clearance through ``collision``.
    """
    acceleration = geometry['acceleration']
    angular_acceleration = geometry['angular_acceleration']
    if (not all(math.isfinite(v) for v in (*base, *target, speed, omega, latency_s,
                                          acceleration, angular_acceleration))
            or speed < 0 or latency_s < 0
            or acceleration <= 0 or angular_acceleration <= 0):
        raise ValueError('invalid insertion motion prediction')
    linear_brake_s = speed/acceleration
    angular_brake_s = abs(omega)/angular_acceleration
    distance = speed*latency_s + speed*linear_brake_s/2
    angle = omega*latency_s + omega*angular_brake_s/2
    padded = dict(geometry, leg_size=geometry['leg_size']+2*MIN_INSERTION_CLEARANCE_M)
    hit = None
    intersection = None
    checked = 0
    for pose, elapsed in braking_trajectory(base, speed, omega, acceleration,
                                           angular_acceleration, latency_s):
        hit = collision(pose, target, padded)
        checked += 1
        if hit:
            intersection = dict(time_s=elapsed, base_pose_xyyaw=list(pose), obstacle=hit)
            break
    # Retain the familiar lateral diagnostics, but distinguish the infinitely
    # long corridor from the finite obstacle envelope actually enforced here.
    report = insertion_clearance(base, target, geometry, project_to_b=False)
    report['current_corridor_ready'] = report['ready']
    report['current_corridor_reason'] = report['reason']
    report.update(ready=hit is None,
        reason=(f'command/braking sweep intersects {hit} '
                f'with {MIN_INSERTION_CLEARANCE_M*1000:.1f} mm margin' if hit else
                'command and braking sweep clear of pallet obstacles'),
        validation_scope='finite_pallet_obstacles_command_and_braking_sweep',
        braking_model='independent_linear_and_angular_deceleration',
        command_speed_mps=speed, command_omega_rps=omega,
        command_latency_s=latency_s,
        braking_time_s=max(linear_brake_s, angular_brake_s),
        linear_braking_time_s=linear_brake_s, angular_braking_time_s=angular_brake_s,
        stopping_distance_m=distance, stopping_yaw_rad=angle,
        first_intersection=intersection, checked_poses=checked)
    return report
