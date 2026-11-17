"""Geometric clearance for the real C-to-B insertion, in the pallet frame."""
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
