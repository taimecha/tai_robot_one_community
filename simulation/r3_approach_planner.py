"""Bounded quintic approach planner used for the real A-to-C trajectory."""
import math

import numpy as np

from r3_core import compose, inverse, wrap


def minimum_jerk_approach(start, end, target, geometry, radius_bound, spacing_m=.01,
                          curvature_cap=.8):
    """Clamped quintic with a bounded shape search for the fork-root trace."""
    origin = (*target[:2], target[2])
    local = inverse(origin)
    a, c = compose(local, start), compose(local, end)
    length = c[0] - a[0]
    if length <= 0 or max(abs(a[2]), abs(c[2])) >= math.pi / 2 - .1:
        raise ValueError('minimum-jerk refinement requires forward X and finite endpoint slopes')
    b1 = length * math.tan(a[2])
    displacement = c[1] - a[1] - b1
    slope_change = length * math.tan(c[2]) - b1
    b3 = 10 * displacement - 4 * slope_change
    b4 = 7 * slope_change - 15 * displacement
    b5 = 6 * displacement - 3 * slope_change
    # These two basis functions and their first two derivatives vanish at both
    # ends. They can reshape A->C without changing A, C, their yaw, or the
    # zero-curvature join. Search for the smallest overshoot of the *fork root*,
    # which moves laterally during chassis yaw because it is ahead of base_link.
    samples = [i / 100 for i in range(101)]
    fx, fy, _ = geometry['base_fork']
    start_root_y = a[1] + fx*math.sin(a[2]) + fy*math.cos(a[2])
    goal_root_y = c[1] + fx*math.sin(c[2]) + fy*math.cos(c[2])
    side = 1 if goal_root_y >= start_root_y else -1
    curvature_limit = min(1 / radius_bound, curvature_cap)

    def evaluate(shape_a, shape_b, make_poses=False):
        poses, root_y = [], []
        max_curvature = 0.
        for u in samples if not make_poses else [i/max(2, math.ceil(length/spacing_m))
                                                 for i in range(max(2, math.ceil(length/spacing_m))+1)]:
            phi = u**3 * (1-u)**3
            phi1 = 3*u*u*(1-u)**3 - 3*u**3*(1-u)**2
            phi2 = 6*u*(1-u)**3 - 18*u*u*(1-u)**2 + 6*u**3*(1-u)
            psi = phi*(2*u-1)
            psi1 = phi1*(2*u-1) + 2*phi
            psi2 = phi2*(2*u-1) + 4*phi1
            y = (a[1] + b1*u + b3*u**3 + b4*u**4 + b5*u**5
                 + shape_a*phi + shape_b*psi)
            slope = (b1 + 3*b3*u*u + 4*b4*u**3 + 5*b5*u**4
                     + shape_a*phi1 + shape_b*psi1) / length
            second = (6*b3*u + 12*b4*u*u + 20*b5*u**3
                      + shape_a*phi2 + shape_b*psi2) / length**2
            max_curvature = max(max_curvature, abs(second) / (1+slope*slope)**1.5)
            root_y.append(y + (fx*slope+fy)/math.sqrt(1+slope*slope))
            if make_poses:
                poses.append(compose(origin, (a[0] + length*u, y, math.atan(slope))))
        overshoot = max(0., max(side*(v-goal_root_y) for v in root_y))
        wrong_way = sum(max(0., -side*(v-u)) for u,v in zip(root_y,root_y[1:]))
        score = overshoot + .15*wrong_way + .004*max_curvature
        return score, max_curvature, overshoot, poses

    # Evaluate the same bounded grid together. The scalar 61*61*101 loop
    # occupied the ROS capture loop for seconds on the Pi, making its own
    # unread recognition heartbeat appear lost. Neither the candidates nor
    # the curvature/fork-root objective change here.
    u = np.asarray(samples)
    phi = u**3 * (1-u)**3
    phi1 = 3*u*u*(1-u)**3 - 3*u**3*(1-u)**2
    phi2 = 6*u*(1-u)**3 - 18*u*u*(1-u)**2 + 6*u**3*(1-u)
    psi = phi*(2*u-1)
    psi1 = phi1*(2*u-1) + 2*phi
    psi2 = phi2*(2*u-1) + 4*phi1
    grid = np.arange(-30, 31)*.05
    shape_a = np.repeat(grid, len(grid))[:, None]
    shape_b = np.tile(grid, len(grid))[:, None]
    y = a[1] + b1*u + b3*u**3 + b4*u**4 + b5*u**5 + shape_a*phi + shape_b*psi
    slope = (b1 + 3*b3*u*u + 4*b4*u**3 + 5*b5*u**4
             + shape_a*phi1 + shape_b*psi1) / length
    second = (6*b3*u + 12*b4*u*u + 20*b5*u**3
              + shape_a*phi2 + shape_b*psi2) / length**2
    curvatures = np.max(np.abs(second)/(1+slope*slope)**1.5, axis=1)
    roots = y + (fx*slope+fy)/np.sqrt(1+slope*slope)
    overshoots = np.maximum(0., np.max(side*(roots-goal_root_y), axis=1))
    wrong_way = np.sum(np.maximum(0., -side*np.diff(roots, axis=1)), axis=1)
    scores = overshoots + .15*wrong_way + .004*curvatures
    choices = sorted((float(scores[k]), float(abs(shape_a[k,0])+abs(shape_b[k,0])),
                      float(shape_a[k,0]), float(shape_b[k,0]))
                     for k in np.flatnonzero(curvatures <= curvature_limit))
    if not choices:
        raise ValueError('no bounded smooth A-to-C refinement')
    # Recheck the selected grid member with the original scalar arithmetic;
    # a vector rounding difference must never admit an over-limit curve.
    for _, _, shape_a, shape_b in choices:
        if evaluate(shape_a, shape_b)[1] <= curvature_limit:
            break
    else:
        raise ValueError('no bounded smooth A-to-C refinement')
    _, max_curvature, overshoot, poses = evaluate(shape_a, shape_b, make_poses=True)
    poses[0], poses[-1] = tuple(start), tuple(end)
    return poses, max_curvature, overshoot, (shape_a, shape_b)
