"""r3 shared frame geometry, immutable plans and swept-envelope validation.

No ROS/actuator imports. A simulation fixture is never a real calibration.
"""
from dataclasses import dataclass
import hashlib
import json
import math

R3_MIN_STRAIGHT_CLEARANCE_M = .10
R3_PLANNED_C_CLEARANCE_M = .15


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def validate_geometry(g):
    for key, expected in {'fork_length': .385, 'fork_pair_width': .155,
                          'tine_thickness': .003, 'pallet_length': .355,
                          'pallet_width': .355, 'leg_size': .050,
                          'leg_height': .100, 'rear_clearance': .300,
                          'stop_gap': .020, 'stop_gap_tolerance': .005,
                          'home_settle': .001, 'lift_target': .080}.items():
        if key not in g or g[key] is None or not math.isclose(g[key], expected, abs_tol=1e-9):
            raise ValueError(f'approved geometry/spec mismatch: {key}')
    if not (0 < g['max_speed'] <= .15 and 0 < g['max_lateral'] <= .020
            and 0 < g['max_yaw_deg'] <= .5):
        raise ValueError('approved motion limits exceeded')
    if len(g['base_fork']) != 3 or not all(v is not None and math.isfinite(v) for v in g['base_fork']):
        raise ValueError('SPEC_INCOMPLETE: base->fork transform')
    if g.get('housing_y') is None or not math.isclose(g['base_fork'][1]-g['housing_y'], -.020, abs_tol=1e-8):
        raise ValueError('fork/housing -20mm must be resolved in base Y')
    if len(g.get('pillar_centers_xy', [])) != 4:
        raise ValueError('four explicit pillar centers required')
    if not math.isclose(g['leg_height']+g['deck_thickness'], g['pallet_height'], abs_tol=1e-8):
        raise ValueError('pallet clear height plus deck differs from total height')
    if not math.isclose(g['fork_bottom_z']+g['tine_thickness'], .074, abs_tol=1e-8):
        raise ValueError('final HOME top must be74mm (trigger75mm minus settle1mm)')


def real_geometry_mode(g, allow_provisional=False):
    """Keep provisional hardware assumptions visible and opt-in at every process."""
    if (g.get('confirmed_real_geometry') is True and not g.get('missing_real')
            and g.get('provenance') != 'SYNTHETIC_FIXTURE_NOT_REAL_CALIBRATION'):
        return 'confirmed'
    if (allow_provisional and g.get('provenance') == 'USER_ACCEPTED_APPROXIMATE_REAL_TRIAL'
            and g.get('approximate_real_trial') is True
            and g.get('confirmed_real_geometry') is False
            and 0 < g.get('max_speed', 1) <= .10 and g.get('max_lateral', 1) <= .009
            and g.get('base_fork') == [.3655, 0., 0.]
            and math.isclose(g.get('housing_y', float('nan')), .02, abs_tol=1e-9)):
        return 'provisional'
    raise ValueError('real geometry unconfirmed; provisional trial requires explicit --allow-provisional')


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], wrap(a[2]+b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2])


def base_for_fork(fork, geometry):
    return compose(fork, inverse(geometry['base_fork']))


def fork_for_base(base, geometry):
    return compose(base, geometry['base_fork'])


def goal_base(target, geometry, gap=None):
    return base_for_fork(compose(target, (-(geometry['stop_gap'] if gap is None else gap), 0., 0.)), geometry)


def errors(base, target, geometry):
    f = compose(inverse(target), fork_for_base(base, geometry))
    return {'gap_m': -f[0], 'ey_m': f[1], 'yaw_error_rad': wrap(f[2])}


def validate_axis_lateral_residual(target, start):
    """Verify the separately reported empirical axis shift was applied once."""
    report = target.get('measurement_report') or {}
    if 'pallet_axis_lateral_residual_m' not in report:
        return  # Legacy frozen targets predate this explicit correction.
    keys = ('pallet_axis_lateral_residual_m', 'forward_base_m',
            'fork_lateral_corrected_m', 'entry_heading_deg',
            'target_forward_after_axis_residual_m', 'target_lateral_after_axis_residual_m')
    if any(not isinstance(report.get(k), (int, float)) or not math.isfinite(report[k]) for k in keys):
        raise ValueError('empirical lateral correction report is incomplete')
    residual = report['pallet_axis_lateral_residual_m']
    if abs(residual) > .05 or not math.isclose(
            (target.get('applied_bias') or {}).get('pallet_axis_lateral_residual_m', float('nan')),
            residual, abs_tol=1e-9):
        raise ValueError('empirical lateral correction provenance mismatch')
    corrected = compose((report['forward_base_m'], report['fork_lateral_corrected_m'],
                         math.radians(report['entry_heading_deg'])), (0., residual, 0.))
    reported_xy = (report['target_forward_after_axis_residual_m'],
                   report['target_lateral_after_axis_residual_m'])
    expected = compose(start, corrected)
    if (math.dist(corrected[:2], reported_xy) > 1e-7
            or math.dist(expected[:2], target['pose'][:2]) > 1e-7
            or abs(wrap(expected[2]-target['pose'][2])) > 1e-7):
        raise ValueError('empirical axis correction was not applied exactly once')


def rectangle(pose, bounds):
    x0, x1, y0, y1 = bounds
    return [compose(pose, (x, y, 0.))[:2] for x, y in
            ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]


def overlap(a, b):
    """Separating-axis test for oriented convex rectangles, including contact."""
    # Most pickup checks compare disjoint rectangles. Reject disjoint axis
    # bounds before the oriented SAT; overlapping bounds still use the full
    # test below. Contact remains occupied because all comparisons are strict.
    ax, ay = zip(*a)
    bx, by = zip(*b)
    if (max(ax) < min(bx) or max(bx) < min(ax)
            or max(ay) < min(by) or max(by) < min(ay)):
        return False
    for poly in (a, b):
        for p, q in zip(poly, poly[1:]+poly[:1]):
            nx, ny = -(q[1]-p[1]), q[0]-p[0]
            aa = [x*nx+y*ny for x, y in a]
            bb = [x*nx+y*ny for x, y in b]
            if max(aa) < min(bb) or max(bb) < min(aa):
                return False
    return True


def polygons(base, target, g):
    f = fork_for_base(base, g)
    tines = [rectangle(f, (0., g['fork_length'], y-g['tine_width']/2, y+g['tine_width']/2))
             for y in g['tine_centers_y']]
    half = g['pallet_width']/2
    leg = g['leg_size']
    legs = [rectangle(target, (x-leg/2, x+leg/2, y-leg/2, y+leg/2))
            for x, y in g['pillar_centers_xy']]
    return rectangle(base, g['body_bounds']), tines, legs


def collision(base, target, g):
    body, tines, legs = polygons(base, target, g)
    if any(overlap(body, p) for p in legs):
        return 'body/pillar'
    # Body and mast cannot pass through the deck even if between the pillars.
    deck = rectangle(target, (0., g['pallet_length'], -g['pallet_width']/2, g['pallet_width']/2))
    if overlap(body, deck):
        return 'body/deck'
    if any(overlap(tine, p) for tine in tines for p in legs):
        return 'tine/pillar'
    if any(overlap(tine, deck) for tine in tines):
        if g['fork_bottom_z'] <= 0.:
            return 'tine/floor'
        if g['fork_bottom_z']+g['tine_thickness'] >= g['leg_height']:
            return 'tine/deck'
    rear = rectangle(target, (g['pallet_length']+g['rear_clearance'],
                             g['pallet_length']+g['rear_clearance']+.01, -2., 2.))
    if overlap(body, rear) or any(overlap(t, rear) for t in tines):
        return 'rear clearance exceeded'
    return None


def tangent_mismatch(a,b):
    """Distance of a forward chord from the interval of endpoint tangents."""
    heading=math.atan2(b[1]-a[1],b[0]-a[0])
    delta=wrap(b[2]-a[2]);chord=wrap(heading-a[2])
    low,high=sorted((0.,delta))
    return max(low-chord,0.,chord-high)


def dense(poses, step=.002, angle_step=math.radians(.1)):
    yield tuple(poses[0])
    for a, b in zip(poses, poses[1:]):
        n = max(1, math.ceil(math.hypot(b[0]-a[0], b[1]-a[1])/step),
                math.ceil(abs(wrap(b[2]-a[2]))/angle_step))
        for i in range(1, n+1):
            u = i/n
            yield (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]), wrap(a[2]+u*wrap(b[2]-a[2])))


def validate_phases(phases, target, g):
    validate_geometry(g)
    if abs(g['base_fork'][2]) > 1e-8:
        raise ValueError('nonzero fork mounting yaw incompatible with exact axis line at vy=0')
    previous = None
    for phase in phases:
        poses = phase['poses']
        if len(poses) < 2:
            raise ValueError('phase has fewer than two poses')
        if previous is not None and (math.dist(previous[:2], poses[0][:2]) > 1e-7
                                   or abs(wrap(previous[2]-poses[0][2])) > 1e-7):
            raise ValueError('phase boundary discontinuity')
        for p in dense(poses):
            hit = collision(p, target, g)
            if hit:
                raise ValueError(hit)
            if phase['type'] == 'ALIGN_YAW':
                # Entire rotation envelope must remain before entry.
                f = fork_for_base(p, g)
                if errors(p, target, g)['gap_m'] <= g['fork_length']:
                    raise ValueError('rotation while fork inserted')
            if phase['type'] == 'STRAIGHT':
                e = errors(p, target, g)
                if abs(e['ey_m']) > 1e-8 or abs(e['yaw_error_rad']) > 1e-8:
                    raise ValueError('terminal line is not exact')
        for a, b in zip(poses, poses[1:]):
            ds = math.dist(a[:2], b[:2])
            if ds < 1e-8:
                if phase['type'] != 'ALIGN_YAW' and abs(wrap(b[2]-a[2])) > 1e-7:
                    raise ValueError('unlabelled rotation')
                continue
            heading = math.atan2(b[1]-a[1], b[0]-a[0])
            # A chord of a curve with varying curvature lies between its
            # endpoint tangents; it need not bisect their angular increment.
            # The old half-increment test falsely rejected valid Ceres curves
            # near the fixed start direction holders. Keep the same 0.5deg
            # discretization allowance and all terminal yaw/lateral limits.
            mismatch=tangent_mismatch(a,b)
            if mismatch > math.radians(.5):
                raise ValueError('nonholonomic/forward tangent check failed')
            # The diff primitive radius is a search parameter, not a hard
            # minimum vehicle radius: skid-steer also supports pure rotation.
            # Feasible speed/omega/acceleration are verified by the follower.
        previous = poses[-1]
    line = phases[-1]
    if line['type'] != 'STRAIGHT' or errors(line['poses'][0], target, g)['gap_m'] <= g['fork_length']:
        raise ValueError('required terminal line starts after fork entry')
    if abs(errors(line['poses'][-1], target, g)['gap_m']-g['stop_gap']) > 1e-9:
        raise ValueError('incorrect root goal gap')
    # A nonzero curvature jump is executable only at a stopped phase boundary.
    # The terminal line remains exact; never deform it to hide this jump.
    approach = phases[-2]['poses']
    a, b = approach[-2:]
    ds = math.dist(a[:2], b[:2])
    if ds and abs(wrap(b[2]-a[2]))/ds > .10 and not line.get('stop_at_join', False):
        raise ValueError('curvature discontinuity requires stopped phase boundary')


class PlanReferenceError(ValueError):
    """A deterministic frame/run mismatch cannot be repaired by more images."""


def validate_plan_reference(snapshot, config):
    if (snapshot.get('frame_id') not in ('odom', 'r3_trial')
            or snapshot.get('feature') != 'front_entry_corridor_center'):
        raise PlanReferenceError('target must be front entry center in odom or validated r3_trial')
    # The AMCL bridge supplies a bounded trial frame to both capture and
    # tracking. Keep that identity rather than relabelling its coordinates
    # as raw odom. Import here because the contract shares core geometry.
    from r3_localization_contract import localization_spec
    try:
        spec = localization_spec(snapshot)
    except (ValueError, TypeError) as exc:
        raise PlanReferenceError(str(exc)) from exc
    if (config.get('localization') or None) != spec:
        raise PlanReferenceError('localization metadata differs between target and plan')


@dataclass(frozen=True)
class FrozenPlan:
    target_json: str
    geometry_json: str
    phases_json: str
    config_json: str

    @classmethod
    def create(cls, snapshot, geometry, phases, config):
        validate_plan_reference(snapshot, config)
        validate_phases(phases, snapshot['pose'], geometry)
        return cls(*(json.dumps(v, sort_keys=True, allow_nan=False) for v in (snapshot, geometry, phases, config)))

    @property
    def plan_id(self):
        return digest(vars(self))


def corrected_base_scalar_target(*, odom_at_capture, forward_base_m, lateral_base_m,
                                 entry_heading_rad, metadata):
    """Already corrected base scalars: never apply an optical TF again."""
    required = ('feature', 'applied_bias', 'calibration_hash', 'capture_stamp_s', 'source_frame')
    if any(k not in metadata for k in required):
        raise ValueError('missing scalar measurement metadata')
    if metadata['source_frame'] != 'base_link_corrected_scalars':
        raise ValueError('explicit base-scalar adapter required')
    if not all(math.isfinite(v) for v in (*odom_at_capture, forward_base_m, lateral_base_m, entry_heading_rad)):
        raise ValueError('nonfinite measurement')
    return dict(metadata, frame_id='odom', pose=compose(odom_at_capture,
                (forward_base_m, lateral_base_m, entry_heading_rad)))


def optical_target(*, T_odom_base, T_base_camera_optical, T_camera_pallet, metadata):
    """Full SE(3) optical adapter. Unknown extrinsics cannot default to zero."""
    import numpy as np
    if any(v is None for v in (T_odom_base, T_base_camera_optical, T_camera_pallet)):
        raise ValueError('SPEC_INCOMPLETE: optical extrinsics missing')
    matrices = [np.asarray(v, dtype=float) for v in (T_odom_base, T_base_camera_optical, T_camera_pallet)]
    for m in matrices:
        if (m.shape != (4, 4) or not np.isfinite(m).all()
                or not np.allclose(m[3], [0, 0, 0, 1])
                or not np.allclose(m[:3, :3].T@m[:3, :3], np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(m[:3, :3]), 1.)):
            raise ValueError('invalid SE3 transform')
    m = matrices[0]@matrices[1]@matrices[2]
    if not all(k in metadata for k in ('feature', 'applied_bias', 'calibration_hash', 'capture_stamp_s')):
        raise ValueError('missing optical target metadata')
    return dict(metadata, frame_id='odom', pose=(float(m[0, 3]), float(m[1, 3]), math.atan2(m[1, 0], m[0, 0])))
