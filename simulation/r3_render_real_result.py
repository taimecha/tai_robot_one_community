#!/usr/bin/env python3
"""Compare planned and recorded base/fork poses using one consistent frame."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.patches import Polygon

from r3_core import FrozenPlan, errors, fork_for_base, polygons, rectangle, wrap, compose


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def pallet_view(point, target):
    """Rotate the display frame so the pallet is horizontal; leave odom data intact."""
    dx, dy = point[0]-target[0], point[1]-target[1]
    c, s = math.cos(target[2]), math.sin(target[2])
    return (target[0]+c*dx+s*dy, -s*dx+c*dy)


def configure_view(ax, view):
    """Reflect the entire drawing for the operator's laptop viewpoint."""
    if view not in ('laptop', 'ros'):
        raise ValueError('view must be laptop or ros')
    if ax.yaxis_inverted() != (view == 'laptop'):
        ax.invert_yaxis()
    direction = '+ xuống dưới, − lên trên' if view == 'laptop' else '+ lên trên, − xuống dưới'
    ax.set_ylabel(f'Y ngang pallet (m) · {direction}')


def execution_poses(rows, target, pose_source='encoder'):
    """Keep XY and yaw from the same estimator, with a single A reference."""
    if pose_source not in ('encoder', 'control'):
        raise ValueError('pose_source must be encoder or control')
    control = [(float(row['x_m']), float(row['y_m']),
                float(row.get('control_yaw_rad') or row['yaw_rad'])) for row in rows]
    raw_keys = ('raw_odom_x_m', 'raw_odom_y_m', 'raw_odom_yaw_rad')
    raw_available = all(all(row.get(key) not in (None, '') for key in raw_keys) for row in rows)
    correction = (target.get('localization') or {}).get('correction_at_capture_xyyaw', (0., 0., 0.))
    encoder = ([compose(correction, tuple(float(row[key]) for key in raw_keys)) for row in rows]
               if raw_available else None)
    if not all(math.isfinite(v) for poses in (control, encoder or []) for p in poses for v in p):
        raise ValueError('executed odom contains nonfinite coordinates')
    use_encoder = pose_source == 'encoder' and encoder is not None
    return dict(selected=encoder if use_encoder else control, control=control, encoder=encoder,
                source='encoder' if use_encoder else 'control')


def render(plan_path, trial_dir, output, view='laptop', pose_source='encoder'):
    saved = json.loads(plan_path.read_text())
    plan = FrozenPlan(**{key: saved[key] for key in
                         ('target_json', 'geometry_json', 'phases_json', 'config_json')})
    if saved.get('plan_id') != plan.plan_id:
        raise ValueError('frozen plan hash mismatch')
    mission = json.loads((trial_dir/'mission_result.json').read_text())
    if mission.get('plan_id') != plan.plan_id:
        raise ValueError('odom log belongs to a different plan')
    rows = read_rows(trial_dir/'executed_odom_base.csv')
    if not rows:
        raise ValueError('no executed odom samples; cannot draw the third trajectory')
    geometry = json.loads(plan.geometry_json)
    target_data = json.loads(plan.target_json)
    target = target_data['pose']
    phases = json.loads(plan.phases_json)
    approach_only = (mission.get('execution_mode') == 'arrive_at_c'
                     or mission.get('status') in ('C_ARRIVED', 'C_ALIGNMENT_WARNING'))
    displayed_phases = phases[:1] if approach_only else phases
    planned_base = [tuple(p) for phase in displayed_phases for p in phase['poses']]
    planned_fork = [fork_for_base(p, geometry) for p in planned_base]
    execution = execution_poses(rows, target_data, pose_source)
    odom = execution['selected']
    executed_fork = [fork_for_base(p, geometry) for p in odom]
    final_pose = odom[-1]
    final_control_yaw = execution['control'][-1][2]
    heading_label = ('encoder + IMU, gắn mốc A' if execution['source'] == 'encoder'
                     else 'ước lượng điều khiển')

    # All paths and all shapes use one rigid view transform. In this
    # frame the pallet is level and the initial robot carries the yaw error.
    planned_base_xy = [pallet_view(p, target) for p in planned_base]
    planned_fork_xy = [pallet_view(p, target) for p in planned_fork]
    odom_xy = [pallet_view(p, target) for p in odom]
    executed_fork_xy = [pallet_view(p, target) for p in executed_fork]
    fig = Figure(figsize=(12, 6), facecolor='white')
    ax = fig.subplots()
    ax.set_facecolor('#f8fafc')
    deck = [pallet_view(p, target) for p in rectangle(target, (0., geometry['pallet_length'],
                              -geometry['pallet_width']/2, geometry['pallet_width']/2))]
    ax.add_patch(Polygon(deck, facecolor='#25a18e', edgecolor='#0f766e',
                         alpha=.4, linewidth=2, zorder=1))
    body, tines, legs = polygons(planned_base[0], target, geometry)
    for leg in legs:
        ax.add_patch(Polygon([pallet_view(p, target) for p in leg],
                             facecolor='#0f766e', alpha=.7, zorder=2))
    body_xy = [pallet_view(p, target) for p in body]
    ax.add_patch(Polygon(body_xy, facecolor='#64748b', edgecolor='#334155',
                         alpha=.25, linewidth=1.5, zorder=2))
    for tine in tines:
        ax.add_patch(Polygon([pallet_view(p, target) for p in tine],
                             facecolor='#64748b', edgecolor='#334155',
                             alpha=.45, zorder=2))
    final_body, final_tines, _ = polygons(final_pose, target, geometry)
    final_body_xy = [pallet_view(p, target) for p in final_body]
    ax.add_patch(Polygon(final_body_xy, facecolor='none', edgecolor='#b45309',
                         linestyle='--', linewidth=2, zorder=5,
                         label=f'Tư thế cuối ước lượng · {heading_label}'))
    final_tine_xy = []
    for tine in final_tines:
        points = [pallet_view(p, target) for p in tine]
        final_tine_xy.extend(points)
        ax.add_patch(Polygon(points, facecolor='none', edgecolor='#b45309',
                             linestyle='--', linewidth=1.8, zorder=5))

    ax.plot([p[0] for p in planned_base_xy], [p[1] for p in planned_base_xy],
            color='#2563eb', linewidth=2.6, label='Tính toán · tâm xe (base_footprint)', zorder=3)
    ax.plot([p[0] for p in planned_fork_xy], [p[1] for p in planned_fork_xy],
            color='#e11d48', linewidth=2.6, label='Tính toán · tâm gốc càng', zorder=3)
    ax.plot([p[0] for p in odom_xy], [p[1] for p in odom_xy],
            color='#f59e0b', linewidth=2.2, linestyle='--',
            marker='o' if len(odom) == 1 else None,
            label='Ghi khi chạy · tâm odom', zorder=4)
    ax.plot([p[0] for p in executed_fork_xy], [p[1] for p in executed_fork_xy],
            color='#0f766e', linewidth=2.2, linestyle='-.',
            label='Ghi khi chạy · tâm gốc càng', zorder=4)
    if execution['source'] == 'encoder' and target_data.get('localization'):
        control_xy = [pallet_view(p, target) for p in execution['control']]
        ax.plot([p[0] for p in control_xy], [p[1] for p in control_xy],
                color='#64748b', linewidth=1.3, linestyle=':',
                label='Ước lượng điều khiển có hiệu chỉnh AMCL', zorder=3)

    a, c, b = planned_base[0], phases[0]['poses'][-1], phases[-1]['poses'][-1]
    waypoints = (('A', a), ('C', c)) if approach_only else (('A', a), ('C', c), ('B', b))
    for name, pose in waypoints:
        display_pose = pallet_view(pose, target)
        ax.scatter(display_pose[0], display_pose[1], s=35, facecolor='#172554', zorder=6)
        ax.annotate(name, display_pose, xytext=(7, 7), textcoords='offset points',
                    fontsize=11, weight='bold', color='#172554')
    ax.scatter(odom_xy[-1][0], odom_xy[-1][1], s=55, facecolor='#d97706',
               edgecolor='white', linewidth=1, zorder=7)
    ax.annotate('Odom cuối', odom_xy[-1], xytext=(9, -17), textcoords='offset points',
                fontsize=9, color='#92400e')
    fork_xy = executed_fork_xy[-1]
    ax.scatter(*fork_xy, s=38, facecolor='#0f766e', edgecolor='white', zorder=7)
    ax.plot([odom_xy[-1][0], fork_xy[0]], [odom_xy[-1][1], fork_xy[1]],
            color='#b45309', linewidth=1.2, zorder=5)
    ax.annotate('Gốc càng cuối', fork_xy, xytext=(-10, -29), textcoords='offset points',
                ha='right', fontsize=9, color='#0f766e')
    ax.annotate('Pallet', pallet_view(target, target), xytext=(8, 10), textcoords='offset points',
                fontsize=10, color='#065f46')
    a_xy = pallet_view(a, target)
    a_yaw = a[2]-target[2]
    ax.arrow(a_xy[0], a_xy[1], .12*math.cos(a_yaw), .12*math.sin(a_yaw),
             head_width=.025, head_length=.035, color='#475569', zorder=5,
             length_includes_head=True)

    all_xy = [*planned_base_xy, *planned_fork_xy, *odom_xy, *executed_fork_xy, *deck, *body_xy,
              *final_body_xy, *final_tine_xy]
    ax.set_xlim(min(p[0] for p in all_xy)-.18, max(p[0] for p in all_xy)+.18)
    ax.set_ylim(min(p[1] for p in all_xy)-.18, max(p[1] for p in all_xy)+.18)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X theo pallet (m)')
    configure_view(ax, view)
    ax.grid(alpha=.22)
    ax.legend(loc='best', framealpha=.95)
    status = mission.get('status', 'UNKNOWN')
    initial_yaw = math.degrees(a_yaw)
    final_error = errors(final_pose, target, geometry)
    final_yaw = math.degrees(final_error['yaw_error_rad'])
    control_yaw_error = math.degrees(wrap(final_control_yaw-target[2]))
    tip_clearance_cm = 100*(final_error['gap_m']-geometry['fork_length'])
    tip_label = (f'độ sâu chèn ước tính {-tip_clearance_cm:.1f} cm' if tip_clearance_cm < 0
                 else f'đầu càng còn cách pallet {tip_clearance_cm:.1f} cm')
    status_label = ('ĐÃ TỚI B · CÀNG NÂNG 8 CM' if status == 'PICKUP_COMPLETE' else
                    'ĐÃ TỚI C' if status == 'C_ARRIVED' else
                    'ĐÃ HẾT A→C · CÒN SAI SỐ C' if status == 'C_ALIGNMENT_WARNING' else
                    'ĐÃ DỪNG TRƯỚC C' if status == 'C_DIAGNOSTIC_DONE' else status)
    view_label = 'GÓC NHÌN LAPTOP · Y DƯƠNG XUỐNG' if view == 'laptop' else 'Y DƯƠNG LÊN'
    path_label = 'A → C' if approach_only else 'A → C → B'
    ax.set_title(f'SƠ ĐỒ ƯỚC LƯỢNG {path_label} · {status_label} · {view_label}\n'
                 f'Pallet: RGB-D khóa tại A  |  Xe: {heading_label}  |  '
                 'không đo lại pallet lúc dừng\n'
                 f'Góc tại A {initial_yaw:+.2f}° · góc cuối ước lượng {final_yaw:+.2f}° '
                 f'(điều khiển {control_yaw_error:+.2f}°) · {tip_label}\n'
                 f'Tâm odom → gốc càng: {geometry["base_fork"][0]*100:.2f} cm dọc xe '
                 f'· {len(odom)} mẫu', fontsize=9.5)
    # Show the signed estimate explicitly. This is a heading vector, never a
    # physical measurement inferred from the operator's photograph.
    final_xy = odom_xy[-1]
    final_display_yaw = wrap(final_pose[2]-target[2])
    ax.annotate('', xy=(final_xy[0]+.30*math.cos(final_display_yaw),
                        final_xy[1]+.30*math.sin(final_display_yaw)),
                xytext=final_xy, arrowprops=dict(arrowstyle='->', color='#b45309', lw=2),
                zorder=8)
    observation_path = trial_dir.parent/'physical_observation.json'
    if observation_path.exists():
        observed = json.loads(observation_path.read_text()).get('display_vi')
        if observed:
            ax.text(.99, .02, observed, transform=ax.transAxes, ha='right', va='bottom',
                    fontsize=9, color='#991b1b',
                    bbox=dict(facecolor='white', edgecolor='#991b1b', alpha=.95))
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170)
    fig.savefig(output.with_suffix('.svg'))
    return {'status': status, 'image': str(output),
            'svg': str(output.with_suffix('.svg')),
            'odom_samples': len(odom), 'plan_id': plan.plan_id, 'view': view,
            'pose_source': execution['source'], 'final_yaw_error_deg': final_yaw,
            'final_control_yaw_error_deg': control_yaw_error,
            'physical_pose_independently_measured': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--trial', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--view', choices=('laptop', 'ros'), default='laptop',
                        help='ros keeps positive Y up; laptop reflects the entire drawing')
    parser.add_argument('--pose-source', choices=('encoder', 'control'), default='encoder',
                        help='encoder keeps XY and yaw anchored to A; control includes applied map corrections')
    args = parser.parse_args()
    result = render(args.plan, args.trial, args.output, args.view, args.pose_source)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
