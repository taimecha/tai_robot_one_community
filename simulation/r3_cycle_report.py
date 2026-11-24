#!/usr/bin/env python3
"""Render full-cycle trajectory onto the occupancy map and summarize cycle metrics."""
import argparse
import csv
import json
import math
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
import numpy as np
from PIL import Image
import yaml

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca*b[0] - sa*b[1],
            a[1] + sa*b[0] + ca*b[1],
            wrap(a[2] + b[2]))


def load_baseline_map_odom(cycle_dir):
    cycle_dir = Path(cycle_dir)
    for name in ('localization_at_A.json', 'localization_at_recognition.json'):
        path = cycle_dir / name
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        handoff = data.get('trial_handoff') if isinstance(data, dict) else None
        baseline = (handoff.get('baseline_map_odom') if isinstance(handoff, dict) else None)
        if not baseline and isinstance(data, dict):
            baseline = data.get('baseline_map_odom')
        if (isinstance(baseline, (list, tuple)) and len(baseline) == 3
                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in baseline)):
            return (float(baseline[0]), float(baseline[1]), float(baseline[2]))
    return None


def load_json(path):
    p = Path(path)
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return None


def fmt_duration(seconds):
    if seconds is None or not math.isfinite(seconds):
        return 'N/A'
    s = max(0.0, float(seconds))
    mins = int(s // 60)
    rem = s - mins * 60
    if mins > 0:
        return f'{mins} phút {rem:.1f} giây ({s:.1f} s)'
    return f'{s:.1f} giây'


def read_actual_csv(cycle_dir):
    p = cycle_dir / 'full_trajectory_actual.csv'
    pts = []
    if p.is_file():
        with p.open(newline='') as f:
            for row in csv.DictReader(f):
                try:
                    pts.append((float(row['x_m']), float(row['y_m']), float(row['yaw_rad']),
                                row.get('phase', '')))
                except (ValueError, KeyError):
                    continue
    if pts:
        return pts
    # Fallback for older runs without full_trajectory_actual.csv: map pickup odom.
    baseline = load_baseline_map_odom(cycle_dir)
    ex = cycle_dir / 'trial' / 'executed_odom_base.csv'
    if baseline is not None and ex.is_file():
        with ex.open(newline='') as f:
            for row in csv.DictReader(f):
                try:
                    x, y, yaw = compose(baseline, (float(row['x_m']), float(row['y_m']), float(row['yaw_rad'])))
                    pts.append((x, y, yaw, 'pickup'))
                except (ValueError, KeyError):
                    continue
    return pts


def read_planned_csv(cycle_dir):
    p = cycle_dir / 'full_trajectory_planned.csv'
    legs = {'to_A': [], 'pickup_A_to_B': [], 'to_D': []}
    if p.is_file():
        with p.open(newline='') as f:
            for row in csv.DictReader(f):
                try:
                    leg = row.get('leg', 'to_A')
                    legs.setdefault(leg, []).append(
                        (float(row['x_m']), float(row['y_m']), float(row['yaw_rad'])))
                except (ValueError, KeyError):
                    continue
    # Fill pickup leg from CSV + baseline if not yet in full_trajectory_planned.csv
    if not legs['pickup_A_to_B']:
        baseline = load_baseline_map_odom(cycle_dir)
        csv_p = cycle_dir / 'plan' / 'path_planned_base.csv'
        if baseline is not None and csv_p.is_file():
            with csv_p.open(newline='') as f:
                for row in csv.DictReader(f):
                    try:
                        legs['pickup_A_to_B'].append(
                            compose(baseline, (float(row['x_m']), float(row['y_m']), float(row['yaw_rad']))))
                    except (ValueError, KeyError):
                        continue
    return legs


def parse_c_clearance_mm(cycle_dir, mission_result):
    if isinstance(mission_result, dict):
        ic = mission_result.get('insertion_clearance')
        if isinstance(ic, dict):
            for key in ('c_clearance_mm', 'min_clearance_mm', 'lateral_clearance_mm'):
                val = ic.get(key)
                if isinstance(val, (int, float)) and math.isfinite(val):
                    return float(val)
    trial_log = cycle_dir / 'trial.log'
    if trial_log.is_file():
        txt = trial_log.read_text(errors='ignore')
        m = re.search(r'khe hở ngang ước lượng tại C\s+([+-]?\d+(?:\.\d+)?)\s*mm', txt)
        if m:
            return float(m.group(1))
    return None


def compute_b_pose_summary(cycle_dir, mission_result):
    baseline = load_baseline_map_odom(cycle_dir)
    planned_csv = cycle_dir / 'plan' / 'path_planned_base.csv'
    executed_csv = cycle_dir / 'trial' / 'executed_odom_base.csv'
    planned_end = None
    actual_end = None
    if planned_csv.is_file():
        rows = list(csv.DictReader(planned_csv.open(newline='')))
        if rows:
            r = rows[-1]
            planned_end = (float(r['x_m']), float(r['y_m']), float(r['yaw_rad']))
    if executed_csv.is_file():
        rows = list(csv.DictReader(executed_csv.open(newline='')))
        if rows:
            r = rows[-1]
            yaw_val = float(r.get('control_yaw_rad') or r['yaw_rad'])
            actual_end = (float(r['x_m']), float(r['y_m']), yaw_val)

    pos_err_m = None
    yaw_err_deg = None
    actual_map = None
    planned_map = None
    if planned_end is not None and actual_end is not None:
        pos_err_m = math.hypot(actual_end[0] - planned_end[0], actual_end[1] - planned_end[1])
        yaw_err_deg = math.degrees(wrap(actual_end[2] - planned_end[2]))
    if baseline is not None:
        if actual_end is not None:
            actual_map = list(compose(baseline, actual_end))
        if planned_end is not None:
            planned_map = list(compose(baseline, planned_end))

    if isinstance(mission_result, dict):
        if mission_result.get('final_control_yaw_error_deg') is not None:
            yaw_err_deg = float(mission_result['final_control_yaw_error_deg'])
    tgc = (mission_result or {}).get('terminal_goal_check') or {}
    lift_fb = ((mission_result or {}).get('lift') or {}).get('feedback') or {}
    return {
        'status': (mission_result or {}).get('status'),
        'position_error_vs_planned_cm': round(pos_err_m * 100.0, 2) if pos_err_m is not None else None,
        'yaw_error_deg': round(yaw_err_deg, 2) if yaw_err_deg is not None else None,
        'gap_m': tgc.get('gap_m'),
        'target_gap_m': tgc.get('target_gap_m'),
        'lift_height_mm': round(float(lift_fb['position']) * 1000.0, 1) if 'position' in lift_fb else None,
        'actual_map_xyyaw': actual_map,
        'planned_map_xyyaw': planned_map,
    }


def build_summary(cycle_dir, cycle_index, start_wall, end_wall):
    nav_a = load_json(cycle_dir / 'navigation_to_A.json') or {}
    snap = load_json(cycle_dir / 'plan' / 'target_snapshot.json') or {}
    mr = load_json(cycle_dir / 'trial' / 'mission_result.json') or {}
    deliv_arr = load_json(cycle_dir / 'delivery_arrival.json') or {}
    deliv_res = load_json(cycle_dir / 'delivery_result.json') or {}

    duration_s = (end_wall - start_wall) if (start_wall is not None and end_wall is not None) else None

    # A results
    a_info = {
        'status': nav_a.get('status'),
        'position_error_cm': round(float(nav_a['position_error_m']) * 100.0, 2) if 'position_error_m' in nav_a else None,
        'heading_error_deg': round(float(nav_a['heading_error_deg']), 2) if 'heading_error_deg' in nav_a else None,
        'arrival_pose_xyyaw': nav_a.get('arrival_pose_xyyaw'),
        'goal_pose_xyyaw': nav_a.get('goal_pose_xyyaw'),
    }

    # Pallet RGB-D measurement
    rep = snap.get('measurement_report') or {}
    fwd_base = rep.get('forward_base_m')
    # Fork root is 0.365 m ahead of base; target_forward_after_axis_residual_m is from base,
    # while trial.log prints fork_root_to_pallet = forward_base_m - 0.365? Let's use frozen_plan if available.
    fp = load_json(cycle_dir / 'plan' / 'frozen_plan.json') or {}
    fork_dist_m = None
    trial_log = cycle_dir / 'trial.log'
    if trial_log.is_file():
        m = re.search(r'Khoảng cách gốc càng tới mặt pallet:\s*([0-9.]+)\s*m', trial_log.read_text(errors='ignore'))
        if m:
            fork_dist_m = float(m.group(1))
    if fork_dist_m is None and isinstance(fwd_base, (int, float)):
        fork_dist_m = round(float(fwd_base) - 0.365, 3)

    pallet_info = {
        'camera_lateral_raw_cm': round(float(rep['camera_lateral_raw_m']) * 100.0, 2) if 'camera_lateral_raw_m' in rep else None,
        'camera_mount_offset_cm': round(float(rep['camera_mount_offset_m']) * 100.0, 2) if 'camera_mount_offset_m' in rep else None,
        'fork_lateral_corrected_cm': round(float(rep['fork_lateral_corrected_m']) * 100.0, 2) if 'fork_lateral_corrected_m' in rep else None,
        'fork_root_to_pallet_m': round(fork_dist_m, 3) if fork_dist_m is not None else None,
        'entry_heading_deg': round(float(rep['entry_heading_deg']), 2) if 'entry_heading_deg' in rep else None,
        'heading_source': rep.get('heading_source'),
        'c_lateral_clearance_mm': parse_c_clearance_mm(cycle_dir, mr),
    }

    # B results
    b_info = compute_b_pose_summary(cycle_dir, mr)

    # D results (same format as A!)
    d_lift_fb = ((deliv_res.get('lift') or {}).get('feedback') or {})
    d_info = {
        'arrival_status': deliv_arr.get('status'),
        'result_status': deliv_res.get('status'),
        'position_error_cm': round(float(deliv_arr['position_error_m']) * 100.0, 2) if 'position_error_m' in deliv_arr else None,
        'heading_error_deg': round(float(deliv_arr['heading_error_deg']), 2) if 'heading_error_deg' in deliv_arr else None,
        'arrival_pose_xyyaw': deliv_arr.get('pose_xyyaw'),
        'goal_pose_xyyaw': deliv_arr.get('goal_pose_xyyaw'),
        'lower_height_mm': round(float(d_lift_fb['position']) * 1000.0, 1) if 'position' in d_lift_fb else None,
    }

    return {
        'cycle_index': int(cycle_index),
        'cycle_dir': str(cycle_dir),
        'cycle_start_wall_s': start_wall,
        'cycle_end_wall_s': end_wall,
        'cycle_duration_s': round(duration_s, 2) if duration_s is not None else None,
        'point_A': a_info,
        'pallet_measurement': pallet_info,
        'point_B': b_info,
        'point_D': d_info,
    }


def format_pose(xyyaw):
    if not isinstance(xyyaw, (list, tuple)) or len(xyyaw) != 3:
        return 'N/A'
    return f'X={xyyaw[0]:.3f} m, Y={xyyaw[1]:.3f} m, yaw={math.degrees(xyyaw[2]):+.2f}°'


def format_report_text(summary, png_path):
    a = summary['point_A']
    p = summary['pallet_measurement']
    b = summary['point_B']
    d = summary['point_D']
    lines = [
        '=' * 68,
        f'BÁO CÁO CHU TRÌNH #{summary["cycle_index"]} (VỀ A -> LẤY PALLET -> GIAO TẠI D)',
        '=' * 68,
        f'1. THỜI GIAN CHU TRÌNH (từ lúc bắt đầu chạy về A -> đặt pallet xong tại D):',
        f'   - Tổng thời gian : {fmt_duration(summary["cycle_duration_s"])}',
        '',
        f'2. KẾT QUẢ TẠI A (so với điểm A đã lưu):',
        f'   - Trạng thái     : {a["status"]}',
        f'   - Lệch vị trí    : {a["position_error_cm"]} cm | Lệch góc: {a["heading_error_deg"]}°',
        f'   - Thực tế tại A  : {format_pose(a["arrival_pose_xyyaw"])}',
        f'   - Điểm A đã lưu  : {format_pose(a["goal_pose_xyyaw"])}',
        '',
        f'3. ĐO VỊ TRÍ XE SO VỚI PALLET (RGB-D tại A):',
        f'   - Camera đo ngang thô      : {p["camera_lateral_raw_cm"]} cm (bù lắp camera: {p["camera_mount_offset_cm"]} cm)',
        f'   - Tâm pallet lệch gốc càng : {p["fork_lateral_corrected_cm"]} cm',
        f'   - Khoảng cách gốc càng     : {p["fork_root_to_pallet_m"]} m',
        f'   - Góc xe so với mặt pallet : {p["entry_heading_deg"]}° ({p["heading_source"]})',
        f'   - Khe hở ngang tại C       : {p["c_lateral_clearance_mm"]} mm',
        '',
        f'4. KẾT QUẢ TẠI B (lấy & nâng pallet):',
        f'   - Trạng thái               : {b["status"]}',
        f'   - Lệch vị trí vs B dự kiến : {b["position_error_vs_planned_cm"]} cm | Lệch góc cuối: {b["yaw_error_deg"]}°',
        f'   - Khoảng hở mặt pallet (B) : {round(b["gap_m"]*1000, 1) if b["gap_m"] is not None else "N/A"} mm (mục tiêu {round(b["target_gap_m"]*1000, 1) if b["target_gap_m"] is not None else "N/A"} mm)',
        f'   - Độ cao nâng càng (ESP32) : {b["lift_height_mm"]} mm',
        '',
        f'5. KẾT QUẢ TẠI D (so với điểm D đã lưu):',
        f'   - Trạng thái     : {d["arrival_status"]} -> {d["result_status"]}',
        f'   - Lệch vị trí    : {d["position_error_cm"]} cm | Lệch góc: {d["heading_error_deg"]}°',
        f'   - Thực tế tại D  : {format_pose(d["arrival_pose_xyyaw"])}',
        f'   - Điểm D đã lưu  : {format_pose(d["goal_pose_xyyaw"])}',
        f'   - Hạ càng (ESP32): {d["lower_height_mm"]} mm',
        '',
        f'6. TỆP QUỸ ĐẠO & BẰNG CHỨNG CHU TRÌNH #{summary["cycle_index"]}:',
        f'   - Ảnh quỹ đạo đầy đủ : {png_path}',
        f'   - CSV thực tế        : {Path(summary["cycle_dir"]) / "full_trajectory_actual.csv"}',
        f'   - CSV tính được      : {Path(summary["cycle_dir"]) / "full_trajectory_planned.csv"}',
        f'   - Ảnh A->C->B        : {Path(summary["cycle_dir"]) / "trajectory_result.png"}',
        '=' * 68,
    ]
    return '\n'.join(lines) + '\n'


def render_full_trajectory_map(cycle_dir, map_yaml_path, summary):
    map_yaml_path = Path(map_yaml_path)
    meta = yaml.safe_load(map_yaml_path.read_text())
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    img_path = Path(meta['image'])
    if not img_path.is_absolute():
        img_path = map_yaml_path.parent / img_path
    grid = np.asarray(Image.open(img_path).convert('L'))
    h, w = grid.shape
    extent = [ox, ox + w * res, oy, oy + h * res]

    actual = read_actual_csv(cycle_dir)
    legs = read_planned_csv(cycle_dir)

    fig = Figure(figsize=(13.5, 7.6), dpi=160)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.65, 1.0], wspace=0.08)
    ax = fig.add_subplot(gs[0, 0])
    ax_info = fig.add_subplot(gs[0, 1])
    ax_info.axis('off')

    ax.imshow(grid, cmap='gray', origin='upper', extent=extent, vmin=0, vmax=255, alpha=0.88)

    # Collect bounds
    xs, ys = [], []

    # Plot planned legs
    leg_styles = [
        ('to_A', 'Quỹ đạo tính: Tới A (Nav2)', '#00acc1', '--', 2.2),
        ('pickup_A_to_B', 'Quỹ đạo tính: A->C->B (Lấy pallet)', '#ff9800', '--', 2.4),
        ('to_D', 'Quỹ đạo tính: B->D (Nav2 giao hàng)', '#00bcd4', ':', 2.4),
    ]
    for key, label, color, ls, lw in leg_styles:
        pts = legs.get(key) or []
        if pts:
            px = [p[0] for p in pts]
            py = [p[1] for p in pts]
            xs.extend(px); ys.extend(py)
            ax.plot(px, py, linestyle=ls, color=color, linewidth=lw, label=label, zorder=4)

    # Plot actual trajectory
    if actual:
        ax_x = [p[0] for p in actual]
        ax_y = [p[1] for p in actual]
        xs.extend(ax_x); ys.extend(ax_y)
        ax.plot(ax_x, ax_y, color='#e91e63', linewidth=2.6, label='Quỹ đạo thực tế (Toàn chu trình)', zorder=5)
        ax.scatter([ax_x[0]], [ax_y[0]], color='#4caf50', s=80, zorder=7, edgecolors='black', label='Điểm bắt đầu')
        ax.annotate('Start', (ax_x[0], ax_y[0]), xytext=(6, 6), textcoords='offset points',
                    fontsize=9, fontweight='bold', color='#1b5e20')

    # Plot key saved/measured poses
    def draw_pose(pose, label, color, marker, offset=(8, -12)):
        if not isinstance(pose, (list, tuple)) or len(pose) != 3:
            return
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        xs.append(x); ys.append(y)
        ax.scatter([x], [y], color=color, s=90, marker=marker, edgecolors='black', zorder=8)
        ax.annotate('', xy=(x + 0.22 * math.cos(yaw), y + 0.22 * math.sin(yaw)), xytext=(x, y),
                    arrowprops=dict(arrowstyle='->', color=color, lw=2.0), zorder=8)
        ax.annotate(label, (x, y), xytext=offset, textcoords='offset points',
                    fontsize=9, fontweight='bold', color=color,
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=color, alpha=0.85))

    draw_pose(summary['point_A']['goal_pose_xyyaw'], 'A (lưu)', '#1565c0', 's', (8, 8))
    draw_pose(summary['point_B']['actual_map_xyyaw'] or summary['point_B']['planned_map_xyyaw'],
              'B (lấy)', '#e65100', '^', (8, -14))
    draw_pose(summary['point_D']['goal_pose_xyyaw'], 'D (lưu)', '#6a1b9a', 'D', (8, 8))

    if xs and ys:
        pad = 0.85
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X bản đồ (m)')
    ax.set_ylabel('Y bản đồ (m)')
    ax.set_title(f'Quỹ đạo đầy đủ Chu trình #{summary["cycle_index"]} · Thời gian: {fmt_duration(summary["cycle_duration_s"])}',
                 fontsize=11, fontweight='bold')
    ax.grid(True, linestyle=':', alpha=0.4)
    ax.legend(loc='lower left', fontsize=8, framealpha=0.92)

    # Metrics summary box on right
    a = summary['point_A']
    p = summary['pallet_measurement']
    b = summary['point_B']
    d = summary['point_D']
    info_text = '\n'.join([
        f'CHU TRÌNH #{summary["cycle_index"]}',
        f'Thời gian: {fmt_duration(summary["cycle_duration_s"])}',
        '',
        '── TẠI A (so với A đã lưu) ──',
        f'Lệch vị trí : {a["position_error_cm"]} cm',
        f'Lệch góc    : {a["heading_error_deg"]}°',
        f'Thực tế     : {format_pose(a["arrival_pose_xyyaw"])}',
        '',
        '── ĐO XE SO VỚI PALLET ──',
        f'Camera thô  : {p["camera_lateral_raw_cm"]} cm',
        f'Lệch gốc càng: {p["fork_lateral_corrected_cm"]} cm',
        f'Khoảng cách : {p["fork_root_to_pallet_m"]} m',
        f'Góc pallet  : {p["entry_heading_deg"]}°',
        f'Khe hở C    : {p["c_lateral_clearance_mm"]} mm',
        '',
        '── TẠI B (LẤY PALLET) ──',
        f'Trạng thái  : {b["status"]}',
        f'Lệch vs B   : {b["position_error_vs_planned_cm"]} cm / {b["yaw_error_deg"]}°',
        f'Nâng càng   : {b["lift_height_mm"]} mm',
        '',
        '── TẠI D (so với D đã lưu) ──',
        f'Trạng thái  : {d["result_status"]}',
        f'Lệch vị trí : {d["position_error_cm"]} cm',
        f'Lệch góc    : {d["heading_error_deg"]}°',
        f'Thực tế     : {format_pose(d["arrival_pose_xyyaw"])}',
        f'Hạ càng     : {d["lower_height_mm"]} mm',
    ])
    ax_info.text(0.02, 0.98, info_text, va='top', ha='left', family='monospace', fontsize=8.8,
                 bbox=dict(boxstyle='round,pad=0.6', fc='#f8f9fa', ec='#90a4ae', lw=1.2))

    png_path = cycle_dir / 'full_trajectory_map.png'
    svg_path = cycle_dir / 'full_trajectory_map.svg'
    fig.savefig(png_path, bbox_inches='tight')
    fig.savefig(svg_path, bbox_inches='tight')
    return png_path, svg_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cycle-dir', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--cycle-index', type=int, default=1)
    parser.add_argument('--cycle-start-wall', type=float, default=None)
    parser.add_argument('--cycle-end-wall', type=float, default=None)
    args = parser.parse_args()

    summary = build_summary(args.cycle_dir, args.cycle_index, args.cycle_start_wall, args.cycle_end_wall)
    png_path, svg_path = render_full_trajectory_map(args.cycle_dir, args.map, summary)
    summary['full_trajectory_map_png'] = str(png_path)
    summary['full_trajectory_map_svg'] = str(svg_path)
    (args.cycle_dir / 'cycle_summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    report_txt = format_report_text(summary, png_path)
    (args.cycle_dir / 'cycle_summary.txt').write_text(report_txt)
    print(report_txt, flush=True)


if __name__ == '__main__':
    main()
