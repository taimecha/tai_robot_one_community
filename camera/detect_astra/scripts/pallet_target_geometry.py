"""Full-resolution pallet-leg geometry from registered depth and an RGB mask.

The correction is an empirical preview, not a validated depth calibration.
All distances are relative to the RGB camera lens. No wheel command is issued.
"""

import math

import cv2
import numpy as np


def corrected_depth_m(raw_depth_mm, u, inverse_depth_x_slope, reference_u):
    """Correct measured optical Z at RGB column u, preserving invalid pixels."""
    z = np.asarray(raw_depth_mm, dtype=np.float64) * 0.001
    u = np.asarray(u, dtype=np.float64)
    with np.errstate(divide="ignore", invalid="ignore"):
        inverse = 1.0 / z + inverse_depth_x_slope * (reference_u - u)
        return np.where((z > 0) & (inverse > 0), 1.0 / inverse, np.nan)


def _leg_components(mask, bbox):
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    h, w = mask.shape
    x1, x2 = max(0, x1), min(w, x2)
    y1, y2 = max(0, y1), min(h, y2)
    if x2 - x1 < 50 or y2 - y1 < 40:
        raise ValueError("pallet bounding box too small")
    for fraction in (0.65, 0.70, 0.75, 0.80):
        start = y1 + int((y2 - y1) * fraction)
        crop = np.asarray(mask[start:y2, x1:x2], dtype=np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(crop, 8)
        minimum_area = max(50, int(0.004 * (x2 - x1) * (y2 - y1)))
        minimum_height = max(8, int(0.25 * (y2 - start)))
        candidates = []
        for index in range(1, count):
            cx, cy, cw, ch, area = map(int, stats[index])
            if (area >= minimum_area and cw >= 6 and ch >= minimum_height
                    and cy + ch >= y2 - start - 8):
                candidates.append((index, cx, cy, cw, ch, area))
        candidates.sort(key=lambda item: item[-1], reverse=True)
        if len(candidates) != 2:
            continue
        candidates.sort(key=lambda item: item[1])
        left, right = candidates
        if right[1] - (left[1] + left[3]) < max(12, int(0.15 * (x2 - x1))):
            continue
        result = []
        for index, cx, cy, cw, ch, area in candidates:
            yy, xx = np.nonzero(labels == index)
            result.append(dict(u=xx + x1, v=yy + start, width=cw,
                               top=cy + start, bottom=cy + start + ch - 1,
                               area=area))
        return result
    raise ValueError("two separate front-leg mask components not found")


def _estimate_opening_height(mask, bbox, components, intrinsics, pitch_rad, depth_z_m):
    """Estimate underside-to-foot height from a persistent central mask gap."""
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    center_u = int(np.median((np.median(components[0]['u']), np.median(components[1]['u']))))
    first_row = max(0, y1 + int(0.55 * (y2 - y1)))
    bottom_v = min(components[0]['bottom'], components[1]['bottom'])
    top_v = None
    for row in range(first_row, bottom_v - 10):
        # Requiring persistence prevents gaps in the top deck from becoming
        # the underside of the front opening.
        if all(not mask[y, center_u] for y in range(row, row + 8)):
            top_v = row
            break
    if top_v is None:
        raise ValueError('opening underside is not visible in RGB mask')
    k, distortion = intrinsics
    rays = cv2.undistortPoints(
        np.array([[[center_u, top_v]], [[center_u, bottom_v]]], np.float64),
        k, distortion).reshape(2, 2)
    height = float(depth_z_m * math.cos(pitch_rad) * (rays[1, 1] - rays[0, 1]))
    if not 0.03 <= height <= 0.25:
        raise ValueError('opening height estimate outside pallet geometry')
    return height


def _sample_leg(component, depth_mm, intrinsics, rows, pitch_rad, slope, reference_u,
                min_z_m, max_z_m, min_depth_samples):
    u, v = component["u"], component["v"]
    center_u = float(np.median(u))
    halfwidth = max(3, int(component["width"] * 0.23))
    central = ((u >= center_u - halfwidth) & (u <= center_u + halfwidth)
               & (v >= rows[0]) & (v <= rows[1]))
    u, v = u[central], v[central]
    if len(u) < min_depth_samples:
        raise ValueError("leg interior has too few mask pixels")
    raw = depth_mm[v, u]
    z = corrected_depth_m(raw, u, slope, reference_u)
    valid = np.isfinite(z) & (z >= min_z_m) & (z <= max_z_m)
    if valid.sum() < min_depth_samples or valid.mean() < 0.65:
        raise ValueError("leg interior has insufficient valid depth")
    u, v, z = u[valid], v[valid], z[valid]
    k, distortion = intrinsics
    pixels = np.column_stack((u, v)).astype(np.float64).reshape(-1, 1, 2)
    rays = cv2.undistortPoints(pixels, k, distortion).reshape(-1, 2)
    x = rays[:, 0] * z
    y = rays[:, 1] * z
    forward = z * math.cos(pitch_rad) - y * math.sin(pitch_rad)
    # Median absolute deviation rejects a leg containing several unrelated faces.
    median_forward = float(np.median(forward))
    mad = float(np.median(np.abs(forward - median_forward)))
    if mad > 0.018:
        raise ValueError(f"leg depth is not one stable front surface (MAD {mad:.3f} m)")
    return dict(optical_x_m=float(np.median(x)), forward_m=median_forward,
                z_m=float(np.median(z)), samples=int(len(z)), mad_m=mad)


def measure_pallet_target(mask, bbox, depth_mm, intrinsics, *, pitch_deg=20.0,
                          inverse_depth_x_slope=0.00015807, reference_u=370.0,
                          min_z_m=0.65, max_z_m=3.5,
                          expected_opening_width_m=0.252,
                          opening_width_tolerance_m=0.08,
                          min_depth_samples=60,
                          camera_ahead_of_chassis_m=0.0665):
    """Return experimental front-opening measurements or raise ValueError.

    ``mask`` is one YOLO pallet segmentation mask at the full RGB image size.
    The algorithm requires two separate lower mask components, then samples
    interior depth from both in the same narrow image-row band. The opening midpoint uses
    the two inner leg edges and each leg's measured Z, avoiding depth edges.
    """
    if mask.shape != depth_mm.shape or depth_mm.ndim != 2:
        raise ValueError("RGB mask and registered depth sizes differ")
    if not np.isfinite([pitch_deg, inverse_depth_x_slope, reference_u]).all():
        raise ValueError("nonfinite geometry parameter")
    left, right = _leg_components(mask, bbox)
    common_top = max(left["top"], right["top"])
    common_bottom = min(left["bottom"], right["bottom"])
    if common_bottom - common_top < 12:
        raise ValueError("front legs lack common sample rows")
    # The same narrow row band samples the two vertical front legs at nearly
    # the same height. A wide band can bias their comparison when the mask or
    # measured depth is uneven along the legs.
    span = common_bottom - common_top
    center_row = (common_top + common_bottom) // 2
    half_band = max(4, min(6, int(round(span * 0.16))))
    rows = (max(common_top, center_row - half_band),
            min(common_bottom, center_row + half_band))
    pitch_rad = math.radians(pitch_deg)
    samples = [
        _sample_leg(component, depth_mm, intrinsics, rows, pitch_rad,
                    inverse_depth_x_slope, reference_u, min_z_m, max_z_m,
                    min_depth_samples)
        for component in (left, right)
    ]
    left_sample, right_sample = samples
    separation = right_sample["optical_x_m"] - left_sample["optical_x_m"]
    if not 0.15 <= separation <= 0.60:
        raise ValueError("leg-center separation outside pallet geometry")
    face_yaw = math.atan2(left_sample["forward_m"] - right_sample["forward_m"],
                          separation)
    if abs(face_yaw) > math.radians(20):
        raise ValueError("front-face yaw outside measurement range")

    # Inner edges are obtained from the mask; use median leg depth instead of
    # boundary depth, which is often mixed with the background by registration.
    edge_us = []
    for component, inner in ((left, max), (right, min)):
        edge_by_row = []
        for row in range(rows[0], rows[1] + 1):
            x_at_row = component["u"][component["v"] == row]
            if len(x_at_row):
                edge_by_row.append(inner(x_at_row))
        if len(edge_by_row) < 8:
            raise ValueError("inner leg edge is incomplete")
        edge_us.append(float(np.median(edge_by_row)))
    edge_pixels = np.array([[edge_us[0], np.mean(rows)],
                            [edge_us[1], np.mean(rows)]], dtype=np.float64).reshape(-1, 1, 2)
    k, distortion = intrinsics
    edge_rays = cv2.undistortPoints(edge_pixels, k, distortion).reshape(-1, 2)
    edge_x = np.array([edge_rays[0, 0] * left_sample["z_m"],
                       edge_rays[1, 0] * right_sample["z_m"]])
    opening_width = float(edge_x[1] - edge_x[0])
    if abs(opening_width - expected_opening_width_m) > opening_width_tolerance_m:
        raise ValueError(f"opening width {opening_width:.3f} m outside expected range")
    camera_forward = (left_sample["forward_m"] + right_sample["forward_m"]) / 2
    try:
        height = _estimate_opening_height(
            mask, bbox, (left, right), intrinsics, pitch_rad,
            (left_sample['z_m'] + right_sample['z_m']) / 2)
    except ValueError:
        height = math.nan  # Horizontal leg measurements remain usable for inspection.
    return dict(camera_forward_m=camera_forward,
                chassis_front_forward_m=camera_forward + camera_ahead_of_chassis_m,
                opening_lateral_m=-float(np.mean(edge_x)),
                face_yaw_rad=face_yaw,
                entry_heading_rad=-face_yaw,
                opening_width_m=opening_width,
                opening_height_estimate_m=height,
                leg_center_separation_m=separation,
                left_forward_m=left_sample["forward_m"],
                right_forward_m=right_sample["forward_m"],
                left_depth_samples=left_sample["samples"],
                right_depth_samples=right_sample["samples"],
                sample_row_v=center_row,
                left_depth_mad_m=left_sample["mad_m"],
                right_depth_mad_m=right_sample["mad_m"])
