#!/usr/bin/env python3
"""Experimental YOLO11 segmentation + registered Astra depth; never AGV control.

Source /opt/ros/jazzy/setup.bash, then use a Python with ROS and ultralytics.
Q quits; S saves the displayed pair, original data, masks, and measurements.
Distance is median optical Z inside the eroded mask, NOT Euclidean range.
XYZ uses the actual depth of a marked valid pixel, NOT the mask median.
"""
import argparse
from datetime import datetime, timezone
import importlib
import json
import os
from pathlib import Path
import sys
import threading
import time

# Report all dependencies in the selected interpreter, without installing anything.
_missing = []
for _name in ('rclpy', 'sensor_msgs', 'cv2', 'numpy', 'torch', 'ultralytics'):
    try:
        globals()[_name] = importlib.import_module(_name)
    except Exception as _exc:
        _missing.append(f'{_name}: {type(_exc).__name__}: {_exc}')
if _missing:
    raise SystemExit(f'Python: {sys.executable}\n' + '\n'.join(_missing) +
                     '\nSource /opt/ros/jazzy/setup.bash and use your existing YOLO Python.'
                     '\nNo packages have been installed or upgraded.')

import cv2
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import remove_ros_args, get_rmw_implementation_identifier
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from ultralytics import YOLO

WINDOW = 'Astra Pro YOLO Depth'
SHAPE = (480, 640)
EXPERIMENTAL = 'XYZ ESTIMATED - CALIBRATION UNVERIFIED - NOT FOR AGV CONTROL'
OPTICAL_FRAME = 'camera_color_optical_frame'


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def decode_image(msg, depth=False):
    """Decode ROS buffers with row padding and endianness; no cv_bridge ABI dependency."""
    channels = 1 if depth else {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4,
                               'mono8': 1}.get(msg.encoding)
    if channels is None or (depth and msg.encoding != '16UC1'):
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    dtype = np.dtype('>u2' if msg.is_bigendian else '<u2') if depth else np.dtype('u1')
    pixel_bytes = channels*dtype.itemsize
    if msg.step < msg.width*pixel_bytes or len(msg.data) < msg.height*msg.step:
        raise ValueError('Malformed image buffer/step')
    shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
    strides = (msg.step, pixel_bytes) if channels == 1 else (msg.step, pixel_bytes, 1)
    image = np.ndarray(shape, dtype=dtype, buffer=msg.data, strides=strides)
    if depth:
        return image.astype(np.uint16, copy=True)
    conversions = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                   'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
    return cv2.cvtColor(image, conversions[msg.encoding]) if msg.encoding in conversions else image.copy()


def image_metadata(msg):
    return dict(stamp_ns=stamp_ns(msg), frame_id=msg.header.frame_id,
                width=msg.width, height=msg.height, encoding=msg.encoding)


def info_metadata(info):
    if info is None:
        return None
    return dict(stamp_ns=stamp_ns(info), frame_id=info.header.frame_id,
                width=info.width, height=info.height,
                k=[float(v) if np.isfinite(v) else None for v in info.k],
                d=[float(v) if np.isfinite(v) else None for v in info.d],
                p=[float(v) if np.isfinite(v) else None for v in info.p],
                r=[float(v) if np.isfinite(v) else None for v in info.r], distortion_model=info.distortion_model,
                binning_x=info.binning_x, binning_y=info.binning_y,
                roi=dict(x=info.roi.x_offset, y=info.roi.y_offset,
                         width=info.roi.width, height=info.roi.height,
                         do_rectify=info.roi.do_rectify))


def validate_intrinsics(rgb, depth, color_info, registration):
    """Use raw RGB K/D only after checking live registration configuration.

    Depth CameraInfo is diagnostic only: invalid K is never used or repaired.
    Parameter confirmation and matching geometry do not establish calibration.
    """
    if (not registration.get('verified', False)
            or registration.get('parameter') != 'depth_registration'
            or registration.get('value') is not True
            or not np.isfinite(registration.get('check_age_ms', float('inf')))
            or not 0 <= registration.get('check_age_ms', float('inf')) <= 3000):
        return None, 'XYZ unavailable: ' + registration.get('reason', 'registration unconfirmed')
    if color_info is None:
        return None, 'XYZ unavailable: missing RGB CameraInfo'
    if any(m.header.frame_id != OPTICAL_FRAME for m in (rgb, depth, color_info)):
        return None, 'XYZ unavailable: frame_id does not match color optical frame'
    if (rgb.height, rgb.width) != SHAPE or (depth.height, depth.width) != SHAPE:
        return None, 'XYZ unavailable: RGB/Depth resolution mismatch'
    info = color_info
    if (info.width, info.height) != (rgb.width, rgb.height):
        return None, 'XYZ unavailable: RGB CameraInfo resolution mismatch'
    duplicate_profile = len(registration.get('source_diagnostics', {}).get('endpoints', {}).get('rgb', [])) == 2
    if duplicate_profile and (not stamp_ns(info) or abs(stamp_ns(info)-stamp_ns(rgb)) > 50_000_000):
        return None, 'XYZ unavailable: Astra UVC RGB CameraInfo must match image within 50 ms'
    if stamp_ns(info) and abs(stamp_ns(info) - stamp_ns(rgb)) > 1_000_000_000:
        return None, 'XYZ unavailable: RGB CameraInfo timestamp too far from image'
    if info.binning_x not in (0, 1) or info.binning_y not in (0, 1):
        return None, 'XYZ unavailable: unsupported RGB binning'
    roi = info.roi
    if roi.x_offset or roi.y_offset or roi.do_rectify or (roi.width, roi.height) not in ((0, 0), (640, 480)):
        return None, 'XYZ unavailable: unsupported RGB CameraInfo ROI'
    k = np.asarray(info.k).reshape(3, 3)
    if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1])
            or k[0, 1] != 0 or k[1, 0] != 0
            or not 0 <= k[0, 2] < rgb.width or not 0 <= k[1, 2] < rgb.height):
        return None, 'XYZ unavailable: invalid/uncalibrated RGB K'
    if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
        return None, 'XYZ unavailable: unsupported RGB distortion model'
    if len(info.d) not in (4, 5, 8, 12, 14) or not np.isfinite(info.d).all():
        return None, 'XYZ unavailable: invalid RGB distortion coefficients'
    return (k, np.asarray(info.d)), 'XYZ estimated in ' + OPTICAL_FRAME + ' (RGB K/D)'


def measure_mask(mask, depth, args, intrinsics):
    """Return statistics, candidate mask, valid sampling mask; never mutate inputs."""
    radius = args.erode_px
    candidate = cv2.erode(mask.astype(np.uint8), np.ones((2*radius+1, 2*radius+1), np.uint8),
                          borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    used = candidate & (depth > 0) & (depth >= args.min_depth_mm) & (depth <= args.max_depth_mm)
    total, count = int(candidate.sum()), int(used.sum())
    ratio = count / total if total else 0.0
    record = dict(candidate_pixels=total, valid_pixels=count, valid_ratio=ratio,
                  median_z_m=None, pixel_uv=None, pixel_z_m=None, xyz_m=None,
                  xyz_frame_id=None)
    if count < args.min_valid_pixels or ratio < args.min_valid_ratio:
        record['status'] = 'Invalid depth'
        return record, candidate, used
    median = float(np.median(depth[used]))
    ys, xs = np.nonzero(used)
    # Select a real measured pixel nearest the median depth; break ties by
    # proximity to the sampling region's center. No synthetic centroid depth.
    center_x, center_y = xs.mean(), ys.mean()
    order = np.lexsort(((xs-center_x)**2 + (ys-center_y)**2,
                        np.abs(depth[ys, xs].astype(np.float64) - median)))
    idx = order[0]
    u, v = int(xs[idx]), int(ys[idx])
    z = float(depth[v, u]) / 1000
    record.update(status='Experimental depth', median_z_m=median/1000,
                  pixel_uv=[u, v], pixel_z_m=z)
    if intrinsics is not None:
        k, distortion = intrinsics
        ray = cv2.undistortPoints(np.array([[[u, v]]], np.float64), k, distortion)[0, 0]
        if np.isfinite(ray).all():
            record['xyz_m'] = [float(ray[0]*z), float(ray[1]*z), z]
            record['xyz_frame_id'] = OPTICAL_FRAME
    return record, candidate, used

def pixel_points(uv, z, intrinsics):
    k, distortion = intrinsics
    rays = cv2.undistortPoints(np.asarray(uv, np.float64).reshape(-1, 1, 2), k, distortion).reshape(-1, 2)
    return np.column_stack((rays*z[:, None], z))


def fit_edge_plane(uv, z, intrinsics):
    """Robust local 3D plane, never a claim of pallet semantic identity."""
    points = pixel_points(uv, z, intrinsics)
    if len(points) < 40 or not np.isfinite(points).all():
        return None
    rng = np.random.default_rng(17)
    best = None
    for _ in range(40):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        offset = -normal @ a
        support = np.abs(points @ normal + offset) < 0.015
        if best is None or support.sum() > best.sum():
            best = support
    if best is None or best.mean() < .75:
        return None
    center = points[best].mean(axis=0)
    _, singular, vt = np.linalg.svd(points[best]-center, full_matrices=False)
    if singular[1] / np.sqrt(best.sum()) < .003:
        return None  # Insufficient surface extent: points nearly collinear.
    normal = vt[-1]
    offset = float(-normal @ center)
    residual = np.abs(points @ normal + offset)
    inliers = residual < .015
    rms = float(np.sqrt(np.mean(residual[inliers]**2)))
    # Camera-facing surface only; no gravity reference, so not proven vertical.
    if inliers.mean() < .75 or rms > .010 or abs(normal[2]) < .75:
        return None
    return normal, offset, float(inliers.mean()), rms, float(np.median(z[inliers]))


def continuous_lower_edge(options, reference, bbox):
    """Fallback only to a longer lower boundary of the same supported surface."""
    if reference.get('coverage', 1.) >= .65 and reference.get('continuity', 1.) >= .9:
        return None
    ab = reference['ab']
    n, d = reference['plane'][:2]
    matches = []
    for option in options:
        other = option['ab']
        dy = float(other[:, 1].mean()-ab[:, 1].mean())
        normal, offset = option['plane'][:2]
        sign = 1 if np.dot(n, normal) >= 0 else -1
        overlap = max(0., min(ab[1, 0], other[1, 0])-max(ab[0, 0], other[0, 0]))
        if (4 <= dy <= min(60., .45*(bbox[3]-bbox[1]))
                and option.get('coverage', 0) >= .65
                and option.get('continuity', 0) >= .9
                and option.get('max_gap', 1) <= .08
                and np.linalg.norm(other[1]-other[0]) >= 1.4*np.linalg.norm(ab[1]-ab[0])
                and overlap >= .7*(ab[1, 0]-ab[0, 0])
                and abs(np.dot(n, normal)) >= .97
                and abs(d-sign*offset) <= .025
                and abs(reference['z']-option['z']) <= .06):
            matches.append(option)
    return max(matches, key=lambda o: o['score']) if matches else None


class FrontEdgeTracker:
    """Experimental RGB+plane-supported candidates, not semantic front-edge detection."""
    def __init__(self):
        self.tracks = {}
        self.next_id = 0

    def select(self, rgb, depth, mask, used, intrinsics, bbox, cls, stamp):
        result = dict(status='FRONT EDGE NOT FOUND', semantic_front_verified=False,
                      confidence_kind='heuristic support score, not probability',
                      A_uv=None, B_uv=None, M_uv=None, sample_uv=None, xyz_m=None,
                      M_xyz_m=None, confidence=0.0, reason='no qualified RGB/Depth edge')
        if intrinsics is None:
            result['reason'] = 'registered RGB intrinsics unavailable'
            return result
        self.tracks = {k: v for k, v in self.tracks.items() if 0 <= stamp-v['stamp'] < 2.0}
        def iou(a, b):
            wh = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2]))
            inter = float(np.prod(wh))
            return inter/max(1., float(np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2]))-inter)
        matches = [(iou(bbox, t['bbox']), k) for k, t in self.tracks.items()
                   if t['cls'] == cls and t['stamp'] != stamp]
        matches.sort(reverse=True)
        if len(matches) > 1 and matches[1][0] > .5:
            result['reason'] = 'ambiguous instance association'
            return result
        track_id = matches[0][1] if matches and matches[0][0] > .5 else None
        previous = self.tracks.get(track_id)
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 120)
        edges[~mask] = 0
        minimum = max(35, int((bbox[2]-bbox[0])*.35))
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=25,
                                minLineLength=minimum, maxLineGap=8)
        options = []
        diagnostics = dict(lines=0, mask_supported=0, plane_supported=0, depth_boundaries=0)
        result['diagnostics'] = diagnostics
        # Re-evaluate the old segment using current RGB and Depth, never old XYZ.
        lines = [] if lines is None else list(np.asarray(lines).reshape(-1, 4))
        if previous is not None:
            lines.insert(0, previous['ab'].reshape(4))
        edge_support = cv2.dilate(edges, np.ones((5, 5), np.uint8))
        if len(lines):
            for line in sorted(np.asarray(lines).reshape(-1, 4), key=lambda l: -np.hypot(l[2]-l[0], l[3]-l[1]))[:40]:
                diagnostics['lines'] += 1
                a, b = line[:2].astype(float), line[2:].astype(float)
                if a[0] > b[0]:
                    a, b = b, a
                direction = b-a
                length = np.linalg.norm(direction)
                if abs(direction[1]) > .27*abs(direction[0]):
                    continue
                tangent = direction/length
                perpendicular = np.array([-tangent[1], tangent[0]])
                along = a + np.linspace(.05, .95, 60)[:, None]*direction
                samples = np.rint(along).astype(int)
                if mask[samples[:, 1], samples[:, 0]].mean() < .9:
                    continue
                supported = edge_support[samples[:, 1], samples[:, 0]] > 0
                continuity = float(supported.mean())
                longest_gap = run = 0
                for present in supported:
                    run = 0 if present else run+1
                    longest_gap = max(longest_gap, run)
                if continuity < .6:
                    continue
                diagnostics['mask_supported'] += 1
                for side in (-1, 1):
                    band = np.rint(along[:, None, :] + side*perpendicular*np.arange(2, 14, 2)[None, :, None]).astype(int).reshape(-1, 2)
                    band = band[(band[:, 0]>=0)&(band[:, 0]<640)&(band[:, 1]>=0)&(band[:, 1]<480)]
                    valid = used[band[:, 1], band[:, 0]]
                    if valid.sum() < 60 or valid.mean() < .55:
                        continue
                    uv = band[valid]
                    z = depth[uv[:, 1], uv[:, 0]].astype(float)/1000
                    plane = fit_edge_plane(uv, z, intrinsics)
                    if plane is None:
                        continue
                    diagnostics['plane_supported'] += 1
                    normal, offset, support, rms, plane_z = plane
                    opposite = np.rint(along - side*perpendicular*8).astype(int)
                    ok = (opposite[:, 0]>=0)&(opposite[:, 0]<640)&(opposite[:, 1]>=0)&(opposite[:, 1]<480)
                    opposite = opposite[ok]
                    valid_other = depth[opposite[:, 1], opposite[:, 0]] > 0
                    # Require measured depth discontinuity, not just a texture edge or holes.
                    if valid_other.sum() < 20:
                        continue
                    other = opposite[valid_other]
                    pts = pixel_points(other, depth[other[:, 1], other[:, 0]]/1000., intrinsics)
                    discontinuity = float((np.abs(pts@normal+offset) > .04).mean())
                    if discontinuity < .6:
                        continue
                    diagnostics['depth_boundaries'] += 1
                    coverage = min(1., length/max(1., bbox[2]-bbox[0]))
                    score = float(support*valid.mean()*discontinuity*coverage)
                    option = dict(ab=np.array([a, b]), plane=plane, side=side,
                                  perpendicular=perpendicular, score=score, z=plane_z,
                                  coverage=coverage, continuity=continuity, max_gap=longest_gap/len(supported))
                    if not any(np.max(np.linalg.norm(option['ab']-o['ab'], axis=1)) < 10 for o in options):
                        options.append(option)
        if not options:
            # Keep identity briefly across dropped frames, but never reuse old XYZ.
            result['reason'] = 'no current edge with sufficient plane/depth support'
            return result
        # Prefer the nearest supported surface, not the smallest raw depth pixel.
        options.sort(key=lambda o: (o['z'], -o['score']))
        close = [o for o in options if abs(o['z']-options[0]['z']) < .06]
        close.sort(key=lambda o: -o['score'])
        chosen = close[0]
        fallback_selected = False
        pending = None
        if previous is not None:
            consistent = [o for o in close if np.max(np.linalg.norm(o['ab']-previous['ab'], axis=1)) < 15
                          and abs(o['z']-previous['z']) < .06]
            reference = min(consistent, key=lambda o: np.linalg.norm(o['ab']-previous['ab'])) if consistent else previous
            lower = continuous_lower_edge(close, reference, bbox)
            if lower is not None:
                old_pending = previous.get('lower_pending')
                same = old_pending is not None and np.max(np.linalg.norm(lower['ab']-old_pending['ab'], axis=1)) < 8
                pending = dict(ab=lower['ab'].copy(), hits=old_pending['hits']+1 if same else 1)
                if pending['hits'] >= 3:
                    chosen = lower
                    previous = None  # Never interpolate between two different edges.
                    fallback_selected = True
                    pending = None
                else:
                    previous['lower_pending'] = pending
            else:
                previous.pop('lower_pending', None)
            if not fallback_selected:
                if consistent:
                    chosen = reference
                else:
                    result['reason'] = 'waiting for supported tracked edge or confirmed continuous lower edge'
                    return result
        if previous is None and not fallback_selected and len(close) > 1 and close[0]['score'] < 1.3*close[1]['score']:
            result['reason'] = 'multiple equally near supported edges; no dominant candidate'
            return result
        if track_id is None:
            track_id = self.next_id
            self.next_id += 1
        hits = previous['hits']+1 if previous is not None else (3 if fallback_selected else 1)
        # EMA only for a geometrically consistent segment; measurement uses current depth.
        ab = chosen['ab']
        if previous is not None:
            shift = np.max(np.linalg.norm(chosen['ab']-previous['ab'], axis=1))
            ab = previous['ab'].copy() if shift <= 3 else .8*previous['ab'] + .2*chosen['ab']
        self.tracks[track_id] = dict(ab=ab, z=chosen['z'], hits=hits, cls=cls,
                                     bbox=bbox.copy(), stamp=stamp, plane=chosen['plane'],
                                     coverage=chosen['coverage'], continuity=chosen['continuity'],
                                     lower_pending=pending,
                                     selection='continuous lower edge' if fallback_selected else (previous.get('selection', 'automatic edge') if previous else 'automatic edge'))
        midpoint = ab.mean(axis=0)
        normal, offset, support, rms, _ = chosen['plane']
        result.update(A_uv=ab[0].tolist(), B_uv=ab[1].tolist(), M_uv=midpoint.tolist(),
                      raw_AB_uv=chosen['ab'].tolist(), confidence=chosen['score'],
                      selection=self.tracks[track_id]['selection'],
                      edge_coverage=chosen['coverage'], edge_continuity=chosen['continuity'],
                      stable_frames=hits, track_id=track_id,
                      plane_normal=normal.tolist(), plane_offset_m=offset,
                      plane_inlier_ratio=support, plane_rms_m=rms,
                      plane_orientation='camera-facing; gravity/vertical not verified')
        if hits < 3:
            result['reason'] = 'candidate pending 3 consistent frames'
            return result
        result.update(status='FRONT EDGE CANDIDATE - UNVERIFIED', reason='semantic identity not verified')
        # Search <=8 pixels from M, on the selected side and measured plane only.
        u0, v0 = np.rint(midpoint).astype(int)
        yy, xx = np.mgrid[max(0,v0-8):min(480,v0+9), max(0,u0-8):min(640,u0+9)]
        uv = np.column_stack((xx.ravel(), yy.ravel()))
        uv = uv[used[uv[:, 1], uv[:, 0]]]
        if len(uv):
            delta = uv-midpoint
            uv = uv[(np.linalg.norm(delta, axis=1)<=8) & (delta@chosen['perpendicular']*chosen['side']>=0)]
        if len(uv):
            pts = pixel_points(uv, depth[uv[:, 1], uv[:, 0]]/1000., intrinsics)
            on_plane = np.abs(pts@normal+offset) < .015
            uv, pts = uv[on_plane], pts[on_plane]
            if len(uv):
                distances = np.linalg.norm(uv-midpoint, axis=1)
                last_sample = previous.get('sample_uv') if previous else None
                if last_sample is not None:
                    same = np.flatnonzero(np.all(uv == last_sample, axis=1))
                    idx = int(same[0]) if len(same) else int(np.argmin(distances))
                else:
                    idx = int(np.argmin(distances))
                self.tracks[track_id]['sample_uv'] = uv[idx].tolist()
                result.update(sample_uv=uv[idx].tolist(), xyz_m=pts[idx].tolist(),
                              sample_z_m=float(pts[idx, 2]),
                              xyz_target='actual sampled pixel near M; NOT interpolated M')
        if result['xyz_m'] is None:
            result['reason'] = 'no valid on-plane pixel near M; no plane interpolation'
        return result

def publisher_policy(endpoints, observed, parameters, rmw, node_instances):
    """Narrow exception for the locally audited Astra UVC implementation.

    ob_camera_node.cpp: setupDevices creates Color but startStreams skips its
    start() when use_uvc_camera is true (lines 255-258). pollFrame calls
    onNewFrameCallback only after successful waitForAnyStream/readFrame.
    UVC startStreaming is invoked separately (lines 276-279). Therefore this
    profile permits an unstarted OpenNI writer, NOT two interchangeable streams.
    GID prefix checking below is restricted to Fast DDS, not portable to all RMWs.
    """
    rgb, depth, info = (endpoints.get(k, []) for k in ('rgb', 'depth', 'rgb_info'))
    writers = rgb + depth + info
    nodes = {e['node'] for e in writers}
    if len(depth) != 1 or len(nodes) != 1:
        return False, 'Depth writer missing/ambiguous or publisher nodes differ'
    if any(len(set(observed.get(k, {})) - {'UNAVAILABLE'}) > 1
           for k in ('rgb', 'depth', 'rgb_info')):
        return False, 'multiple actual message GIDs observed; restart after source investigation'
    if len(rgb) == 1 and len(info) == 1:
        return True, 'single RGB/Depth/CameraInfo writer'
    if (len(rgb), len(info)) != (2, 2) or nodes != {'/camera/camera'}:
        return False, 'not the audited Astra topology (RGB=2, Depth=1, RGB CameraInfo=2)'
    if node_instances != 1:
        return False, 'camera node name missing or duplicated'
    if rmw not in ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        return False, 'Astra duplicate-writer profile requires verifiable Fast DDS participant'
    gids = [e['gid'] for e in writers]
    if (any(len(g) < 32 or not any(bytes.fromhex(g)) for g in gids)
            or len(set(gids)) != len(gids) or len({g[:24] for g in gids}) != 1):
        return False, 'image/CameraInfo endpoints are not distinct writers of one DDS participant'
    if not all(parameters.get(k) is True for k in
               ('depth_registration', 'enable_color', 'use_uvc_camera')):
        return False, 'Astra UVC profile requires registration, color and UVC all true'
    return True, 'audited Astra UVC profile: OpenNI Color created but not started'


class CameraReceiver(Node):
    def __init__(self, args, node_name='yolo_depth'):
        super().__init__(node_name)
        self.args, self.lock = args, threading.Lock()
        self.latest, self.infos = {}, {}
        self.source_observations = {}
        self.last_source_gid = {}
        self.source_diagnostics = {}
        self.last_source_log = 0.0
        self.publisher_gate = False
        self.publisher_reason = 'sources not inspected'
        self.registration = dict(verified=False, reason='registration not checked')
        self.registration_checked = 0.0
        self.registration_client = None
        self.registration_node = None
        self.registration_future = None
        self.registration_requested = 0.0
        self.status = 'Waiting for RGB / Depth'
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subs = []
        for stream, topic in (('rgb', '/camera/color/image_raw'), ('depth', '/camera/depth/image_raw')):
            self.subs.append(self.create_subscription(Image, topic,
                             self.source_callback(stream, False), qos))
        for stream, topic in (('rgb', '/camera/color/camera_info'), ('depth', '/camera/depth/camera_info')):
            self.subs.append(self.create_subscription(CameraInfo, topic,
                             self.source_callback(stream, True), qos))

        self.registration_timer = self.create_timer(1.0, self.check_registration)

    def source_callback(self, stream, camera_info):
        # Exactly two callback arguments requests ROS MessageInfo (publisher GID).
        key = stream + ('_info' if camera_info else '')
        def callback(msg, message_info):
            raw_gid = (message_info.get('publisher_gid', []) if isinstance(message_info, dict)
                       else getattr(message_info, 'publisher_gid', []))
            gid = bytes(raw_gid).hex() if raw_gid and any(raw_gid) else 'UNAVAILABLE'
            # Some Jazzy rclpy builds expose timestamps but not publisher_gid.
            # An unavailable GID groups messages for diagnostics, not one proven writer.
            now = time.monotonic()
            with self.lock:
                self.last_source_gid[key] = gid
                observations = self.source_observations.setdefault(key, {})
                entry = observations.setdefault(gid, dict(count=0, first_seen=now))
                entry.update(count=entry['count']+1, last_seen=now,
                             frame_id=msg.header.frame_id, width=msg.width, height=msg.height,
                             stamp_ns=stamp_ns(msg))
                # A newly observed second writer invalidates an earlier check immediately.
                if len(observations) > 1 and key in ('rgb', 'depth', 'rgb_info'):
                    self.registration.update(verified=False, reason=f'multiple observed {key} GIDs')
            if camera_info:
                self.receive_info(stream, msg)
            else:
                self.receive(stream, msg)
        return callback

    def inspect_sources(self, rgb, depth):
        """Endpoint names/GID prefixes are evidence, never proof of OS process identity.

        A dormant writer is not equivalent to the active UVC writer.
        Accept only the explicitly audited startup/configuration profile below.
        """
        def describe(endpoint):
            return dict(node=endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name,
                        gid=bytes(endpoint.endpoint_gid).hex())
        endpoints = dict(rgb=[describe(e) for e in rgb], depth=[describe(e) for e in depth],
                         rgb_info=[describe(e) for e in self.get_publishers_info_by_topic(
                             '/camera/color/camera_info')])
        now = time.monotonic()
        with self.lock:
            observed = {stream: {gid: {**record, 'age_ms': (now-record['last_seen'])*1000}
                                 for gid, record in entries.items()}
                        for stream, entries in self.source_observations.items()}
        nodes = {e['node'] for entries in endpoints.values() for e in entries}
        self.source_diagnostics = dict(endpoints=endpoints, observed=observed,
                                      driver_parameters=self.source_diagnostics.get('driver_parameters', {}),
                                      same_named_node=len(nodes) == 1,
                                      process_identity='not exposed by ROS graph; not inferred from GID',
                                      message_gid_available=bool(observed.get('rgb')) and 'UNAVAILABLE' not in observed.get('rgb', {}),
                                      duplicate_policy='XYZ locked; silence does not establish source equivalence')
        self.source_diagnostics['rmw'] = get_rmw_implementation_identifier()
        self.source_diagnostics['camera_node_instances'] = sum(
            name == 'camera' and namespace == '/camera'
            for name, namespace in self.get_node_names_and_namespaces())
        self.update_publisher_policy()
        if now-self.last_source_log >= 5:
            self.get_logger().info('Source diagnostics: ' + json.dumps(self.source_diagnostics))
            self.last_source_log = now
        return nodes

    def update_publisher_policy(self):
        d = self.source_diagnostics
        self.publisher_gate, self.publisher_reason = publisher_policy(
            d['endpoints'], d['observed'], d.get('driver_parameters', {}),
            d['rmw'], d['camera_node_instances'])
        d['publisher_policy'] = self.publisher_reason
        d['duplicate_policy'] = 'audited Astra UVC topology only; not arbitrary multiple writers'

    def set_registration(self, verified, reason):
        with self.lock:
            self.registration = dict(verified=verified, reason=reason,
                                     node=self.registration_node,
                                     parameter='depth_registration',
                                     value=True if verified else None,
                                     checked_at_utc=datetime.now(timezone.utc).isoformat(),
                                     source_diagnostics=self.source_diagnostics)
            self.registration_checked = time.monotonic()

    def check_registration(self):
        """Read-only parameter check on the actual image publisher, never set it."""
        try:
            rgb = self.get_publishers_info_by_topic('/camera/color/image_raw')
            depth = self.get_publishers_info_by_topic('/camera/depth/image_raw')
            nodes = self.inspect_sources(rgb, depth)
            # Read parameters even for the duplicate endpoint case, for diagnosis.
            if not rgb or len(depth) != 1 or len(nodes) != 1:
                self.set_registration(False, self.publisher_reason)
                return
            source = next(iter(nodes))
            if not self.publisher_gate:
                self.set_registration(False, self.publisher_reason)
            if source != self.registration_node:
                if self.registration_client is not None:
                    self.destroy_client(self.registration_client)
                self.registration_node = source
                self.registration_client = self.create_client(GetParameters, source + '/get_parameters')
                self.registration_future = None
                self.set_registration(False, 'waiting for camera parameter service')
            future = self.registration_future
            if future is not None:
                if future.done():
                    response = future.result()
                    values = response.values
                    names = ['depth_registration', 'enable_color', 'use_uvc_camera']
                    parameters = {name: (value.bool_value if value.type == ParameterType.PARAMETER_BOOL
                                         else None) for name, value in zip(names, values)}
                    self.source_diagnostics['driver_parameters'] = parameters
                    self.update_publisher_policy()
                    valid = (parameters.get('depth_registration') is True and self.publisher_gate)
                    reason = ('depth_registration=true; ' + self.publisher_reason if valid else
                              self.publisher_reason if not self.publisher_gate else
                              'depth_registration is false, missing or not boolean')
                    self.set_registration(bool(valid), reason)
                    self.registration_future = None
                elif time.monotonic() - self.registration_requested > 2.0:
                    self.registration_client.remove_pending_request(future)
                    future.cancel()
                    self.registration_future = None
                    self.set_registration(False, 'registration parameter request timed out')
                else:
                    return
            if not self.registration_client.service_is_ready():
                self.set_registration(False, 'camera parameter service unavailable')
                return
            request = GetParameters.Request()
            request.names = ['depth_registration', 'enable_color', 'use_uvc_camera']
            self.registration_future = self.registration_client.call_async(request)
            self.registration_requested = time.monotonic()
        except Exception as exc:
            self.set_registration(False, f'registration check failed: {exc}')
            self.registration_future = None

    def receive(self, stream, msg):
        with self.lock:
            self.latest[stream] = (msg, time.monotonic())

    def receive_info(self, stream, msg):
        with self.lock:
            self.infos[stream] = msg

    def take_pair(self):
        with self.lock:
            now = time.monotonic()
            self.latest = {s: value for s, value in self.latest.items()
                           if now-value[1] <= self.args.max_age_ms/1000}
            if len(self.latest) != 2:
                self.status = 'Waiting for fresh RGB / Depth (old frames discarded)'
                return None
            rgb, depth = self.latest['rgb'][0], self.latest['depth'][0]
            if stamp_ns(rgb) == 0 or stamp_ns(depth) == 0:
                self.latest.clear()
                self.status = 'Rejected: zero image timestamp'
                return None
            dt = (stamp_ns(depth)-stamp_ns(rgb))/1e6
            if abs(dt) > self.args.max_delta_ms:
                del self.latest['rgb' if dt > 0 else 'depth']
                self.status = f'Rejected timestamp dt={dt:+.2f} ms > {self.args.max_delta_ms:g} ms'
                return None
            self.latest.clear()
            self.status = f'Accepted dt={dt:+.2f} ms'
            registration = dict(self.registration)
            registration['source_diagnostics'] = dict(self.source_diagnostics)
            registration['pair_publisher_gids'] = dict(self.last_source_gid)
            ci = self.infos.get('rgb')
            registration['rgb_camera_info_delta_ms'] = (
                (stamp_ns(ci)-stamp_ns(rgb))/1e6 if ci is not None else None)
            registration['check_age_ms'] = (now-self.registration_checked)*1000
            if registration['check_age_ms'] > 3000:
                registration.update(verified=False, reason='registration confirmation expired')
            return rgb, depth, dt, self.infos.get('rgb'), self.infos.get('depth'), registration


def put(image, text, x, y, color=(255, 255, 255), scale=0.48):
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


class Processor:
    def __init__(self, args, model):
        self.args, self.model = args, model
        self.front_edges = FrontEdgeTracker()
        self.manual_ab = []
        self.manual_bbox = None
        self.manual_invalid = False
        self.show_old_point = False
        self.rgb_view = np.zeros((*SHAPE, 3), np.uint8)
        self.depth_view = self.rgb_view.copy()
        self.snapshot = None
        self.summary = 'No accepted pair displayed'
        self.xyz_status = 'XYZ unavailable'
        self.ratios = 'No masks'
        self.error = ''

    def mouse(self, event, x, y, flags, userdata):
        # OpenCV maps mouse coordinates to the full displayed canvas.
        if not getattr(self.args, 'manual_edge', False):
            return
        if event != cv2.EVENT_LBUTTONDOWN or not (0 <= x < 640 and 32 <= y < 512):
            return
        if len(self.manual_ab) == 2:
            self.manual_ab = []
        self.manual_ab.append([x, y-32])
        self.manual_bbox = None
        self.manual_invalid = False
        print('Manual edge: click A then B on RGB. R returns to automatic candidates.', self.manual_ab)

    def manual_measure(self, depth, mask, used, intrinsics, bbox):
        result = dict(status='MANUAL EDGE - UNVERIFIED', reason='select A and B on RGB',
                      semantic_front_verified=False, xyz_m=None,
                      confidence=None, confidence_kind='operator selection; not automatic confidence')
        if len(self.manual_ab) != 2:
            return result
        ab = np.asarray(self.manual_ab, float)
        midpoint = ab.mean(axis=0)
        u, v = np.rint(midpoint).astype(int)
        result.update(A_uv=ab[0].tolist(), B_uv=ab[1].tolist(), M_uv=midpoint.tolist(),
                      sample_uv=None, M_xyz_m=None,
                      xyz_target='actual rounded midpoint pixel; fixed image coordinates')
        if not mask[v, u]:
            result['reason'] = 'manual midpoint outside this instance mask'
            return result
        if self.manual_bbox is None:
            self.manual_bbox = bbox.copy()
        if np.max(np.abs(bbox-self.manual_bbox)) > 20:
            self.manual_invalid = True
        if self.manual_invalid:
            result['reason'] = 'pallet/bounding box moved; reselect A/B'
            return result
        if intrinsics is None:
            result['reason'] = 'registered RGB intrinsics unavailable'
            return result
        valid = mask & (depth >= self.args.min_depth_mm) & (depth <= self.args.max_depth_mm) & (depth > 0)
        if not valid[v, u]:
            result['reason'] = 'midpoint has no valid Depth; no alternate pixel used'
            return result
        patch = depth[max(0,v-2):v+3, max(0,u-2):u+3]
        support = valid[max(0,v-2):v+3, max(0,u-2):u+3]
        values = patch[support].astype(float)
        zmm = float(depth[v, u])
        if len(values) < 5 or abs(zmm-np.median(values)) > 30:
            result['reason'] = 'midpoint Depth lacks consistent local support'
            return result
        xyz = pixel_points([[u, v]], np.array([zmm/1000]), intrinsics)[0]
        if not np.isfinite(xyz).all():
            result['reason'] = 'invalid projection'
            return result
        result.update(sample_uv=[int(u), int(v)], sample_z_m=zmm/1000, xyz_m=xyz.tolist(),
                      reason='operator-selected edge; measured midpoint pixel; calibration unverified')
        return result

    def process(self, pair):
        rgb_msg, depth_msg, dt, ci, di, registration = pair
        if (rgb_msg.height, rgb_msg.width) != SHAPE or (depth_msg.height, depth_msg.width) != SHAPE:
            raise ValueError('RGB and Depth must both be 640x480; no resizing permitted')
        if depth_msg.encoding != '16UC1':
            raise ValueError(f'Expected 16UC1 millimeters, got {depth_msg.encoding}')
        rgb = decode_image(rgb_msg)
        depth = decode_image(depth_msg, depth=True)
        intrinsics, xyz_status = validate_intrinsics(rgb_msg, depth_msg, ci, registration)
        result = self.model.predict(source=rgb, conf=self.args.conf, imgsz=640,
                                    retina_masks=True, device=self.args.device, verbose=False)[0]
        if tuple(result.orig_shape) != SHAPE:
            raise ValueError('YOLO result does not reference original RGB dimensions')
        # retina_masks removes letterbox padding in Ultralytics. Do not stretch a
        # lower-resolution/padded mask. A shape mismatch is a hard error.
        count = 0 if result.boxes is None else len(result.boxes)
        if count and (result.masks is None or len(result.masks.data) != count):
            raise ValueError('Missing segmentation masks for detections')
        masks = [] if not count else result.masks.data.cpu().numpy() > 0.5
        if count and masks.shape[1:] != SHAPE:
            raise ValueError('Expected retina_masks in original 640x480 coordinates')
        rgb_view = rgb.copy()
        normalized = np.clip(depth.astype(np.float32)/self.args.max_depth_mm*255, 0, 255).astype(np.uint8)
        depth_view = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        depth_view[depth == 0] = (255, 0, 255)
        records, files, labels = [], {}, []
        for i, mask in enumerate(masks):
            box = result.boxes[i]
            cls, confidence = int(box.cls.item()), float(box.conf.item())
            name = result.names[cls]
            record, candidate, used = measure_mask(mask, depth, self.args, intrinsics)
            old_pixel = record['pixel_uv']
            old_xyz = record['xyz_m']
            bbox = box.xyxy[0].cpu().numpy()
            edge = self.front_edges.select(rgb, depth, mask, used, intrinsics, bbox, cls,
                                           stamp_ns(rgb_msg)/1e9) if record['median_z_m'] is not None else dict(
                status='FRONT EDGE NOT FOUND', reason='insufficient valid Depth', xyz_m=None)
            if self.manual_ab:
                edge = self.manual_measure(depth, mask, used, intrinsics, bbox)
            record.update(front_edge=edge, old_pixel_uv=old_pixel, old_xyz_m=old_xyz,
                          pixel_uv=edge.get('sample_uv'), pixel_z_m=edge.get('sample_z_m'),
                          xyz_m=edge.get('xyz_m'), xyz_frame_id=OPTICAL_FRAME if edge.get('xyz_m') else None)
            record.update(instance=i, class_id=cls, class_name=name, confidence=confidence)
            records.append(record)
            # Original mask remains unchanged for RGB; erosion is sampling only.
            color = np.array(((40 + i*83) % 200 + 30, (130 + i*47) % 200 + 30,
                              (80 + i*67) % 200 + 30), np.uint8)
            rgb_view[mask] = (0.65*rgb_view[mask] + 0.35*color).astype(np.uint8)
            contours, _ = cv2.findContours(candidate.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(depth_view, contours, -1, (0, 255, 255), 1)
            depth_view[used] = (0.4*depth_view[used] + 0.6*np.array([0, 255, 0])).astype(np.uint8)
            text = 'Invalid depth' if record['median_z_m'] is None else f"Zmed={record['median_z_m']:.3f} m"
            label = f'#{i} {name} {confidence:.2f} | {text} | valid={record["valid_ratio"]:.0%}'
            xyxy = box.xyxy[0].cpu().numpy()
            x = max(2, min(250, int(xyxy[0])))
            y = max(20, min(455, int(xyxy[1])))
            labels.append((label, x, y, record))
            if edge.get('A_uv') is not None:
                a, b, m = (tuple(np.rint(edge[k]).astype(int)) for k in ('A_uv', 'B_uv', 'M_uv'))
                for view in (rgb_view, depth_view):
                    cv2.line(view, a, b, (0, 255, 255), 2)
                    for point, title in ((a, 'A'), (b, 'B'), (m, 'M')):
                        cv2.drawMarker(view, point, (0, 0, 255), cv2.MARKER_CROSS, 13, 2)
                        put(view, title, point[0]+5, max(12, point[1]-6), scale=.4)
            if self.show_old_point and old_pixel is not None:
                for view in (rgb_view, depth_view):
                    cv2.circle(view, tuple(old_pixel), 5, (255, 0, 255), 1)
                    put(view, 'old', old_pixel[0]+6, old_pixel[1], scale=.4)
            if record['pixel_uv'] is not None:
                uv = tuple(record['pixel_uv'])
                for view in (rgb_view, depth_view):
                    cv2.drawMarker(view, uv, (0, 0, 0), cv2.MARKER_CROSS, 19, 4)
                    cv2.drawMarker(view, uv, (255, 255, 255), cv2.MARKER_CROSS, 17, 1)
                    put(view, f'P#{i}', uv[0]+8, max(16, uv[1]-8))
            files[f'mask_{i}.png'] = mask.astype(np.uint8)*255
            files[f'candidate_{i}.png'] = candidate.astype(np.uint8)*255
            files[f'used_pixels_{i}.png'] = used.astype(np.uint8)*255
        for label, x, y, record in labels:
            put(rgb_view, label, x, y, scale=0.43)
            put(rgb_view, record['front_edge']['status'], x, min(478, y+36), (0, 220, 255), scale=.40)
            xyz = record['xyz_m']
            if xyz is not None:
                put(rgb_view, 'P: X={:.3f} m Y={:.3f} m Z={:.3f} m'.format(*xyz), x, min(478, y+18), scale=0.43)
            else:
                put(rgb_view, 'XYZ unavailable', x, min(478, y+18), scale=0.43)
        self.rgb_view, self.depth_view = rgb_view, depth_view
        self.summary = f'displayed dt={dt:+.2f} ms | RGB/Depth 640x480 | detections={count}'
        self.xyz_status = (xyz_status + ' | edge XYZ available') if any(r['xyz_m'] is not None for r in records) else 'XYZ unavailable: ' + ('; '.join(r['front_edge'].get('reason', '') for r in records) or 'no pallet')
        self.ratios = ', '.join(f'#{i}: {r["valid_pixels"]}/{r["candidate_pixels"]} ({r["valid_ratio"]:.1%})'
                                for i, r in enumerate(records)) or 'No masks'
        self.error = ''
        self.snapshot = (
            dict(rgb=image_metadata(rgb_msg), depth=image_metadata(depth_msg), delta_ms=dt,
                 color_camera_info=info_metadata(ci), depth_camera_info=info_metadata(di),
                 xyz_status=xyz_status, experimental=True, calibration_verified=False,
                 warning=EXPERIMENTAL, xyz_frame_id=OPTICAL_FRAME,
                 intrinsics_source='/camera/color/camera_info',
                 registration_check=registration, depth_intrinsics_used=False,
                 rgb_intrinsics_valid=intrinsics is not None,
                 distance_definition='median optical Z, not Euclidean range',
                 xyz_definition='P on candidate plane near M; not XYZ of M; +X right, +Y down, +Z forward',
                 front_edge_semantics_verified=False,
                 measurements=records, erode_px=self.args.erode_px,
                 min_valid_pixels=self.args.min_valid_pixels, min_valid_ratio=self.args.min_valid_ratio,
                 depth_range_mm=[self.args.min_depth_mm, self.args.max_depth_mm],
                 model=str(self.args.model), conf=self.args.conf, imgsz=640),
            dict(files, **{'rgb.png': rgb, 'depth_mm.png': depth,
                          'rgb_annotated.png': rgb_view, 'depth_annotated.png': depth_view}))

    def compose(self, fps, status):
        canvas = np.zeros((660, 1280, 3), np.uint8)
        canvas[32:512, :640] = self.rgb_view
        canvas[32:512, 640:] = self.depth_view
        put(canvas, 'RGB: AUTO EDGE / M=midpoint P=Depth sample | R:reset | O:old', 10, 23)
        put(canvas, 'Depth: GREEN=valid samples, YELLOW=eroded mask, MAGENTA=zero', 650, 23, scale=0.43)
        for y, line in enumerate((EXPERIMENTAL, f'FPS={fps:.1f} | {self.summary}',
                                  self.xyz_status, f'Valid pixels: {self.ratios}',
                                  self.error or status,
                                  'Zmed = optical depth, not range. HOLD retains last result. Q: quit | S: save')):
            put(canvas, line[:165], 10, 536+y*22, (0, 220, 255) if y == 0 else (255, 255, 255))
        return canvas

    def save(self, canvas):
        if self.snapshot is None:
            print('No accepted result to save')
            return
        metadata, files = self.snapshot
        directory = self.args.output_dir / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        try:
            directory.mkdir(parents=True, exist_ok=False)
            for name, data in {**files, 'inspection.png': canvas}.items():
                if not cv2.imwrite(str(directory/name), data):
                    raise OSError(f'Failed writing {name}')
            (directory/'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
            print(f'Saved {directory.resolve()}')
        except (OSError, cv2.error) as exc:
            print(f'Save failed (folder may be incomplete): {exc}', file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=Path(__file__).resolve().parent/'models/best_v2.pt')
    parser.add_argument('--manual-edge', action='store_true', help='Optional manual A/B diagnostic mode; default is automatic')
    parser.add_argument('--show-old-point', action='store_true', help='Legacy option; press O to toggle the old comparison point')
    parser.add_argument('--conf', type=float, default=0.60,
                        help='YOLO confidence threshold (default: 0.60)')
    parser.add_argument('--device', default='0' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--max-delta-ms', type=float, default=50)
    parser.add_argument('--max-age-ms', type=float, default=250, help='Maximum pending frame residence time')
    parser.add_argument('--display-fps', type=float, default=10)
    parser.add_argument('--erode-px', type=int, default=3)
    parser.add_argument('--min-valid-pixels', type=int, default=100)
    parser.add_argument('--min-valid-ratio', type=float, default=0.30)
    parser.add_argument('--min-depth-mm', type=int, default=1)
    parser.add_argument('--max-depth-mm', type=int, default=10000)
    parser.add_argument('--output-dir', type=Path, default=Path('yolo_depth_captures'))
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not np.isfinite(args.conf) or not 0 <= args.conf <= 1:
        parser.error('--conf must be finite and between 0 and 1')
    if not np.isfinite([args.max_delta_ms, args.max_age_ms, args.display_fps, args.min_valid_ratio]).all():
        parser.error('Numeric arguments must be finite')
    if not 0 <= args.max_delta_ms <= 50 or args.max_age_ms <= 0 or args.display_fps <= 0:
        parser.error('Require delta 0..50 ms, age > 0 and FPS > 0')
    if not 0 <= args.erode_px <= 50 or args.min_valid_pixels < 1 or not 0 < args.min_valid_ratio <= 1:
        parser.error('Require erosion 0..50, min pixels >= 1, valid ratio in (0,1]')
    if not 1 <= args.min_depth_mm < args.max_depth_mm <= 65535:
        parser.error('Invalid depth range in millimeters')
    if not args.model.is_file():
        parser.error(f'Model not found: {args.model}')
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('No graphical display: enable WSLg/X server')
    print(f'Python={sys.executable} | OpenCV={cv2.__version__} | NumPy={np.__version__} | '
          f'Ultralytics={ultralytics.__version__} | device={args.device}')
    print(EXPERIMENTAL)
    model = YOLO(str(args.model))
    if model.task != 'segment':
        parser.error(f'Model task is {model.task}, expected segment')
    # Warm up before subscribing, so initial model setup cannot queue camera frames.
    model.predict(np.zeros((*SHAPE, 3), np.uint8), conf=args.conf, imgsz=640,
                  retina_masks=True, device=args.device, verbose=False)
    rclpy.init(args=sys.argv)
    node = CameraReceiver(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    receiver_thread = threading.Thread(target=executor.spin, daemon=True)
    receiver_thread.start()
    processor = Processor(args, model)
    fps, count, started = 0.0, 0, time.monotonic()
    next_display = 0.0
    canvas = processor.compose(fps, node.status)
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | getattr(cv2, 'WINDOW_GUI_NORMAL', 0))
        cv2.resizeWindow(WINDOW, 1280, 660)
        cv2.setMouseCallback(WINDOW, processor.mouse)
        while rclpy.ok():
            now = time.monotonic()
            if now >= next_display:
                pair = node.take_pair()
                if pair is not None:
                    try:
                        processor.process(pair)
                    except Exception as exc:
                        processor.error = f'HOLD / {type(exc).__name__}: {exc}'
                        print(processor.error, file=sys.stderr)
                elapsed = time.monotonic()-started
                if elapsed >= 1:
                    fps = count/elapsed
                    print(f'FPS={fps:.2f} | {processor.summary} | valid={processor.ratios} | {node.status}')
                    started, count = time.monotonic(), 0
                canvas = processor.compose(fps, node.status)
                cv2.imshow(WINDOW, canvas)
                count += 1
                next_display = time.monotonic() + 1/args.display_fps
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), ord('Q')):
                break
            if key in (ord('r'), ord('R')):
                processor.manual_ab = []
                processor.manual_bbox = None
                processor.front_edges = FrontEdgeTracker()
            if key in (ord('o'), ord('O')):
                processor.show_old_point = not processor.show_old_point
            if key in (ord('s'), ord('S')):
                processor.save(canvas)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        receiver_thread.join(timeout=2)
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
