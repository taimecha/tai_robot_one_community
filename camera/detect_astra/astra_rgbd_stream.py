"""Astra registered RGB/Depth input and validation; no neural-network imports."""
from datetime import datetime, timezone
import json
import threading
import time

import cv2
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

SHAPE = (480, 640)
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
                             self.source_callback(stream, True),
                             QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=16,
                                        reliability=ReliabilityPolicy.BEST_EFFORT,
                                        durability=DurabilityPolicy.VOLATILE)))

        # ROS graph/parameter inspection can take hundreds of milliseconds.
        # Keep it serialized with itself, independently of image delivery.
        self.registration_group = MutuallyExclusiveCallbackGroup()
        self.registration_timer = self.create_timer(
            1.0, self.check_registration, callback_group=self.registration_group)

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
            # Image callbacks may discover a second actual writer while the
            # graph check is in flight. Its stale snapshot cannot re-enable XYZ.
            if verified:
                for key in ('rgb', 'depth', 'rgb_info'):
                    if len(self.source_observations.get(key, {})) > 1:
                        verified, reason = False, f'multiple observed {key} GIDs'
                        break
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
                self.registration_client = self.create_client(
                    GetParameters, source + '/get_parameters',
                    callback_group=self.registration_group)
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
