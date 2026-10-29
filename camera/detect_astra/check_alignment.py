#!/usr/bin/python3
"""Inspect RGB/depth pixel correspondence; does not perform registration.

Run: source /opt/ros/jazzy/setup.bash
     /usr/bin/python3 check_alignment.py --max-delta-ms 50
Q: quit. S: save the currently accepted pair and its metadata. E: toggle depth edges.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time
import textwrap

try:
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.utilities import remove_ros_args
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
except ImportError as exc:
    raise SystemExit(
        f"Missing/unusable dependency: {exc}\nPython: {sys.executable}\n"
        "Source /opt/ros/jazzy/setup.bash and run /usr/bin/python3.\n"
        "Check ROS packages rclpy, sensor_msgs, cv_bridge, python3-opencv, "
        "python3-numpy. Nothing has been installed automatically."
    ) from exc


WINDOW_NAME = "Astra Pro Alignment"
CANVAS_SHAPE = (1024, 1280, 3)

def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def colorize_depth(depth, maximum_mm):
    valid = depth != 0  # 16UC1: zero means no measured depth.
    scaled = np.rint(np.clip(depth.astype(np.float32) / maximum_mm, 0, 1) * 255)
    color = cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_TURBO)
    color[~valid] = (255, 0, 255)
    return color, valid


def depth_edges(depth, threshold_mm):
    """Median-denoise then measure depth gradients, without modifying input.

    Only use pixels whose complete median + Sobel support (5x5) has valid
    measurements. Hole boundaries are intentionally omitted, not treated as
    object edges. Gradient units are mm/pixel, independent of display colors.
    """
    valid = (depth != 0).astype(np.uint8)
    supported = cv2.erode(valid, np.ones((5, 5), np.uint8),
                          borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    filtered = cv2.medianBlur(depth, 3).astype(np.float32)
    dx = cv2.Sobel(filtered, cv2.CV_32F, 1, 0, ksize=3, scale=1.0 / 8)
    dy = cv2.Sobel(filtered, cv2.CV_32F, 0, 1, ksize=3, scale=1.0 / 8)
    return ((cv2.magnitude(dx, dy) >= threshold_mm) & supported).astype(np.uint8) * 255


def panel(image, lines):
    """Draw on a copy at its native size; never resize or mutate source data."""
    output = image.copy()
    columns = max(1, (image.shape[1] - 16) // 8)
    wrapped = [part for line in lines for part in textwrap.wrap(line, columns)]
    max_lines = max(1, (image.shape[0] - 12) // 18)
    wrapped = wrapped[:max_lines]
    y0 = max(0, image.shape[0] - 18 * len(wrapped) - 10)
    cv2.rectangle(output, (0, y0), (image.shape[1] - 1, image.shape[0] - 1), (0, 0, 0), -1)
    for i, line in enumerate(wrapped):
        cv2.putText(output, line, (8, y0 + 18 * (i + 1)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return output


class AlignmentChecker(Node):
    def __init__(self, args):
        super().__init__('check_alignment')
        self.args = args
        self.bridge = CvBridge()
        self.latest = {}  # One pending message per stream; callbacks overwrite old frames.
        self.views = {name: np.zeros((480, 640, 3), np.uint8)
                      for name in ('RGB', 'Depth', 'RGB+Depth Overlay')}
        self.status = 'Waiting for a synchronized pair'
        self.displayed_pair_info = 'No accepted pair displayed'
        self.details = []
        self.last_pair_info = 'No pair received'
        self.display_fps = 0.0
        self.fps_started = time.monotonic()
        self.display_count = 0
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.last_received = {}
        self.last_warning = {}
        self.snapshot = None
        self.edges_enabled = False
        self.subscriptions_ = [
            self.create_subscription(Image, '/camera/color/image_raw',
                                     lambda msg: self.receive('rgb', msg), qos),
            self.create_subscription(Image, '/camera/depth/image_raw',
                                     lambda msg: self.receive('depth', msg), qos),
        ]
        self.get_logger().info(
            'QoS: BEST_EFFORT / VOLATILE (accepts reliable or best-effort publishers). '
            f'Max timestamp delta: {args.max_delta_ms:g} ms. No registration or resizing.')
        self.show_error('Waiting for both image topics...')

    def warn(self, key, message):
        now = time.monotonic()
        if now - self.last_warning.get(key, -float('inf')) >= 2:
            self.get_logger().warning(message)
            self.last_warning[key] = now

    def show_error(self, message):
        # Keep the last accepted pair visible instead of flashing blank images.
        # Rejected frames never enter any of the three image panes.
        self.status = 'HOLD / ' + message

    def compose_view(self):
        """Fixed 2x2 layout; native 640x480 images, labels and separate status."""
        canvas = np.zeros(CANVAS_SHAPE, np.uint8)
        for name, x, y in (('RGB', 0, 0), ('Depth', 640, 0),
                           ('RGB+Depth Overlay', 0, 512)):
            label = 'RGB + Depth edges (red)' if name == 'RGB' and self.edges_enabled else name
            cv2.putText(canvas, label, (x + 12, y + 23),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            canvas[y + 32:y + 512, x:x + 640] = self.views[name]
        info = [
            'Astra Pro Alignment',
            f'Display FPS: {self.display_fps:.2f} | cap: {self.args.display_fps:g}',
            self.status,
            'DISPLAYED PAIR: ' + self.displayed_pair_info,
            'LATEST INPUT: ' + self.last_pair_info,
            *self.details,
            f'Depth edges: {"ON" if self.edges_enabled else "OFF"} | '
            f'gradient >= {self.args.edge_threshold_mm:g} mm/pixel',
            'E: toggle red Depth edges on RGB',
            'Edges near invalid Depth holes are excluded.',
            'Q: quit | S: save displayed accepted pair',
            'HOLD: images remain from the last accepted pair.',
            'No resize / registration performed.',
        ]
        canvas[512:1024, 640:1280] = panel(np.zeros((512, 640, 3), np.uint8), info)
        return canvas

    def toggle_edges(self):
        self.edges_enabled = not self.edges_enabled
        self.update_rgb_view()
        self.get_logger().info(f'Depth edges: {"ON" if self.edges_enabled else "OFF"}')

    def update_rgb_view(self):
        # Always start with the original accepted RGB, including while on HOLD.
        if self.snapshot is not None:
            images, _ = self.snapshot
            key = 'rgb_depth_edges.png' if self.edges_enabled else 'rgb.png'
            self.views['RGB'] = images[key].copy()

    def receive(self, stream, msg):
        # No conversion or GUI work in ROS callbacks.
        self.last_received[stream] = time.monotonic()
        self.latest[stream] = msg

    def process_latest(self):
        """Called only at the GUI rate; no FIFO of pairs to build latency."""
        if self.check_stale():
            return
        if 'rgb' not in self.latest or 'depth' not in self.latest:
            return
        rgb, depth = self.latest['rgb'], self.latest['depth']
        delta_ms = (stamp_ns(depth) - stamp_ns(rgb)) / 1e6
        accepted = abs(delta_ms) <= self.args.max_delta_ms
        self.last_pair_info = (
            f'dt={delta_ms:+.3f} ms | RGB={rgb.width}x{rgb.height} | '
            f'Depth={depth.width}x{depth.height}')
        if accepted:
            self.latest.clear()  # Each selected frame is processed at most once.
        else:
            # Keep only the newer timestamp to await its counterpart.
            del self.latest['rgb' if delta_ms > 0 else 'depth']
            self.warn('timestamp', f'Timestamp mismatch: {self.last_pair_info}; '
                      f'limit {self.args.max_delta_ms:g} ms; no overlay.')
            self.show_error(f'TIMESTAMP MISMATCH: {delta_ms:+.3f} ms; '
                            f'limit {self.args.max_delta_ms:g} ms.')
            return
        self.display(rgb, depth, delta_ms)

    def report_display(self):
        """Count actual new image submissions, not idle waitKey iterations."""
        self.display_count += 1

    def report_fps(self):
        now = time.monotonic()
        elapsed = now - self.fps_started
        if elapsed >= 1.0:
            self.display_fps = self.display_count / elapsed
            self.get_logger().info(
                f'Display FPS={self.display_fps:.2f} (cap {self.args.display_fps:g}) | '
                f'last pair: {self.last_pair_info}')
            self.display_count = 0
            self.fps_started = now

    def display(self, rgb_msg, depth_msg, delta_ms):
        details = [
            f'RGB: {rgb_msg.width}x{rgb_msg.height} | frame_id={rgb_msg.header.frame_id!r}',
            f'Depth: {depth_msg.width}x{depth_msg.height} | frame_id={depth_msg.header.frame_id!r}',
            f'dt (Depth - RGB): {delta_ms:+.3f} ms | limit: {self.args.max_delta_ms:g} ms',
            f'Depth scale: 0..{self.args.depth_max_mm:g} mm (clipped) | MAGENTA = invalid (0)',
            'Q: quit | S: save accepted pair | Matching size/frame_id does NOT prove registration',
        ]
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            if depth_msg.encoding != '16UC1':
                raise ValueError(f'Expected Depth 16UC1 in mm, got {depth_msg.encoding!r}')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            colored, valid = colorize_depth(depth, self.args.depth_max_mm)
        except Exception as exc:
            self.warn('conversion', f'Image conversion failed: {exc}')
            self.show_error(f'Image conversion failed: {exc}')
            return
        if rgb.shape[:2] != depth.shape[:2]:
            error = (f'SIZE MISMATCH: RGB {rgb_msg.width}x{rgb_msg.height}, '
                     f'Depth {depth_msg.width}x{depth_msg.height}. No resize; no overlay.')
            self.warn('size', error)
            self.show_error(error)
            return
        if rgb.shape[:2] != (480, 640):
            error = ('OVERLAY REQUIRES 640x480: ' + self.last_pair_info +
                     '. Configure camera resolution; no resize; no overlay.')
            self.warn('resolution', error)
            self.show_error(error)
            return
        overlay = cv2.addWeighted(rgb, 0.6, colored, 0.4, 0)
        overlay[~valid] = (255, 0, 255)
        edges = depth_edges(depth, self.args.edge_threshold_mm)
        rgb_edges = rgb.copy()
        rgb_edges[edges != 0] = (0, 0, 255)  # BGR red, at native pixel coordinates.
        rendered = panel(overlay, details)
        self.views['RGB'] = rgb.copy()
        self.views['Depth'] = colored.copy()
        self.views['RGB+Depth Overlay'] = overlay.copy()
        self.status = 'LIVE / synchronized pair'
        self.displayed_pair_info = self.last_pair_info
        self.details = details
        self.snapshot = (
            {'rgb.png': rgb, 'depth_mm.png': depth, 'depth_color.png': colored,
             'overlay.png': overlay, 'overlay_info.png': rendered,
             'depth_edges.png': edges, 'rgb_depth_edges.png': rgb_edges,
             'invalid_mask.png': (~valid).astype(np.uint8) * 255},
            {'rgb': self.metadata(rgb_msg), 'depth': self.metadata(depth_msg),
             'delta_ms_depth_minus_rgb': delta_ms, 'max_delta_ms': self.args.max_delta_ms,
             'depth_max_mm': self.args.depth_max_mm, 'depth_alpha': 0.4,
             'invalid_depth_value_mm': 0, 'registration_verified': False,
             'edge_method': 'median3_sobel3_valid5',
             'edge_threshold_mm_per_pixel': self.args.edge_threshold_mm})
        self.update_rgb_view()

    @staticmethod
    def metadata(msg):
        return {'stamp_ns': stamp_ns(msg), 'frame_id': msg.header.frame_id,
                'width': msg.width, 'height': msg.height, 'encoding': msg.encoding}

    def check_stale(self):
        now = time.monotonic()
        missing = [s for s in ('rgb', 'depth') if now - self.last_received.get(s, 0) > 2]
        if missing:
            message = 'No frames for >2 s: ' + ', '.join(missing)
            self.warn('stale', message)
            # Only change the UI once per stale state, avoiding unnecessary redraws.
            if message != getattr(self, '_stale_message', None):
                self.show_error(message)
            self._stale_message = message
            self.latest.clear()
            return True
        self._stale_message = None
        return False

    def save(self):
        if self.snapshot is None:
            self.get_logger().warning('No accepted pair to save.')
            return
        images, metadata = self.snapshot
        metadata = {**metadata, 'edges_enabled': self.edges_enabled}
        directory = self.args.output_dir / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        try:
            directory.mkdir(parents=True, exist_ok=False)
            images = {**images, 'alignment_view.png': self.compose_view()}
            for filename, image in images.items():
                if not cv2.imwrite(str(directory / filename), image):
                    raise OSError(f'Failed writing {filename}')
            (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
            self.get_logger().info(f'Saved: {directory.resolve()}')
        except (OSError, cv2.error) as exc:
            self.get_logger().error(f'Save failed (folder may be incomplete): {exc}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-delta-ms', type=float, default=50.0)
    parser.add_argument('--depth-max-mm', type=float, default=5000.0)
    parser.add_argument('--edge-threshold-mm', type=float, default=30.0,
                        help='Depth gradient threshold in mm/pixel (default: 30)')
    parser.add_argument('--display-fps', type=float, default=10.0)
    parser.add_argument('--output-dir', type=Path, default=Path('alignment_captures'))
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not np.isfinite(args.max_delta_ms) or not 0 <= args.max_delta_ms <= 50:
        parser.error('--max-delta-ms must be finite and between 0 and 50')
    if not np.isfinite(args.depth_max_mm) or args.depth_max_mm <= 0:
        parser.error('--depth-max-mm must be finite and > 0')
    if not np.isfinite(args.edge_threshold_mm) or args.edge_threshold_mm <= 0:
        parser.error('--edge-threshold-mm must be finite and > 0')
    if not np.isfinite(args.display_fps) or args.display_fps <= 0:
        parser.error('--display-fps must be finite and > 0')
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('No graphical display. Enable WSLg/X server to use OpenCV windows.')
    rclpy.init(args=sys.argv)
    node = None
    try:
        flags = cv2.WINDOW_NORMAL | getattr(cv2, 'WINDOW_GUI_NORMAL', 0)
        cv2.namedWindow(WINDOW_NAME, flags)
        # Initial viewport only. Sensor images and the composite are never resized.
        cv2.resizeWindow(WINDOW_NAME, 1000, 800)
        node = AlignmentChecker(args)
        interval = 1.0 / args.display_fps
        next_display = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            if time.monotonic() >= next_display:
                node.process_latest()
                node.report_fps()
                cv2.imshow(WINDOW_NAME, node.compose_view())
                node.report_display()
                # Schedule from completion: no catch-up bursts after slow work.
                next_display = time.monotonic() + interval
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q')):
                break
            if key in (ord('e'), ord('E')):
                node.toggle_edges()
            if key in (ord('s'), ord('S')):
                node.save()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
