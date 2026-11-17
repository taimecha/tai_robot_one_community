"""Bounded semantic acquisition followed by independent RGB-D measurement."""
from collections import deque
import math

from pallet_detection_selection import MIN_PALLET_CONFIDENCE, PALLET_CLASSES


def valid_target(msg, now_s):
    values = (msg.camera_forward_m, msg.chassis_front_forward_m, msg.opening_lateral_m,
              msg.entry_heading_rad, msg.face_yaw_rad, msg.rgb_depth_delta_ms,
              msg.opening_width_m, msg.leg_center_separation_m)
    stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    return (msg.valid and msg.header.frame_id == 'camera_color_optical_frame'
            and all(math.isfinite(v) for v in values) and -.05 <= now_s-stamp <= .25
            and msg.camera_forward_m > 0 and abs(msg.rgb_depth_delta_ms) <= 50
            and min(msg.left_depth_samples, msg.right_depth_samples) >= 60
            and .172 <= msg.opening_width_m <= .332
            and .15 <= msg.leg_center_separation_m <= .60
            and abs(msg.entry_heading_rad+msg.face_yaw_rad) < .001)


class AcquisitionSession:
    MIN_SECONDS = 3.0
    # One missed 1-Hz neural inference must not discard two good independent
    # classifications of the same tracked pallet. This is semantic identity
    # only; every result is fresh when admitted and metric RGB-D gates below
    # remain separate. The total evidence window is bounded to five seconds.
    MAX_DETECTION_GAP_S = 2.5
    SEMANTIC_WINDOW_S = 5.0
    # Minimum observation span is a quality gate, never an acquisition deadline.
    MAX_SECONDS = None

    def __init__(self, seed_age_s=1.0):
        self.seed_age_s = seed_age_s
        # Must be set only by a separately validated calibration/accuracy gate.
        # Semantic confirmation can stop YOLO, but cannot establish accuracy.
        self.accuracy_validated = False
        self.session = 0
        self.started = None
        self.started_ros = None
        self.phase = 'PAUSED'
        self.locked = False
        self.reason = 'Chờ bắt đầu nhận diện'
        self.detections = deque(maxlen=8)
        self.targets = deque(maxlen=64)
        self.last_detection = None
        self.last_target = None
        self.target_received = None
        self.target_ok = False

    def start(self, wall, ros):
        self.session += 1
        self.started, self.started_ros = wall, ros
        self.phase, self.locked = 'ACQUIRING', False
        self.reason = 'Đang nhận diện pallet; chờ đủ dữ liệu mới và ổn định'
        self.detections.clear(); self.targets.clear()
        self.last_detection = self.last_target = self.target_received = None
        self.target_ok = False

    def detection(self, stamp, wall, ros, valid, confidence=0., bbox=(), label=''):
        if self.phase != 'ACQUIRING' or stamp < self.started_ros:
            return
        if self.last_detection is not None and stamp <= self.last_detection:
            return
        self.last_detection = stamp
        if (not valid or not -.05 <= ros-stamp <= self.seed_age_s or not math.isfinite(confidence)
                or not MIN_PALLET_CONFIDENCE <= confidence <= 1. or label not in PALLET_CLASSES
                or len(bbox) != 4 or not all(math.isfinite(v) for v in bbox)
                or bbox[2] <= bbox[0] or bbox[3] <= bbox[1]):
            self.detections.clear()
            return
        if self.detections:
            previous = self.detections[-1]
            a, b = bbox, previous[2]
            intersection = max(0., min(a[2],b[2])-max(a[0],b[0])) * max(0., min(a[3],b[3])-max(a[1],b[1]))
            union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
            if (label != previous[3] or stamp-previous[0] > self.MAX_DETECTION_GAP_S
                    or intersection/union < .5):
                self.detections.clear()
        self.detections.append((stamp, wall, tuple(bbox), label))
        while self.detections and stamp-self.detections[0][0] > self.SEMANTIC_WINDOW_S:
            self.detections.popleft()

    def target(self, stamp, wall, ros, valid, x=0., y=0., heading=0., lost=False):
        if self.started_ros is None or stamp < self.started_ros:
            return
        if self.last_target is not None and stamp <= self.last_target:
            return
        self.last_target = stamp
        self.target_ok = valid and -.05 <= ros-stamp <= .25 and all(math.isfinite(v) for v in (x,y,heading))
        self.target_received = wall
        if not self.target_ok:
            self.targets.clear()
            if lost and self.locked:
                self.locked, self.phase = False, 'LOST'
                self.reason = 'Mất dấu pallet; bấm nhận diện lại'
            return
        if self.targets:
            old = self.targets[-1]
            if stamp-old[0] > .25 or abs(x-old[1]) > .03 or abs(y-old[2]) > .02 or abs(heading-old[3]) > math.radians(2):
                self.targets.clear()
        self.targets.append((stamp,x,y,heading))
        while self.targets and stamp-self.targets[0][0] > .5:
            self.targets.popleft()

    def measurement_ready(self, wall, ros):
        return (self.target_ok and self.target_received is not None and wall-self.target_received <= .25
                and -.05 <= ros-self.last_target <= .25 and len(self.targets) >= 3
                and self.targets[-1][0]-self.targets[0][0] >= .2
                and max(v[1] for v in self.targets)-min(v[1] for v in self.targets) <= .03
                and max(v[2] for v in self.targets)-min(v[2] for v in self.targets) <= .02
                and max(v[3] for v in self.targets)-min(v[3] for v in self.targets) <= math.radians(2))

    def tick(self, wall, ros):
        """Return True exactly when this session must pause YOLO."""
        if self.phase != 'ACQUIRING':
            return False
        elapsed = wall-self.started
        semantic = (len(self.detections) >= 3 and wall-self.detections[-1][1] <= 1.0
                    and -.05 <= ros-self.detections[-1][0] <= self.seed_age_s)
        # Gather geometry after semantic acquisition has released the neural
        # worker. Requiring the RGB-D window here kept YOLO running whenever
        # its CPU/transport load interrupted that very window. Pausing does
        # not lock a target: VERIFYING_RGBD still needs fresh geometry.
        if elapsed >= self.MIN_SECONDS and semantic:
            self.phase, self.locked = 'STOPPING_YOLO', False
            self.reason = 'Đang chờ inference kết thúc và heartbeat YOLO tắt'
            return True
        return False

    def yolo_stopped(self):
        if self.phase == 'STOPPING_YOLO':
            self.phase = 'VERIFYING_RGBD'
            self.reason = 'Chờ các phép đo RGB-D mới sau khi YOLO dừng'

    def verified(self):
        if self.phase == 'VERIFYING_RGBD':
            if self.accuracy_validated:
                self.phase, self.locked = 'READY', True
                self.reason = 'Đã xác minh; chờ người vận hành bắt đầu'
            else:
                self.phase, self.locked = 'CALIBRATION_REQUIRED', False
                self.reason = 'YOLO đã tắt; chưa xác nhận sai số tuyệt đối để chạy xe'

    def pause(self):
        if self.phase == 'ACQUIRING':
            self.phase = 'PAUSED'
            self.reason = 'Đã dừng nhận diện; bấm nhận diện lại để xác nhận pallet'

    def snapshot(self, wall, ros):
        return dict(session=self.session, started_ros_s=self.started_ros,
                    phase=self.phase, locked=self.locked,
                    measurement_ready=self.locked and self.measurement_ready(wall, ros),
                    reason=self.reason, elapsed_s=max(0., wall-self.started) if self.started is not None else 0.,
                    min_seconds=self.MIN_SECONDS, max_seconds=self.MAX_SECONDS,
                    detections=len(self.detections), target_samples=len(self.targets),
                    semantic_capture_stamps_s=[d[0] for d in self.detections],
                    accuracy_validated=self.accuracy_validated)
