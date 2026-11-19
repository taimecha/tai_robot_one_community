"""Independent endpoint-to-map consistency check for an AMCL hypothesis.

This is a gate on the supplied pose, not a global localization algorithm or
proof that the map has a unique matching location. It uses only the standard
library so the bridge can run in the system ROS Python environment.
"""
import heapq
import math
import statistics
from types import SimpleNamespace


# Use more of a real scan for continuous navigation readiness. This reduces
# sampling jitter near the configured runtime/teaching fit thresholds.
LIVE_SCAN_MAX_SAMPLES = 720
NAVIGATION_MIN_INLIER_RATIO = .65
NAVIGATION_MAX_MEDIAN_M = .12
RECOGNITION_MIN_INLIER_RATIO = NAVIGATION_MIN_INLIER_RATIO
RECOGNITION_MAX_MEDIAN_M = .05


def _validate_scan_match(fit, purpose, min_ratio, max_median):
    ratio=fit.get('inlier_ratio') if isinstance(fit,dict) else None
    median=fit.get('median_distance_m') if isinstance(fit,dict) else None
    finite=lambda value:isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
    if (not isinstance(fit,dict) or fit.get('ready') is not True
            or not finite(ratio) or not min_ratio<=ratio<=1.
            or not finite(median) or not 0.<=median<=max_median+1e-9):
        ratio_text=f'{ratio*100:.1f}%' if finite(ratio) else 'unavailable'
        median_text=f'{median*100:.2f} cm' if finite(median) else 'unavailable'
        raise ValueError(f'{purpose} scan/map agreement requires >={min_ratio*100:g}% inliers and median wall '
                         f'distance <={max_median*100:g} cm (actual {ratio_text}, {median_text})')


def validate_recognition_scan_match(fit):
    """Require close wall agreement before pallet measurement at A."""
    _validate_scan_match(fit, 'recognition', RECOGNITION_MIN_INLIER_RATIO, RECOGNITION_MAX_MEDIAN_M)


def validate_navigation_scan_match(fit):
    """Use the same wall-fit limits as the continuous localization monitor."""
    _validate_scan_match(fit, 'navigation', NAVIGATION_MIN_INLIER_RATIO, NAVIGATION_MAX_MEDIAN_M)


class ScanMapCheck:
    def __init__(self, grid, occupied_threshold=65, max_distance_m=.5):
        self.width, self.height = int(grid.info.width), int(grid.info.height)
        self.resolution = float(grid.info.resolution)
        origin = grid.info.origin
        q = origin.orientation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if (self.width <= 0 or self.height <= 0
                or not math.isfinite(self.resolution) or self.resolution <= 0
                or not all(math.isfinite(v) for v in
                           (origin.position.x, origin.position.y, norm))
                or abs(norm-1.) > .02
                or len(grid.data) != self.width*self.height
                or not 1 <= occupied_threshold <= 100
                or not math.isfinite(max_distance_m) or max_distance_m < .15):
            raise ValueError('invalid occupancy map for scan consistency')
        self.origin = (float(origin.position.x), float(origin.position.y))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.cos_yaw, self.sin_yaw = math.cos(yaw), math.sin(yaw)
        self.data = tuple(int(value) for value in grid.data)
        self.max_distance_m = max_distance_m
        self.distance = [math.inf]*len(self.data)
        self.nearest_occupied = [None]*len(self.data)
        occupied = [value >= occupied_threshold for value in self.data]
        if not any(occupied):
            raise ValueError('occupancy map contains no occupied cells')
        queue = []
        # All wall cells have distance zero, but only their boundaries need
        # enter the heap. Propagate a bounded 8-neighbour distance field once.
        for index, is_wall in enumerate(occupied):
            if not is_wall:
                continue
            self.distance[index] = 0.
            self.nearest_occupied[index] = index
            x, y = index % self.width, index // self.width
            boundary = any(0 <= nx < self.width and 0 <= ny < self.height
                           and not occupied[ny*self.width+nx]
                           for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)))
            if boundary:
                queue.append((0., index))
        heapq.heapify(queue)
        steps = [(dx,dy,self.resolution*math.hypot(dx,dy))
                 for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy]
        while queue:
            distance, index = heapq.heappop(queue)
            if distance != self.distance[index]:
                continue
            x, y = index % self.width, index // self.width
            for dx,dy,cost in steps:
                nx,ny = x+dx,y+dy
                candidate = distance+cost
                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue
                neighbour = ny*self.width+nx
                if candidate <= max_distance_m and candidate < self.distance[neighbour]:
                    self.distance[neighbour] = candidate
                    self.nearest_occupied[neighbour] = self.nearest_occupied[index]
                    heapq.heappush(queue,(candidate,neighbour))

    def endpoint_distance(self, x, y):
        dx, dy = x-self.origin[0], y-self.origin[1]
        local_x = self.cos_yaw*dx+self.sin_yaw*dy
        local_y = -self.sin_yaw*dx+self.cos_yaw*dy
        gx = math.floor(local_x/self.resolution)
        gy = math.floor(local_y/self.resolution)
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return math.inf
        index = gy*self.width+gx
        if self.data[index] < 0:
            return math.inf
        # Occupancy values describe square cell areas, not zero-size points
        # at cell centres. The old cell-to-cell field assigned a whole 5 cm
        # step to an endpoint even 0.5 mm outside a wall cell. Keep the field
        # only as a bounded nearest-source index and measure the actual point
        # against occupied square bounds in the map's local coordinates.
        # Neighbour source candidates reduce grid Voronoi ambiguities. A
        # missed source can only overestimate distance: every candidate here
        # remains a real occupied cell, never interpolated/expanded free space.
        candidates=set()
        for ny in range(max(0,gy-1),min(self.height,gy+2)):
            for nx in range(max(0,gx-1),min(self.width,gx+2)):
                source=self.nearest_occupied[ny*self.width+nx]
                if source is not None:
                    candidates.add(source)
        best=math.inf
        for source in candidates:
            sx,sy=source%self.width,source//self.width
            left,bottom=sx*self.resolution,sy*self.resolution
            ex=max(left-local_x,0.,local_x-(left+self.resolution))
            ey=max(bottom-local_y,0.,local_y-(bottom+self.resolution))
            best=min(best,math.hypot(ex,ey))
        return best if best<=self.max_distance_m else math.inf

    def score(self, scan, sensor_pose_map, *, max_samples=120, min_endpoints=30,
              inlier_distance_m=.15, min_inlier_ratio=NAVIGATION_MIN_INLIER_RATIO,
              max_median_distance_m=NAVIGATION_MAX_MEDIAN_M):
        if (len(sensor_pose_map) != 3
                or not all(math.isfinite(v) for v in sensor_pose_map)
                or not all(math.isfinite(v) for v in
                           (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max))
                or scan.angle_increment == 0 or scan.range_min < 0
                or scan.range_max <= scan.range_min or max_samples < min_endpoints
                or min_endpoints < 1):
            raise ValueError('invalid scan geometry for map consistency')
        # Max-range returns indicate no measured obstacle and cannot verify a
        # wall hypothesis. Unknown or out-of-map endpoints remain mismatches.
        valid = [(index,float(distance)) for index,distance in enumerate(scan.ranges)
                 if math.isfinite(distance) and 0 < distance
                 and scan.range_min <= distance < scan.range_max*(1.-1e-6)]
        selected = valid
        if len(valid) > max_samples:
            selected = [valid[round(i*(len(valid)-1)/(max_samples-1))]
                        for i in range(max_samples)]
        distances = []
        sx,sy,yaw = sensor_pose_map
        for index,distance in selected:
            angle = yaw+scan.angle_min+index*scan.angle_increment
            distances.append(self.endpoint_distance(sx+distance*math.cos(angle),
                                                     sy+distance*math.sin(angle)))
        count = len(distances)
        # OccupancyGrid resolution is float32. Its representation must not
        # exclude a point on the nominal threshold by a few nanometres.
        inliers = sum(distance <= inlier_distance_m+1e-7 for distance in distances)
        ratio = inliers/count if count else 0.
        median = statistics.median(distances) if distances else math.inf
        ready = count >= min_endpoints and ratio >= min_inlier_ratio and median <= max_median_distance_m+1e-7
        if count < min_endpoints:
            reason = 'insufficient finite obstacle endpoints for scan/map verification'
        elif not ready:
            reason = 'LiDAR endpoints do not fit the selected map at the AMCL pose'
        else:
            reason = 'scan/map endpoint fit accepted; global uniqueness is not established'
        return dict(ready=ready, reason=reason, valid_endpoints=len(valid),
                    sampled_endpoints=count, inlier_endpoints=inliers,
                    inlier_ratio=ratio,
                    median_distance_m=median if math.isfinite(median) else None,
                    inlier_distance_m=inlier_distance_m,
                    min_inlier_ratio=min_inlier_ratio,
                    max_median_distance_m=max_median_distance_m,
                    distance_field='point-to-occupied-cell-area distance with bounded nearest-source candidates',
                    uniqueness_verified=False)

    def score_trusted_sectors(self, scan, sensor_pose_map, base_sensor):
        """Report map fit in the robot-left and rear-left endpoint regions.

        ``base_sensor`` is the full planar sensor pose in base_footprint at the
        scan stamp. Selection uses endpoint coordinates in that frame, not raw
        LiDAR angles, so a rotated or offset mounting cannot swap robot sides.
        These are inspection diagnostics and never adjust the supplied pose.
        """
        if len(base_sensor) != 3 or not all(math.isfinite(v) for v in base_sensor):
            raise ValueError('invalid base/sensor transform for trusted sectors')
        tx, ty, yaw = base_sensor
        masks = {'robot_left': [], 'rear_left': []}
        for index, distance in enumerate(scan.ranges):
            valid = (math.isfinite(distance) and 0 < distance
                     and scan.range_min <= distance < scan.range_max * (1. - 1e-6))
            if valid:
                angle = yaw + scan.angle_min + index * scan.angle_increment
                x = tx + distance * math.cos(angle)
                y = ty + distance * math.sin(angle)
                left = y > .25
                rear_left = left and x < -.3
            else:
                left = rear_left = False
            masks['robot_left'].append(distance if left else math.nan)
            masks['rear_left'].append(distance if rear_left else math.nan)
        result = {
            'selection_frame': 'base_footprint',
            'base_sensor_xyyaw': list(base_sensor),
            'used_for_pose_refinement': False,
            'physical_accuracy_verified': False,
        }
        for name, ranges in masks.items():
            masked_scan = SimpleNamespace(
                angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                range_min=scan.range_min, range_max=scan.range_max, ranges=ranges)
            fit = self.score(masked_scan, sensor_pose_map, max_samples=60, min_endpoints=15)
            fit['endpoint_selection'] = ('base y > 0.25 m' if name == 'robot_left'
                                         else 'base y > 0.25 m and base x < -0.30 m')
            result[name] = fit
        return result
