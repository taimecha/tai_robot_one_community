"""Bounded whole-map LiDAR matching for stationary initialization.

This module has no ROS imports, publishers or motor interfaces. A winning
sampled hypothesis must also match held-out beams. The score and separation
from sampled alternatives establish consistency, not physical pose accuracy.
"""
import math
import time

import cv2
import numpy as np


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GlobalScanMatcher:
    def __init__(self, grid, min_inlier_ratio=.65):
        # Avoid library worker pools competing with sensor callbacks on Pi.
        cv2.setNumThreads(1)
        if not .5 <= min_inlier_ratio <= 1.:
            raise ValueError('invalid global initialization inlier ratio')
        self.min_inlier_ratio = float(min_inlier_ratio)
        self.width, self.height = int(grid.info.width), int(grid.info.height)
        self.resolution = float(grid.info.resolution)
        origin = grid.info.origin
        quaternion = origin.orientation
        values = (origin.position.x, origin.position.y, quaternion.x,
                  quaternion.y, quaternion.z, quaternion.w)
        if (self.width <= 0 or self.height <= 0
                or not math.isfinite(self.resolution) or self.resolution <= 0
                or self.width*self.height > 2_000_000
                or len(grid.data) != self.width*self.height
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in values[2:])-1.) > .02):
            raise ValueError('invalid occupancy map for global scan matching')
        self.origin = np.asarray(values[:2], dtype=float)
        self.origin_yaw = math.atan2(2*(quaternion.w*quaternion.z+quaternion.x*quaternion.y),
                                     1-2*(quaternion.y**2+quaternion.z**2))
        self.grid = np.asarray(grid.data, dtype=np.int16).reshape(self.height, self.width)
        self.wall = self.grid >= 65
        self.free = (self.grid >= 0) & (self.grid <= 25)
        if not self.wall.any() or not self.free.any():
            raise ValueError('global scan matching needs occupied and free map cells')
        field = cv2.distanceTransform((~self.wall).astype(np.uint8),
                                      cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*self.resolution
        self.distance = np.minimum(field, .5)
        self.distance[self.grid < 0] = .5

    @staticmethod
    def _different(a, b):
        return math.dist(a[:2], b[:2]) > .35 or abs(_wrap(a[2]-b[2])) > math.radians(15.)

    def _map_pose(self, pose):
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return [float(self.origin[0]+c*pose[0]-s*pose[1]),
                float(self.origin[1]+s*pose[0]+c*pose[1]),
                _wrap(float(pose[2])+self.origin_yaw)]

    def _grid_pose(self, pose):
        dx, dy = pose[0]-self.origin[0], pose[1]-self.origin[1]
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return [c*dx+s*dy, -s*dx+c*dy, _wrap(pose[2]-self.origin_yaw)]

    def _record(self, pose, metric):
        return dict(pose_xyyaw=self._map_pose(pose), score=float(metric[0]),
                    inlier_ratio=float(metric[1]), median_distance_m=float(metric[2]),
                    crossing_wall_ratio=float(metric[3]))

    def evaluate(self, scan, base_lidar_xyyaw, pose_xyyaw):
        """Score an externally supplied map pose without optimizing or seeding."""
        if len(pose_xyyaw) != 3 or not all(math.isfinite(v) for v in pose_xyyaw):
            raise ValueError('invalid map pose for scan evaluation')
        valid, _, _, geometry = self._geometry(scan, base_lidar_xyyaw)
        if not len(valid):
            raise ValueError('no valid beams for scan evaluation')
        points, interior = geometry(valid)
        pose = self._grid_pose(pose_xyyaw)
        return self._record(pose, self._scores([pose], points, interior)[0])

    def _scores(self, candidates, endpoints, interior):
        """Array batches cap temporary storage independently of map size."""
        answers = []
        for start in range(0, len(candidates), 1024):
            poses = np.asarray(candidates[start:start+1024], dtype=float)
            c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
            def indices(points):
                x = poses[:, 0, None]+c*points[:, 0]-s*points[:, 1]
                y = poses[:, 1, None]+s*points[:, 0]+c*points[:, 1]
                ix, iy = np.floor(x/self.resolution).astype(int), np.floor(y/self.resolution).astype(int)
                inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
                return ix, iy, inside
            ix, iy, inside = indices(endpoints)
            distances = np.full(ix.shape, .5)
            distances[inside] = self.distance[iy[inside], ix[inside]]
            likelihood = np.exp(-.5*(distances/.10)**2).mean(axis=1)
            inliers = (distances <= .15+1e-7).mean(axis=1)
            ix, iy, inside = indices(interior)
            blocked = np.ones(ix.shape, dtype=bool)
            blocked[inside] = (self.wall[iy[inside], ix[inside]]
                               | (self.grid[iy[inside], ix[inside]] < 0))
            crossing = blocked.mean(axis=1)
            score = likelihood-.3*crossing
            bx = np.floor(poses[:, 0]/self.resolution).astype(int)
            by = np.floor(poses[:, 1]/self.resolution).astype(int)
            good = (bx >= 0) & (bx < self.width) & (by >= 0) & (by < self.height)
            good[good] &= self.free[by[good], bx[good]]
            score[~good] = -1.
            answers.append(np.stack((score, inliers, np.median(distances, axis=1), crossing), axis=1))
        return np.concatenate(answers)

    @staticmethod
    def _geometry(scan, base_lidar):
        if (len(base_lidar) != 3 or not all(math.isfinite(v) for v in base_lidar)
                or not all(math.isfinite(v) for v in (scan.angle_min, scan.angle_increment,
                                                     scan.range_min, scan.range_max))
                or scan.angle_increment == 0 or scan.range_min < 0 or scan.range_max <= scan.range_min):
            raise ValueError('invalid scan or LiDAR mounting geometry')
        ranges = np.asarray(scan.ranges, dtype=float)
        valid = np.flatnonzero(np.isfinite(ranges) & (ranges >= max(.15, scan.range_min))
                               & (ranges < scan.range_max*(1.-1e-6)))
        angles = scan.angle_min+valid*scan.angle_increment
        wrapped = np.sort(np.mod(angles, 2*math.pi))
        coverage = 2*math.pi-np.diff(np.r_[wrapped, wrapped[:1]+2*math.pi]).max() if len(valid) else 0.
        sectors = len(np.unique(np.floor(np.mod(angles, 2*math.pi)/(math.pi/6)).astype(int)))
        def points(index):
            if len(index) > 180:
                index = index[np.rint(np.linspace(0, len(index)-1, 180)).astype(int)]
            angle = scan.angle_min+index*scan.angle_increment+base_lidar[2]
            beam = np.stack((ranges[index]*np.cos(angle), ranges[index]*np.sin(angle)), axis=1)
            endpoints = beam+base_lidar[:2]
            interior = np.concatenate([beam[offset::4]*fraction+base_lidar[:2]
                                       for offset, fraction in enumerate((.25, .5, .75, .9))])
            return endpoints, interior
        return valid, coverage, sectors, points

    def match(self, scan, base_lidar_xyyaw):
        started = time.monotonic()
        report = dict(accepted=False, reason='', best=None, distinct_candidates=[],
                      runner_up_margin=None, uniqueness_physically_proven=False,
                      physical_accuracy_verified=False,
                      method='sampled whole-map endpoint matching with held-out beam validation',
                      min_inlier_ratio=self.min_inlier_ratio,
                      search=dict(coarse_xy_m=.20, coarse_yaw_deg=5., refined_xy_m=.025,
                                  refined_yaw_deg=.5, max_seeds=18, batch_size=1024))
        try:
            valid, coverage, sectors, geometry = self._geometry(scan, base_lidar_xyyaw)
        except (ValueError, TypeError) as exc:
            report['reason'] = str(exc)
            return report
        report.update(valid_rays=len(valid), angular_coverage_deg=math.degrees(coverage), angular_sectors=sectors)
        if len(valid) < 120 or coverage < math.pi or sectors < 7:
            report['reason'] = 'global matching requires >=120 valid rays spanning >=180 degrees and >=7 angular sectors'
            return report
        train, interior = geometry(valid[::2])
        held, held_interior = geometry(valid[1::2])
        step = max(1, int(round(.20/self.resolution)))
        yy, xx = np.where(self.free[::step, ::step])
        positions = np.column_stack(((xx*step+.5)*self.resolution, (yy*step+.5)*self.resolution))
        if not len(positions):
            report['reason'] = 'no free-space sample on the bounded global search grid'
            return report
        if len(positions) > 6_000:
            report['reason'] = 'map exceeds bounded global search budget'
            return report
        candidates, metrics = [], []
        for yaw in np.deg2rad(np.arange(-180, 180, 5)):
            poses = np.column_stack((positions, np.full(len(positions), yaw)))
            scores = self._scores(poses, train, interior)
            top = np.argsort(scores[:, 0])[-15:]
            candidates.extend(poses[top]); metrics.extend(scores[top])
        candidates, metrics = np.asarray(candidates), np.asarray(metrics)
        seeds = []
        for index in np.argsort(metrics[:, 0])[::-1]:
            pose = candidates[index]
            if all(self._different(pose, previous) for previous in seeds):
                seeds.append(pose)
            if len(seeds) == 18:
                break
        delta = np.meshgrid(np.arange(-.20, .201, .025), np.arange(-.20, .201, .025),
                            np.deg2rad(np.arange(-5., 5.01, .5)), indexing='ij')
        offsets = np.column_stack([part.ravel() for part in delta])
        refined = []
        for seed in seeds:
            poses = offsets+seed
            scores = self._scores(poses, train, interior)
            best = int(np.argmax(scores[:, 0]))
            refined.append((poses[best], scores[best]))
        refined.sort(key=lambda value: -value[1][0])
        distinct = []
        for pose, score in refined:
            if all(self._different(pose, previous[0]) for previous in distinct):
                distinct.append((pose, score))
        if not distinct:
            report['reason'] = 'no free-space candidate in global search'
            return report
        report['distinct_candidates'] = [self._record(pose, score) for pose, score in distinct]
        pose, train_score = distinct[0]
        held_score = self._scores([pose], held, held_interior)[0]
        margin = float(train_score[0]-(distinct[1][1][0] if len(distinct) > 1 else 0.))
        report.update(best=self._record(pose, train_score), held_out=self._record(pose, held_score),
                      runner_up_margin=margin, training_rays=len(train), held_out_rays=len(held),
                      elapsed_s=time.monotonic()-started,
                      coarse_free_positions=len(positions), refined_seeds=len(seeds))
        def quality(metric):
            return (metric[0] >= .58 and metric[1] >= self.min_inlier_ratio
                    and metric[2] <= .06+1e-7 and metric[3] <= .10)
        if not quality(train_score):
            report['reason'] = ('best global candidate fails scan/map quality '
                                f'({self.min_inlier_ratio:.0%} inliers, median <=6 cm, score >=0.58, crossings <=10%)')
        elif not quality(held_score):
            report['reason'] = 'held-out LiDAR beams reject the global candidate'
        elif margin < .10:
            report['reason'] = 'global scan match is ambiguous: sampled alternative has similar score'
        else:
            report.update(accepted=True, reason='global scan match and held-out beams accepted; physical uniqueness is unproven')
        return report
