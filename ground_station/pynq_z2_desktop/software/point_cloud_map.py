"""Offline 2-D scan-style renderer. No sensor, network, or height data."""
import json
import math
from pathlib import Path

import cv2
import numpy as np

WIDTH, HEIGHT = 800, 600


class Polyline:
    """Arc-length timing, preserving every corner and gate crossing."""
    def __init__(self, points, duration):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise ValueError('Invalid route')
        keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9]
        self.points = points[keep]
        if len(self.points) < 2 or not math.isfinite(duration) or duration <= 0:
            raise ValueError('Route needs movement and positive duration')
        self.duration = float(duration)
        self.lengths = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.distances = np.r_[0., np.cumsum(self.lengths)]
        self.total = self.distances[-1]

    def locate(self, elapsed):
        distance = np.clip(elapsed/self.duration, 0., 1.)*self.total
        index = min(len(self.lengths)-1,
                    int(np.searchsorted(self.distances, distance, side='right')-1))
        alpha = (distance-self.distances[index])/self.lengths[index]
        point = self.points[index]+alpha*(self.points[index+1]-self.points[index])
        delta = self.points[index+1]-self.points[index]
        return point, math.degrees(math.atan2(delta[1], delta[0])), index

    def at(self, elapsed):
        point, angle, _ = self.locate(elapsed)
        return float(point[0]), float(point[1]), angle

    def trail(self, elapsed):
        point, _, index = self.locate(elapsed)
        return np.rint(np.vstack([self.points[:index+1], point])).astype(np.int32)


def transform_reference(points):
    """First screenshot -> display; same uniform transform for map and route."""
    points = np.asarray(points, dtype=np.float64)
    scale = .53
    return (points-[750., 490.])*scale+[400., 301.]


class PointCloudMap:
    def __init__(self, filename, path, radius=220.):
        with np.load(filename, allow_pickle=False) as data:
            self.points = data['points'].astype(np.float32)
            intensity = data['intensity'].astype(np.float32)
            self.kinds = data['kinds'].astype(np.uint8)
            self.metadata = json.loads(str(data['metadata'].item()))
        n = len(self.points)
        if (not 0 < n <= 12000 or self.points.shape != (n, 2)
                or intensity.shape != (n,) or self.kinds.shape != (n,)
                or not np.isfinite(self.points).all() or not np.isfinite(intensity).all()
                or np.any(self.points < 0) or np.any(self.points > [WIDTH-1, HEIGHT-1])
                or np.any(intensity < 0) or np.any(intensity > 1)):
            raise ValueError('Invalid preset point cloud')
        self.xy = np.rint(self.points).astype(np.int32)
        self.indices = self.xy[:, 1]*WIDTH+self.xy[:, 0]
        # Gate returns are amber; cartons and boundary returns cyan/white.
        palette = np.array([[175, 147, 101], [207, 187, 144], [68, 166, 225]], np.float32)
        if np.any(self.kinds >= len(palette)):
            raise ValueError('Unknown map feature kind')
        self.colors = np.clip(palette[self.kinds]*intensity[:, None], 0, 255).astype(np.uint8)
        self.reveal = np.full(n, np.inf, dtype=np.float32)
        self.scan_radius = radius
        # Only N-element temporary arrays, never an N x 601 distance matrix.
        for t in np.linspace(0., path.duration, int(round(path.duration*10))+1):
            x, y, _ = path.at(t)
            delta = self.points-[x, y]
            hit = np.einsum('ij,ij->i', delta, delta) <= radius*radius
            self.reveal[hit] = np.minimum(self.reveal[hit], t)
        if not np.isfinite(self.reveal).all():
            raise ValueError('Preset route does not scan the complete map')
        self.order = np.argsort(self.reveal, kind='stable')
        self.times = self.reveal[self.order]
        # Deduplicate display pixels once. NumPy ufunc.at is slow on ARM;
        # retain source return counts, but paint unique pixels with direct stores.
        pixel_indices, inverse = np.unique(self.indices, return_inverse=True)
        pixel_times = np.full(len(pixel_indices), np.inf, np.float32)
        pixel_colors = np.zeros((len(pixel_indices), 3), np.uint8)
        np.minimum.at(pixel_times, inverse, self.reveal)
        for channel in range(3):
            np.maximum.at(pixel_colors[:, channel], inverse, self.colors[:, channel])
        pixel_order = np.argsort(pixel_times, kind='stable')
        self.pixel_indices = pixel_indices[pixel_order]
        self.pixel_times = pixel_times[pixel_order]
        self.pixel_colors = pixel_colors[pixel_order]
        self.fresh_colors = np.minimum(self.pixel_colors.astype(np.uint16)+55, 255).astype(np.uint8)
        self.gates = transform_reference(np.array(self.metadata['gate_centers']))
        self.gate_times = []
        for center in self.gates:
            times = [t for t in np.linspace(0., path.duration, int(round(path.duration*10))+1)
                     if np.linalg.norm(np.array(path.at(t)[:2])-center) <= radius]
            self.gate_times.append(min(times))
        self.base = np.empty((HEIGHT, WIDTH, 4), np.uint8)
        self.base[:] = (16, 12, 7, 255)
        for x in range(40, WIDTH, 40):
            cv2.line(self.base, (x, 38), (x, 565), (25, 22, 14, 255), 1)
        for y in range(45, 566, 40):
            cv2.line(self.base, (18, y), (782, y), (25, 22, 14, 255), 1)
        self.display = self.base.copy()
        self.count = 0
        self.pixel_count = 0
        self.last_tick = -1

    def draw(self, frame, elapsed, path):
        tick = int(math.floor(min(path.duration, max(0., elapsed))*10+1e-5))
        if tick < self.last_tick:
            self.display[:] = self.base
            self.count = 0
            self.pixel_count = 0
        if tick != self.last_tick:
            end = int(np.searchsorted(self.times, tick/10.+1e-5, side='right'))
            pixel_end = int(np.searchsorted(self.pixel_times, tick/10.+1e-5, side='right'))
            self.count = end
            self.pixel_count = pixel_end
            self.last_tick = tick
            # Refresh only return pixels, not another complete 1.9-MB image.
            # Full-frame memory copies alone take ~13 ms on this board.
            self.display.reshape(-1, 4)[self.pixel_indices[:pixel_end], :3] = self.pixel_colors[:pixel_end]
            if tick/10. < path.duration:
                fresh_start = np.searchsorted(self.pixel_times, max(0., tick/10.-.65), side='left')
                fresh = slice(fresh_start, pixel_end)
                self.display.reshape(-1, 4)[self.pixel_indices[fresh], :3] = self.fresh_colors[fresh]
            draw_cloud_hud(self.display, tick/10., path.duration, self.count, len(self.points))
        np.copyto(frame, self.display)
        # Draw annotations on the fresh frame, never into the return cache.
        # This prevents history-dependent edges after rewind/hidden updates.
        for i, (center, when) in enumerate(zip(self.gates, self.gate_times), 1):
            if tick/10.+1e-5 >= when:
                p = tuple(np.rint(center).astype(int))
                cv2.drawMarker(frame, p, (73, 184, 242, 255), cv2.MARKER_CROSS, 10, 1)
                cv2.putText(frame, 'G'+str(i), (p[0]+9, p[1]-7), 0, .35,
                            (103, 192, 232, 255), 1, cv2.LINE_8)
        if elapsed < path.duration:
            x, y, _ = path.at(elapsed)
            # Subtle local acquisition pulse, a visual effect rather than sensor physics.
            cv2.circle(frame, (round(x), round(y)), 24+int((elapsed % 1.5)*18),
                       (67, 54, 27, 255), 1, cv2.LINE_AA)
        return self.count


def draw_quadcopter(frame, x, y, heading, scale=1.):
    """Four rotor rings, X arms, front (+x) marker, top-down projection."""
    x, y = round(x), round(y)
    angle = math.radians(heading)
    co, si = math.cos(angle), math.sin(angle)

    def rotate(px, py):
        return (round(x+scale*(co*px-si*py)), round(y+scale*(si*px+co*py)))

    cv2.circle(frame, (x, y), round(20*scale), (21, 18, 10, 255), -1)
    for dx, dy in ((-10, -10), (-10, 10), (10, -10), (10, 10)):
        center = rotate(dx, dy)
        color = (88, 187, 255, 255) if dx > 0 else (226, 222, 153, 255)
        cv2.line(frame, (x, y), center, (225, 229, 218, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, center, max(3, round(5*scale)), color, 1, cv2.LINE_AA)
        cv2.circle(frame, center, 1, color, -1)
    body = np.array([rotate(-5, -4), rotate(5, -4), rotate(5, 4), rotate(-5, 4)])
    cv2.fillConvexPoly(frame, body, (231, 236, 225, 255), cv2.LINE_AA)
    nose = np.array([rotate(11, 0), rotate(5, -3), rotate(5, 3)])
    cv2.fillConvexPoly(frame, nose, (74, 177, 255, 255), cv2.LINE_AA)


def draw_route(frame, path, elapsed):
    trail = path.trail(elapsed)
    cv2.polylines(frame, [trail], False, (82, 224, 205, 255), 2, cv2.LINE_AA)
    start, finish = np.rint(path.points[[0, -1]]).astype(int)
    cv2.circle(frame, tuple(start), 3, (143, 245, 145, 255), 1, cv2.LINE_AA)
    cv2.circle(frame, tuple(finish), 3, (91, 184, 242, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, 'START', (start[0]-48, start[1]+45), 0, .35,
                (143, 245, 145, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, 'END', (finish[0]+9, finish[1]+45), 0, .35,
                (91, 184, 242, 255), 1, cv2.LINE_AA)


def draw_cloud_hud(frame, elapsed, duration, visible_count, total):
    cv2.rectangle(frame, (0, 0), (799, 36), (21, 17, 10, 255), -1)
    cv2.putText(frame, 'PRESET LIDAR MAP', (22, 24), 0, .6, (238, 220, 173, 255), 1, cv2.LINE_AA)
    state = 'COMPLETE' if elapsed >= duration else 'SCANNING'
    cv2.putText(frame, state, (624, 24),
                0, .48, (128, 228, 185, 255), 1, cv2.LINE_AA)
    cv2.rectangle(frame, (0, 568), (799, 599), (21, 17, 10, 255), -1)
    cv2.putText(frame, f'2D DEMO   |   RETURNS {visible_count:05d}/{total:05d}',
                (22, 588), 0, .4, (178, 172, 135, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, 'BOXES  /  GATES  /  QUAD', (506, 588), 0, .4,
                (128, 189, 211, 255), 1, cv2.LINE_AA)
