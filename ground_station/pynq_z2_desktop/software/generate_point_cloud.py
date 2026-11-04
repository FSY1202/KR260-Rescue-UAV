"""Reproduce the synthetic 2-D map from the supplied reference screenshots."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from point_cloud_map import transform_reference

SEED = 20261103


def generate(reference_file, route_file):
    raw = Path(reference_file).read_bytes()
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    other = cv2.imdecode(np.frombuffer(Path(route_file).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    mask = cv2.inRange(image, (0, 0, 0), (85, 85, 85))
    other_mask = cv2.inRange(other, (0, 0, 0), (85, 85, 85))
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    _, _, other_stats, _ = cv2.connectedComponentsWithStats(other_mask)
    selected = sorted([tuple(map(int, r)) for r in stats[1:] if r[4] > 100],
                      key=lambda row: (row[1], row[0]))
    other_selected = sorted([tuple(map(int, r)) for r in other_stats[1:] if r[4] > 100],
                            key=lambda row: (row[1], row[0]))
    if len(selected) != 15 or len(other_selected) != 15:
        raise ValueError('Reference features changed')
    for a, b in zip(selected, other_selected):
        if not (a[0]-b[0] == 188 and a[1]-b[1] == 28 and a[2:] == b[2:]):
            raise ValueError('Reference screenshots do not align at (188,28)')
    feature_kinds = np.zeros(len(stats), np.uint8)
    boxes, gates = [], []
    for index, (x, y, w, h, area) in enumerate(stats):
        if index == 0 or area <= 100:
            continue
        if w > 150 and 20 < h < 60:
            feature_kinds[index] = 2
            gates.append([x+w/2, y+h/2])
        elif w > 30 and h > 20:
            feature_kinds[index] = 1
            boxes.append([int(x), int(y), int(w), int(h)])
    if len(boxes) != 8 or len(gates) != 2:
        raise ValueError('Expected eight cartons and two pass-through gates')
    gates.sort(key=lambda center: center[1])
    ys, xs = np.where(mask != 0)
    rng = np.random.default_rng(SEED)
    choose = rng.choice(len(xs), size=min(12000, len(xs)), replace=False)
    points = transform_reference(np.column_stack([xs[choose], ys[choose]]))
    points += rng.normal(0, .65, points.shape)
    kinds = feature_kinds[labels[ys[choose], xs[choose]]]
    intensity = rng.uniform(.38, 1., len(points)).astype(np.float32)
    metadata = dict(format='PRESET_2D_CLOUD_1', seed=SEED, map_size=[800, 600],
                    coordinate_system='display_pixels', synthetic=True,
                    reference_sha256=hashlib.sha256(raw).hexdigest(),
                    alignment_second_to_first=[188, 28], cartons=boxes,
                    gate_centers=gates, scan_radius_pixels=220)
    # Display-only illustration derived from features; no measured flight route.
    left = min(row[0] for row in selected) + 70
    right = max(row[0] + row[2] for row in selected) - 70
    top = min(row[1] for row in selected) + 20
    bottom = max(row[1] + row[3] for row in selected) - 20
    middle = (top + bottom) / 2
    g1, g2 = gates
    illustration = [[left, bottom], [left, middle],
                    [(left+g1[0])/2, middle], [(left+g1[0])/2, g1[1]+60],
                    [g1[0], g1[1]+60], [g1[0], top],
                    [g2[0], top], [g2[0], g2[1]+60],
                    [right, g2[1]+60], [right, bottom]]
    route = transform_reference(np.array(illustration))
    return dict(points=points.astype(np.float32), intensity=intensity, kinds=kinds,
                metadata=np.array(json.dumps(metadata))), dict(
        description='Preset 2-D quadcopter route; bottom start, through both gate centers',
        duration_seconds=60, coordinate_system='pixel', map_size=[800, 600],
        interpolation='arc_length_polyline', illustration_generated=True,
        alignment_second_to_first=[188, 28], points=route.tolist())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, default=Path(__file__).resolve().parents[1]/'data')
    args = parser.parse_args()
    asset, route = generate(args.data/'reference_map.png', args.data/'reference_route.png')
    np.savez_compressed(args.data/'map_cloud.npz', **asset)
    (args.data/'trajectory.json').write_text(json.dumps(route, indent=2), encoding='utf-8')
    print('Generated', len(asset['points']), '2-D points; eight cartons, two gates, aligned route')
