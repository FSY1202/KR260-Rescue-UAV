"""Regression cases for wrist notches and shadow holes seen by the real camera."""
import json
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'software'))
from classify_hand import classify_clean


def fixtures():
    fist = np.zeros((192, 192), np.uint8)
    cv2.circle(fist, (96, 70), 32, 255, -1)
    cv2.rectangle(fist, (78, 83), (114, 191), 255, -1)
    bent = np.zeros_like(fist)
    cv2.ellipse(bent, (90, 75), (36, 37), 0, 0, 360, 255, -1)
    cv2.fillPoly(bent, [np.array([[68, 88], [110, 86], [118, 191],
        [58, 191], [66, 130], [63, 110]], np.int32)], 255)
    cv2.circle(bent, (60, 96), 12, 255, -1)
    cv2.rectangle(bent, (0, 181), (70, 191), 255, -1)
    shadow = fist.copy()
    cv2.ellipse(shadow, (96, 74), (20, 10), -12, 0, 360, 0, -1)
    opened = np.zeros_like(fist)
    cv2.rectangle(opened, (35, 85), (135, 145), 255, -1)
    for x, top in [(35, 45), (57, 20), (79, 10), (101, 20), (123, 45)]:
        cv2.rectangle(opened, (x, top), (x + 12, 100), 255, -1)
    scissors = np.zeros_like(fist)
    cv2.fillPoly(scissors, [np.array([[65, 145], [115, 145], [110, 80],
        [145, 30], [128, 20], [92, 68], [60, 20], [42, 30], [68, 80]], np.int32)], 255)
    for mask, label, name in [(fist, 'FIST', 'straight_wrist'),
                              (bent, 'FIST', 'bent_wrist_background'),
                              (shadow, 'FIST', 'palm_shadow_hole'),
                              (opened, 'OPEN', 'open'),
                              (scissors, 'V', 'scissors')]:
        yield mask, label, name
    opened_wrist = opened.copy()
    cv2.rectangle(opened_wrist, (68, 140), (104, 191), 255, -1)
    scissors_wrist = scissors.copy()
    cv2.rectangle(scissors_wrist, (73, 140), (107, 191), 255, -1)
    yield opened_wrist, 'OPEN', 'open_wrist'
    yield scissors_wrist, 'V', 'scissors_wrist'


class HandClassifierTest(unittest.TestCase):
    def test_three_labels_with_wrist_in_four_directions(self):
        for mask, expected, name in fixtures():
            for rotation in range(4):
                with self.subTest(name=name, rotation=rotation):
                    info, _ = classify_clean(np.ascontiguousarray(np.rot90(mask, rotation)))
                    self.assertEqual(info['gesture'], expected, info)
                    json.dumps(info)  # Status must never contain NumPy scalars.

    def test_actual_camera_failures_and_other_gestures(self):
        directory = Path(__file__).with_name('fixtures')
        labels = json.loads((directory / 'labels.json').read_text(encoding='utf-8'))
        for sample in labels:
            mask = cv2.imread(str(directory / sample['file']), cv2.IMREAD_GRAYSCALE)
            self.assertIsNotNone(mask)
            for rotation in range(4):
                with self.subTest(file=sample['file'], rotation=rotation):
                    info, _ = classify_clean(np.ascontiguousarray(np.rot90(mask, rotation)))
                    if sample['partially_clipped'] or sample.get('allow_unknown', False):
                        self.assertIn(info['gesture'], ('FIST', 'UNKNOWN'), info)
                    else:
                        self.assertEqual(info['gesture'], sample['expected'], info)

    def test_three_fingers_are_not_open_hand(self):
        three = np.zeros((192, 192), np.uint8)
        cv2.rectangle(three, (25, 100), (160, 155), 255, -1)
        for x, top in [(25, 30), (85, 10), (145, 35)]:
            cv2.rectangle(three, (x, top), (x+15, 110), 255, -1)
        wrist = three.copy()
        cv2.rectangle(wrist, (75, 140), (110, 191), 255, -1)
        for mask in (three, wrist):
            for rotation in range(4):
                with self.subTest(rotation=rotation):
                    info, _ = classify_clean(np.ascontiguousarray(np.rot90(mask, rotation)))
                    self.assertEqual(info['gesture'], 'UNKNOWN', info)

    def test_empty_full_and_thin_object(self):
        empty = np.zeros((192, 192), np.uint8)
        self.assertEqual(classify_clean(empty)[0]['gesture'], 'NONE')
        self.assertEqual(classify_clean(np.full_like(empty, 255))[0]['gesture'], 'UNKNOWN')
        bar = empty.copy()
        cv2.rectangle(bar, (87, 15), (103, 175), 255, -1)
        self.assertEqual(classify_clean(bar)[0]['gesture'], 'UNKNOWN')

    def test_single_extended_finger_is_not_fist(self):
        mask = np.zeros((192, 192), np.uint8)
        cv2.rectangle(mask, (60, 100), (130, 155), 255, -1)
        cv2.rectangle(mask, (86, 15), (104, 110), 255, -1)
        wrist = mask.copy()
        cv2.rectangle(wrist, (78, 140), (112, 191), 255, -1)
        for silhouette in (mask, wrist):
            for rotation in range(4):
                info, _ = classify_clean(np.ascontiguousarray(np.rot90(silhouette, rotation)))
                self.assertEqual(info['gesture'], 'UNKNOWN', info)


if __name__ == '__main__':
    unittest.main(verbosity=2)
