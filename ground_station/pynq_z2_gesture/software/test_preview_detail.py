"""Ensure the preview preserves camera detail and analysis overlay coordinates."""
import unittest
import cv2
import numpy as np
from gesture_demo import picture


class PreviewDetailTests(unittest.TestCase):
    def test_native_640_detail_is_not_downsampled(self):
        raw = np.full((480, 640, 2), 128, np.uint8)
        # Alternating one-pixel luminance stripes vanish if reduced to 320.
        raw[:, ::2, 0] = 32
        raw[:, 1::2, 0] = 210
        expected = cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUY2)
        rendered = picture(raw, None, None, 'NONE', 'pl', 20, 1, False)
        self.assertEqual(rendered.shape, (480, 640, 3))
        np.testing.assert_array_equal(rendered[100:140, 200:300],
                                      expected[100:140, 200:300])

    def test_roi_and_contour_use_scaled_analysis_coordinates(self):
        raw = np.full((480, 640, 2), 128, np.uint8)
        contour = np.array([[[10, 10]], [[30, 10]], [[10, 30]]], np.int32)
        rendered = picture(raw, None, contour, 'UNKNOWN', 'pl', 20, 1, False)
        np.testing.assert_array_equal(rendered[48, 128], [0, 255, 0])
        np.testing.assert_array_equal(rendered[68, 148], [0, 210, 255])


if __name__ == '__main__':
    unittest.main()
