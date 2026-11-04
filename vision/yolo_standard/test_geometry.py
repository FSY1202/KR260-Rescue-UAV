import unittest
import cv2
import numpy as np
from ncnn_detector import unrotate_box, merge_detections


class GeometryTest(unittest.TestCase):
    def test_inverse_uses_actual_rotated_pixels(self):
        for height,width in ((80,100),(480,640),(113,70)):
            mask=np.zeros((height,width),dtype=np.uint8)
            mask[13:56,7:39]=255
            for angle,code in ((0,None),(90,cv2.ROTATE_90_CLOCKWISE),
                               (180,cv2.ROTATE_180),(270,cv2.ROTATE_90_COUNTERCLOCKWISE)):
                rotated=mask if code is None else cv2.rotate(mask,code)
                x,y,w,h=cv2.boundingRect(cv2.findNonZero(rotated))
                self.assertEqual(unrotate_box([x,y,x+w,y+h],width,height,angle),[7,13,39,56])

    def test_multiple_angles_do_not_duplicate_people(self):
        items=[dict(xyxy=[10,20,50,80],confidence=.9,source_angle=0),
               dict(xyxy=[11,21,51,81],confidence=.8,source_angle=90),
               dict(xyxy=[100,100,140,160],confidence=.7,source_angle=270)]
        selected=merge_detections(items,.25)
        self.assertEqual(len(selected),2)
        self.assertEqual(selected[0]['confidence'],.9)
        self.assertEqual(merge_detections([], .25),[])


if __name__=='__main__':
    unittest.main()
