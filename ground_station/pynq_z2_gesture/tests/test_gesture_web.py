"""An older status poll cannot supply the labels for a newer JPEG."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'software'))
from gesture_web import GestureWeb


class GestureWebTest(unittest.TestCase):
    def test_local_display_needs_no_jpeg_until_an_http_image_request(self):
        with tempfile.TemporaryDirectory() as directory:
            status=Path(directory)/'status.json';renders=[]
            def renderer(raw,clean,contour,label,backend,fps,frame_id):
                renders.append(frame_id)
                return np.full((24,32,3),frame_id,np.uint8)
            preview=GestureWeb('127.0.0.1',0,status,renderer,lazy_jpeg=True)
            base='http://127.0.0.1:'+str(preview.port)
            raw=np.full((24,32,2),19,np.uint8)
            try:
                preview.publish(raw,None,'FIST','pl',30,1)
                deadline=time.monotonic()+3
                while True:
                    try:
                        with urlopen(base+'/raw.yuyv',timeout=1) as response:
                            self.assertEqual(response.read(),raw.tobytes());break
                    except HTTPError as error:
                        if error.code!=503 or time.monotonic()>deadline:raise
                        time.sleep(.01)
                self.assertEqual(renders,[])
                with self.assertRaises(HTTPError) as error:urlopen(base+'/frame.jpg',timeout=1)
                self.assertEqual(error.exception.code,503)
                preview.publish(raw,None,'V','pl',30,2)
                deadline=time.monotonic()+3
                while True:
                    try:
                        with urlopen(base+'/frame.jpg',timeout=1) as response:
                            self.assertEqual(response.headers['X-Frame-Id'],'2')
                            self.assertEqual(response.headers['X-Gesture'],'V');break
                    except HTTPError as error:
                        if error.code!=503 or time.monotonic()>deadline:raise
                        time.sleep(.01)
                self.assertEqual(renders,[2])
            finally:preview.close()

    def test_image_and_labels_use_one_published_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            status = Path(directory) / 'status.json'
            status.write_text('{"status":"running","gesture":"OPEN"}', encoding='utf-8')
            def renderer(raw, clean, contour, label, backend, fps, frame_id):
                return np.full((24, 32, 3), frame_id, np.uint8)
            preview = GestureWeb('127.0.0.1', 0, status, renderer)
            base = 'http://127.0.0.1:'+str(preview.port)
            def wait_frame(frame_id):
                deadline = time.monotonic()+3
                while time.monotonic() < deadline:
                    try:
                        with urlopen(base+'/frame.jpg', timeout=2) as response:
                            body, headers = response.read(), response.headers
                        if headers['X-Frame-Id'] == str(frame_id):
                            return body, headers
                    except HTTPError as exc:
                        if exc.code != 503:
                            raise
                    time.sleep(.01)
                self.fail('Preview did not publish the requested frame')
            try:
                raw = np.zeros((24, 32, 2), np.uint8)
                preview.publish(raw, None, 'FIST', 'pl', 29.1, 40, raw_label='UNKNOWN')
                body, headers = wait_frame(40)
                self.assertEqual(headers['X-Gesture'], 'FIST')
                self.assertEqual(headers['X-Raw-Gesture'], 'UNKNOWN')
                self.assertAlmostEqual(float(headers['X-Processing-Fps']), 29.1)
                self.assertAlmostEqual(float(cv2.imdecode(np.frombuffer(body, np.uint8), 1).mean()), 40, delta=1)
                status.write_text(json.dumps({'status':'running', 'gesture':'OPEN',
                                              'raw_gesture':'OPEN', 'frame':1}), encoding='utf-8')
                with urlopen(base+'/status.json', timeout=2) as response:
                    self.assertEqual(json.load(response)['gesture'], 'OPEN')
                _, headers = wait_frame(40)
                self.assertEqual(headers['X-Gesture'], 'FIST')
                # A new image replaces its metadata, rather than carrying the
                # preceding image's label or the delayed status-file label.
                preview.publish(raw, None, 'V', 'pl', 28.8, 80)
                body, headers = wait_frame(80)
                self.assertEqual(headers['X-Gesture'], 'V')
                self.assertEqual(headers['X-Raw-Gesture'], 'V')
                self.assertAlmostEqual(float(cv2.imdecode(np.frombuffer(body, np.uint8), 1).mean()), 80, delta=1)
            finally:
                preview.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
