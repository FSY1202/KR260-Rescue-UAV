import http.server
import os
import json
import threading
import time
import unittest
from unittest.mock import patch
import urllib.request
import urllib.error
import cv2
import numpy as np
from ncnn_preview import Preview,handler_for,select_preview_profile,source_metadata


class PreviewProfileTest(unittest.TestCase):
    def test_mission_profile_pins_320_single_pass(self):
        with patch.dict(os.environ, {'KR260_VISION_PROFILE':'mission'}):
            self.assertEqual(select_preview_profile(True),(1,320,.5))

    def test_default_preview_preserves_original_profile_and_interval(self):
        self.assertFalse(Preview().fast_preview)
        with patch('ncnn_preview.inference_budget',return_value=(2,640)) as budget:
            self.assertEqual(select_preview_profile(),(2,640,.5))
            budget.assert_called_once_with()

    def test_fast_preview_uses_one_384_pass_with_shorter_interval(self):
        self.assertTrue(Preview(fast_preview=True).fast_preview)
        with patch('ncnn_preview.inference_budget',return_value=(2,640)):
            self.assertEqual(select_preview_profile(True),(1,384,.2))

    def test_live_lease_always_preserves_its_profile_and_interval(self):
        for size in (320,384,480,640):
            for fast in (False,True):
                with self.subTest(size=size,fast=fast), \
                     patch('ncnn_preview.inference_budget',return_value=(1,size)):
                    self.assertEqual(select_preview_profile(fast),(1,size,.5))

    def test_profile_is_rechecked_when_lease_changes(self):
        with patch('ncnn_preview.inference_budget',side_effect=[(2,640),(1,320),(2,640)]):
            self.assertEqual(select_preview_profile(True),(1,384,.2))
            self.assertEqual(select_preview_profile(True),(1,320,.5))
            self.assertEqual(select_preview_profile(True),(1,384,.2))


class SourceMetadataTest(unittest.TestCase):
    def test_valid_source_frame_identity_is_preserved(self):
        stamp=time.monotonic_ns()-1000
        metadata=source_metadata({'X-Camera-Session':'a'*32,'X-Camera-Frame-Seq':'3',
                                  'X-Camera-Monotonic-Ns':str(stamp)})
        self.assertEqual(metadata,dict(source_camera_session_id='a'*32,source_frame_seq=3,
                                       source_frame_monotonic_ns=stamp,source_metadata_error=None))

    def test_legacy_source_does_not_invent_frame_identity(self):
        metadata=source_metadata({})
        self.assertIsNone(metadata['source_camera_session_id'])
        self.assertIsNone(metadata['source_frame_seq'])
        self.assertIsNone(metadata['source_frame_monotonic_ns'])
        self.assertIsNotNone(metadata['source_metadata_error'])


class HttpFreshnessTest(unittest.TestCase):
    def test_only_fresh_images_are_served(self):
        preview=Preview()
        preview.info.update(state='READY',count=1,detections=[{'label':'person'}])
        preview.received=time.monotonic()
        preview.jpeg=cv2.imencode('.jpg',np.zeros((8,8,3),dtype=np.uint8))[1].tobytes()
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),handler_for(preview))
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        address='http://127.0.0.1:'+str(server.server_port)
        try:
            with urllib.request.urlopen(address+'/snapshot.jpg',timeout=2) as response:
                self.assertTrue(response.read().startswith(b'\xff\xd8'))
            preview.received=time.monotonic()-6
            with urllib.request.urlopen(address+'/status.json',timeout=2) as response:
                stale=json.load(response)
            self.assertEqual(stale['state'],'STALE')
            self.assertIsNone(stale['count'])
            self.assertEqual(stale['detections'],[])
            with self.assertRaises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(address+'/snapshot.jpg',timeout=2)
            self.assertEqual(error.exception.code,503)
            preview.info.update(state='ERROR',count=None,detections=[])
            preview.jpeg=None
            with self.assertRaises(urllib.error.HTTPError):
                urllib.request.urlopen(address+'/snapshot.jpg',timeout=2)
        finally:
            server.shutdown();server.server_close();worker.join(timeout=2)


if __name__=='__main__': unittest.main()
