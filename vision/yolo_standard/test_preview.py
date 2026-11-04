import http.server
import json
import threading
import time
import unittest
import urllib.request
import urllib.error
import cv2
import numpy as np
from ncnn_preview import Preview,handler_for


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
