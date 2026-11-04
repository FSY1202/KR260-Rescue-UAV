"""Frame bytes and metadata must be atomically copied from the same frame."""
import http.server
from pathlib import Path
import sys
import threading
import time
import types
import unittest
import urllib.request

try:
    import fcntl
except ImportError:
    sys.modules['fcntl']=types.ModuleType('fcntl')
try:
    import pwd
except ImportError:
    sys.modules['pwd']=types.ModuleType('pwd')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'deploy'))
from camera_preview import Preview, handler_for


class Metadata(unittest.TestCase):
    def test_source_headers_and_stale_frame(self):
        preview=Preview(None,None,3)
        stamp=time.monotonic_ns()-1000000
        preview.raw=b'frame-bytes';preview.updated=stamp/1e9
        preview.info={'frame_seq':42,'capture_completed_monotonic_ns':stamp}
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),handler_for(preview))
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        try:
            url='http://127.0.0.1:'+str(server.server_port)+'/raw.jpg'
            with urllib.request.urlopen(url,timeout=1) as response:
                self.assertEqual(response.read(),b'frame-bytes')
                self.assertEqual(response.headers['X-Camera-Session'],preview.session)
                self.assertEqual(response.headers['X-Camera-Frame-Seq'],'42')
                self.assertEqual(response.headers['X-Camera-Monotonic-Ns'],str(stamp))
            preview.updated=time.monotonic()-6
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(url,timeout=1)
            self.assertEqual(caught.exception.code,503)
        finally:
            server.shutdown();server.server_close();worker.join(timeout=1)


if __name__=='__main__':unittest.main()
