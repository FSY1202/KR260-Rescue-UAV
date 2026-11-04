"""Local, dependency-free HTTP preview for the existing gesture pipeline.

Only this process owns the camera and PL. HTTP requests consume immutable JPEG
bytes; slow browsers cannot hold the inference loop or the image encoder.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import threading
import time
from urllib.parse import urlsplit
import uuid

import cv2


PAGE = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PYNQ-Z2 手势识别</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101823;color:#eaf0f7;font:16px system-ui,"Microsoft YaHei",sans-serif}
main{max-width:980px;margin:28px auto;padding:0 18px}h1{font-size:25px;margin-bottom:7px}
p{color:#adbdd1;line-height:1.7}#state{display:inline-block;padding:8px 13px;border-radius:7px;background:#34445c;margin:10px 0}
.screen{position:relative;width:100%;max-width:640px;aspect-ratio:4/3;background:#060b12;border:1px solid #384b62;border-radius:8px;overflow:hidden}
#camera{width:100%;height:100%;object-fit:contain;display:none}#overlay{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;background:#101823b8;text-align:center;padding:22px}
.cards{display:flex;flex-wrap:wrap;gap:12px;margin:18px 0}.card{min-width:140px;background:#1d2a3c;padding:14px 18px;border-radius:8px}.card small{display:block;color:#a8bad0;margin-bottom:7px}.card strong{font-size:23px}
button{color:#eaf0f7;background:#253b57;border:1px solid #4e6580;padding:8px 14px;border-radius:5px;cursor:pointer}#detail{font-size:14px;overflow-wrap:anywhere}a{color:#9acaff}
</style></head><body><main>
<h1>PYNQ-Z2 手势识别</h1>
<p>把单只手放进绿色框内，可识别拳头、张开手掌和剪刀。画面由板子本地处理。</p>
<div id="state" role="status">正在连接板子…</div>
<div class="screen" id="screen"><img id="camera" alt="摄像头实时画面"><div id="overlay">等待摄像头画面…</div></div>
<p><button id="size">切换为 320 像素宽</button></p>
<div class="cards"><div class="card"><small>原始识别</small><strong id="raw">—</strong></div><div class="card"><small>稳定识别</small><strong id="stable">—</strong></div><div class="card"><small>处理速度</small><strong id="fps">—</strong></div><div class="card"><small>网页帧号</small><strong id="frame">—</strong></div></div>
<p id="detail">等待本次识别会话启动。</p>
<p>网页画面最多约 5 帧/秒刷新，识别仍对每个处理帧进行。无手显示“无手”，形状不明确显示“无法确认”。</p>
</main><script>
const byId=id=>document.getElementById(id);
const labels={FIST:'拳头',OPEN:'张开手掌',V:'剪刀',NONE:'无手',UNKNOWN:'无法确认'};
let disconnected=false, haveFrame=false, imageBusy=false, imageFailed=false, stopped=false;
let small=false;
byId('size').onclick=()=>{small=!small;byId('screen').style.maxWidth=small?'320px':'640px';byId('size').textContent=small?'切换为 640 像素宽':'切换为 320 像素宽';};
function overlay(text){byId('overlay').textContent=text;byId('overlay').style.display=text?'flex':'none';}
function state(text,color){byId('state').textContent=text;byId('state').style.background=color;}
async function statusLoop(){
 try{
  const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),3000);
  let response;try{response=await fetch('/status.json?t='+Date.now(),{cache:'no-store',signal:controller.signal});}finally{clearTimeout(timer);}
  if(!response.ok)throw Error('状态 HTTP '+response.status);
  const data=await response.json();disconnected=false;
  const web=data.web||{};stopped=data.status==='stopped';
  // Once an image is present, its own response supplies the displayed labels.
  // A slower status poll must not replace them with an earlier frame's result.
  if(!haveFrame){
   byId('raw').textContent=labels[data.raw_gesture]||'—';byId('stable').textContent=labels[data.gesture]||'—';
   byId('fps').textContent=typeof data.fps==='number'?data.fps.toFixed(1)+' FPS':'—';
   byId('frame').textContent=web.frame_id==null?'—':web.frame_id;
  }
  if(data.status==='failed'||web.error){state('识别或预览失败','#8c303b');overlay('程序发生错误，请查看下方信息。');}
  else if(stopped){state('识别已停止','#705322');overlay('识别程序已停止，显示的是最后一帧。');}
  else if(web.state==='stalled'){state('画面暂停更新','#705322');overlay('摄像头画面暂未更新，请检查板子运行状态。');}
  else if(web.state==='waiting'||!haveFrame){state('等待摄像头画面','#34445c');overlay('识别正在启动，等待摄像头画面…');}
  else if(imageFailed){state('画面暂不可用','#705322');overlay('正在重新获取摄像头画面…');}
  else{state('识别运行中','#236044');overlay('');}
  byId('detail').textContent=data.error||web.error||('后端：'+(data.backend||'—').toUpperCase()+'；本次已处理 '+(data.frame||0)+' 帧；丢帧 '+(data.dropped||0)+'；网页状态：'+(web.state||'waiting')+'。');
 }catch(error){disconnected=true;state('与板子的连接已断开','#8c303b');overlay('无法连接板子，正在自动重试…');byId('detail').textContent=error.message;}
 setTimeout(statusLoop,1000);
}
async function imageLoop(){
 if(!imageBusy){imageBusy=true;let objectURL;
  try{const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),3000);
   let response;try{response=await fetch('/frame.jpg?t='+Date.now(),{cache:'no-store',signal:controller.signal});if(!response.ok)throw Error('画面 HTTP '+response.status);const blob=await response.blob();objectURL=URL.createObjectURL(blob);}finally{clearTimeout(timer);}
   const decoded=new Image();decoded.src=objectURL;await decoded.decode();
   const img=byId('camera');const previous=img.dataset.url;img.src=objectURL;img.dataset.url=objectURL;img.style.display='block';if(previous)URL.revokeObjectURL(previous);
   const rawLabel=response.headers.get('X-Raw-Gesture'), stableLabel=response.headers.get('X-Gesture');
   const frame=response.headers.get('X-Frame-Id'), fps=Number(response.headers.get('X-Processing-Fps'));
   byId('raw').textContent=labels[rawLabel]||'—';byId('stable').textContent=labels[stableLabel]||'—';
   byId('frame').textContent=frame||'—';byId('fps').textContent=Number.isFinite(fps)?fps.toFixed(1)+' FPS':'—';
   img.dataset.frameId=frame||'';img.dataset.gesture=stableLabel||'';
   haveFrame=true;imageFailed=false;
  }catch(error){imageFailed=true;if(objectURL)URL.revokeObjectURL(objectURL);}
  finally{imageBusy=false;}
 }
 setTimeout(imageLoop,200);
}
statusLoop();imageLoop();
</script></body></html>'''


class _PreviewServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    allow_reuse_address = True

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(3)
        return connection, address


class _Handler(BaseHTTPRequestHandler):
    # Finite requests keep disconnected clients from accumulating indefinitely.
    protocol_version = 'HTTP/1.0'

    def log_message(self, *_):
        pass

    def do_GET(self):
        self._serve(False)

    def do_HEAD(self):
        self._serve(True)

    def _serve(self, head_only):
        preview = self.server.preview
        path = urlsplit(self.path).path
        code, content_type = 200, 'application/json; charset=utf-8'
        retry = False
        frame_id = None
        frame_labels = None
        raw_size = None
        if path == '/':
            body, content_type = PAGE.encode('utf-8'), 'text/html; charset=utf-8'
        elif path == '/frame.jpg':
            with preview._condition:
                preview._jpeg_requested_at=time.monotonic()
                body = preview._jpeg
                frame_id = preview._frame_id
                frame_labels = preview._frame_labels
            if body is None:
                code, retry = 503, True
                body = b'{"status":"waiting","message":"No preview frame available"}'
            else:
                content_type = 'image/jpeg'
        elif path == '/mask.png':
            with preview._condition:
                clean = None if preview._clean is None else preview._clean.copy()
                frame_id = preview._frame_id
            if clean is None:
                code, retry = 503, True
                body = b'{"status":"waiting","message":"No matching mask available"}'
            else:
                try:
                    ok, encoded = cv2.imencode('.png', clean)
                    if not ok:
                        raise RuntimeError('PNG encoder returned no image')
                    body, content_type = encoded.tobytes(), 'image/png'
                except Exception:
                    code, body = 500, b'{"error":"Cannot encode mask"}'
        elif path == '/raw.yuyv':
            with preview._condition:
                raw = preview._raw
                frame_id = preview._frame_id
            if raw is None:
                code, retry = 503, True
                body = b'{"status":"waiting","message":"No matching raw frame available"}'
            else:
                # publish already owns this array; retain its reference while a
                # new preview frame is encoded. Copy bytes only on this request.
                body, content_type = raw.tobytes(), 'application/octet-stream'
                raw_size = (raw.shape[1], raw.shape[0])
        elif path == '/status.json':
            body = json.dumps(preview.status(), ensure_ascii=False).encode('utf-8')
        elif path == '/health':
            with preview._condition:
                health = {'status': 'ok', 'session_id': preview.session_id,
                          'encoder_error': preview._error,
                          'encoded_frames': preview._encoded}
            body = json.dumps(health).encode('utf-8')
        else:
            code, body = 404, b'{"error":"Not found"}'
        try:
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store, max-age=0')
            self.send_header('X-Content-Type-Options', 'nosniff')
            if frame_id is not None and code == 200:
                self.send_header('X-Frame-Id', str(frame_id))
            if frame_labels is not None and code == 200:
                self.send_header('X-Raw-Gesture', frame_labels[0])
                self.send_header('X-Gesture', frame_labels[1])
                self.send_header('X-Processing-Fps', str(frame_labels[2]))
            if raw_size is not None and code == 200:
                self.send_header('X-Width', str(raw_size[0]))
                self.send_header('X-Height', str(raw_size[1]))
                self.send_header('X-Pixel-Format', 'YUYV')
            if retry:
                self.send_header('Retry-After', '1')
            self.end_headers()
            if not head_only:
                self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError):
            # Browser disconnects do not affect the camera/PL owner.
            pass


class GestureWeb:
    """One latest pending frame, one encoder, and a bounded immutable JPEG cache.

    ``renderer`` has the same signature as gesture_demo.picture. Call publish
    at the desired preview rate, independently of the inference frame rate.
    """
    def __init__(self, host, port, status_file, renderer, jpeg_quality=75, lazy_jpeg=False):
        if not 1 <= int(jpeg_quality) <= 100:
            raise ValueError('JPEG quality must be 1..100')
        self.status_file = Path(status_file)
        self.renderer = renderer
        self.jpeg_quality = int(jpeg_quality)
        self.lazy_jpeg=bool(lazy_jpeg)
        self._jpeg_requested_at=float('-inf')
        self.session_id = uuid.uuid4().hex
        self._condition = threading.Condition()
        self._pending = None
        self._jpeg = None
        self._clean = None
        self._raw = None
        self._frame_id = None
        self._frame_labels = None
        self._encoded_at = None
        self._error = None
        self._encoded = 0
        self._replaced = 0
        self._closed = False
        self._baseline = self._signature()
        self._server = _PreviewServer((host, int(port)), _Handler)
        self._server.preview = self
        self.host, self.port = self._server.server_address[:2]
        self._encoder = threading.Thread(target=self._encode, name='gesture-jpeg', daemon=True)
        self._http = threading.Thread(target=self._server.serve_forever,
                                      kwargs={'poll_interval': .1}, name='gesture-http', daemon=True)
        try:
            self._encoder.start()
            self._http.start()
        except Exception:
            with self._condition:
                self._closed = True
                self._condition.notify_all()
            self._server.server_close()
            raise

    def _signature(self):
        try:
            stat = self.status_file.stat()
            return stat.st_dev, stat.st_ino, stat.st_mtime_ns, stat.st_size
        except OSError:
            return None

    def publish(self, raw, contour, label, backend, fps, frame_id, clean=None, raw_label=None):
        # No renderer, encoder, file operation, or socket operation runs here.
        # Explicit copies keep Capture and DMA buffers out of another thread.
        packet = (raw.copy(), None if contour is None else contour.copy(),
                  label, backend, float(fps), int(frame_id),
                  None if clean is None else clean.copy(), str(raw_label or label))
        with self._condition:
            if self._closed:
                return
            if self._pending is not None:
                self._replaced += 1
            self._pending = packet
            self._condition.notify()

    def _encode(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                raw, contour, label, backend, fps, frame_id, clean, raw_label = self._pending
                self._pending = None
                want_jpeg=not self.lazy_jpeg or time.monotonic()-self._jpeg_requested_at<3
            try:
                jpeg=None
                if want_jpeg:
                    image = self.renderer(raw, None, contour, label, backend, fps, frame_id)
                    ok, encoded = cv2.imencode('.jpg', image,
                                               [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                    if not ok:
                        raise RuntimeError('JPEG encoder returned no image')
                    jpeg = encoded.tobytes()
                with self._condition:
                    self._jpeg = jpeg
                    self._clean = clean
                    self._raw = raw
                    self._frame_id = frame_id
                    self._frame_labels = (raw_label, label, fps)
                    self._encoded_at = time.monotonic()
                    self._encoded += int(jpeg is not None)
                    self._error = None
            except Exception as exc:
                with self._condition:
                    self._error = '{}: {}'.format(type(exc).__name__, exc)
                    self._jpeg = None
                    self._clean = None
                    self._raw = None

    def status(self):
        # A prior run's persistent "running" record cannot represent this run.
        # The recognizer must replace/update the status file after construction.
        record = {'status': 'starting', 'raw_gesture': 'UNKNOWN', 'gesture': 'UNKNOWN'}
        signature = self._signature()
        if signature is not None and signature != self._baseline:
            try:
                candidate = json.loads(self.status_file.read_text(encoding='utf-8'))
                if isinstance(candidate, dict):
                    record = candidate
            except (OSError, ValueError):
                record['status_message'] = 'Waiting for a complete status update'
        with self._condition:
            age = None if self._encoded_at is None else time.monotonic() - self._encoded_at
            state = ('stopped' if self._closed else 'failed' if self._error else
                     'waiting' if age is None else 'stalled' if age > 3 else 'streaming')
            record['web'] = {'session_id': self.session_id, 'state': state,
                             'frame_id': self._frame_id,
                             'frame_age_seconds': None if age is None else round(age, 3),
                             'encoded_frames': self._encoded, 'replaced_pending': self._replaced,
                             'error': self._error}
        return record

    def close(self):
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._pending = None
            self._condition.notify_all()
        self._server.shutdown()
        self._server.server_close()
        self._http.join(timeout=3)
        self._encoder.join(timeout=3)
        if self._encoder.is_alive():
            raise RuntimeError('Preview encoder did not stop')
