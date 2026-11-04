"""BTN0-controlled HDMI viewer of the latest local YUYV frame."""
import argparse
from collections import deque
import json
from pathlib import Path
import subprocess
import sys
import signal
import time
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'pynq_z2_gesture/software'))
from local_preview import LocalReader
from fast_surface import FastSurface

NAMES={'FIST':'FIST / ROCK','OPEN':'OPEN / PALM','V':'V / SCISSORS',
       'NONE':'NO HAND','UNKNOWN':'ADJUST HAND'}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--button-control',type=Path)
    parser.add_argument('--shared-frame',type=Path,default=Path('/run/pynq-desktop/camera-live.frame'))
    args=parser.parse_args()
    cv2.setNumThreads(1)
    visible=args.button_control is None
    requested=[None]
    signal.signal(signal.SIGUSR1,lambda *_:requested.__setitem__(0,True))
    signal.signal(signal.SIGUSR2,lambda *_:requested.__setitem__(0,False))
    sequence=0
    selection_path=Path('/run/pynq-desktop/view-selection.json')
    selection_token=None
    reader=None
    surface=None
    window=False
    last_token=None
    last_received=0
    next_open=next_button=next_wake=next_status=next_wait=0
    title='Gesture Recognition'
    rendered=deque()
    display_total=0
    timings=deque(maxlen=60)
    source_frame=None
    frame_age_ms=processing_fps=0
    status=Path('/run/pynq-xfce/camera-control-status.json')
    if args.button_control:
        try:
            previous=json.loads(status.read_text())
            sequence=int(previous['sequence']);visible=bool(previous['visible'])
        except (OSError,ValueError,KeyError):pass

    def rate(now):
        if len(rendered)<2 or now-rendered[-1]>1:return 0.0
        return (len(rendered)-1)/(rendered[-1]-rendered[0])

    def record(log=False):
        now=time.perf_counter()
        data={'visible':visible,'sequence':sequence,'transport':'shared_yuyv_neon_xshm',
              'display_fps':round(rate(now),2) if visible else 0.0,'processing_fps':round(processing_fps,2),
              'source_frame':source_frame,'frame_age_ms':round(frame_age_ms,2)}
        data.update(display_frames=display_total,updated_monotonic=now)
        if timings:
            data['stages_ms']={key:round(sum(row[key] for row in timings)/len(timings),2) for key in timings[0]}
        if args.button_control:
            temporary=status.with_suffix('.tmp')
            temporary.write_text(json.dumps(data));temporary.replace(status)
        if log:print('CAMERA_VIEW '+('SHOW' if visible else 'HIDE')+' '+json.dumps(data),flush=True)

    record(True)
    try:
        while True:
            now=time.perf_counter()
            if requested[0] is not None:
                visible=requested[0];requested[0]=None;record(True)
            if args.button_control and now>=next_button:
                try:
                    selection=json.loads(selection_path.read_text())
                    token=(selection['revision'],selection['view'])
                    if token!=selection_token:
                        visible=selection['view']=='camera'
                        selection_token=token
                        record(True)
                except (OSError,ValueError,KeyError):selection=None
                try:new_sequence=int(json.loads(args.button_control.read_text())['sequence'])
                except (OSError,ValueError,KeyError):new_sequence=sequence
                if selection is None:
                    if new_sequence<sequence:visible=False
                    elif (new_sequence-sequence)%2:visible=not visible
                if new_sequence!=sequence:
                    sequence=new_sequence;record(True)
                next_button=now+.02
            if not visible:
                if window:
                    surface.close();surface=None;window=False
                rendered.clear()
                time.sleep(.02)
                continue
            if not window:
                surface=FastSurface(fullscreen=args.button_control is not None)
                window=True;last_token=None;rendered.clear();next_wake=next_wait=0
            if now>=next_wake:
                for command in (['/usr/bin/xset','s','reset'],['/usr/bin/xset','dpms','force','on']):
                    subprocess.run(command,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)
                next_wake=now+60
            index=surface.acquire()
            if index==-2:
                if not args.button_control:break
                visible=False;record(True);continue
            if index<0:
                time.sleep(.002);continue
            if reader is None and now>=next_open:
                try:reader=LocalReader(args.shared_frame)
                except (OSError,ValueError):next_open=now+.5
            packet=None
            read_start=time.perf_counter()
            if reader is not None:
                try:packet=reader.read(last_token,zero_copy=True)
                except (OSError,ValueError):
                    reader.close();reader=None;next_open=now+.5
            read_end=time.perf_counter()
            frame=None
            convert_end=overlay_end=read_end
            if packet is not None:
                raw,contour,metadata,last_token=packet
                if now-metadata['acquired']<3:
                    source_frame=metadata['frame_id'];processing_fps=metadata['fps']
                    try:frame=surface.convert(index,raw)
                    finally:reader.release()
                    convert_end=time.perf_counter()
                    label=metadata['label']
                    color=(0,220,0,255) if label in ('FIST','OPEN','V') else (0,210,255,255)
                    cv2.rectangle(frame,(128,48),(511,431),(0,255,0,255),1)
                    if contour is not None:
                        cv2.drawContours(frame,[contour*2],-1,color,1,offset=(128,48))
                    cv2.rectangle(frame,(0,0),(639,43),(25,25,25,255),-1)
                    cv2.putText(frame,NAMES[label],(9,29),cv2.FONT_HERSHEY_SIMPLEX,.75,color,2)
                    cv2.rectangle(frame,(0,449),(639,479),(25,25,25,255),-1)
                    cv2.putText(frame,f'PL {processing_fps:.1f} FPS | VIEW {rate(now):.1f} FPS | 640x480',
                                (8,470),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255,255),1)
                    overlay_end=time.perf_counter()
                    last_received=now
                    frame_age_ms=(time.perf_counter()-metadata['acquired'])*1000
                else:reader.release()
                raw=None
            if frame is None and now-last_received>3 and now>=next_wait:
                frame=surface.images[index];frame[:]=0
                cv2.putText(frame,'Waiting for camera...',(100,240),0,1,(255,255,255,255),1)
                next_wait=now+.2
            if frame is not None:
                show_start=time.perf_counter()
                surface.present(index)
                show_end=time.perf_counter()
            else:show_start=show_end=time.perf_counter()
            closed=surface.closed()
            finished=time.perf_counter()
            if packet is not None and frame is not None and finished-last_received<3:
                rendered.append(finished)
                display_total+=1
                timings.append(dict(read_ms=(read_end-read_start)*1000,convert_ms=(convert_end-read_end)*1000,
                                    overlay_ms=(overlay_end-convert_end)*1000,present_ms=(show_end-show_start)*1000,
                                    events_ms=(finished-show_end)*1000))
                while len(rendered)>2 and rendered[1]<finished-1:rendered.popleft()
            if finished>=next_status:
                record();next_status=finished+1
            if closed:
                if not args.button_control:break
                visible=False;record(True)
            if frame is None:time.sleep(.002)
            packet=None
    finally:
        if reader is not None:reader.close()
        if surface is not None:surface.close()


if __name__=='__main__':main()
