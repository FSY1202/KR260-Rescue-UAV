"""Two completion-tracked X11 shared images and fused YUYV-to-BGRA scaling."""
import ctypes as ct
from pathlib import Path
import numpy as np


class FastSurface:
    def __init__(self, source_width=640, source_height=480, fullscreen=True,
                 target_width=640,target_height=480):
        self.lib=ct.CDLL(str(Path(__file__).with_name('_live_x11.so')))
        ptr,integer=ct.c_void_p,ct.c_int
        self.lib.live_open.argtypes=[integer]*5;self.lib.live_open.restype=ptr
        self.lib.live_close.argtypes=[ptr]
        self.lib.live_poll.argtypes=[ptr]
        self.lib.live_acquire.argtypes=[ptr]
        self.lib.live_pixels.argtypes=[ptr,integer];self.lib.live_pixels.restype=ptr
        self.lib.live_stride.argtypes=[ptr]
        self.lib.live_convert.argtypes=[ptr,integer,ptr,integer]
        self.lib.live_present.argtypes=[ptr,integer]
        self.source=(source_height,source_width)
        self.target=(target_height,target_width)
        self.handle=self.lib.live_open(source_width,source_height,target_width,target_height,int(fullscreen))
        if not self.handle:raise RuntimeError('Cannot open X11 shared video surface')
        stride=self.lib.live_stride(self.handle)
        self.images=[]
        for index in range(2):
            address=self.lib.live_pixels(self.handle,index)
            memory=(ct.c_uint8*(stride*target_height)).from_address(address)
            self.images.append(np.ndarray((target_height,target_width,4),np.uint8,buffer=memory,strides=(stride,4,1)))

    def acquire(self):return self.lib.live_acquire(self.handle)

    def convert(self, index, raw):
        if raw.dtype!=np.uint8 or raw.shape!=(*self.source,2) or not raw.flags.c_contiguous:
            raise ValueError('Packed YUYV shape does not match video surface')
        rows=self.lib.live_convert(self.handle,index,raw.ctypes.data,raw.strides[0])
        if rows!=self.target[0]:raise RuntimeError('Video converter returned an incomplete frame')
        return self.images[index]

    def present(self,index):
        if not self.lib.live_present(self.handle,index):raise RuntimeError('Shared video presentation failed')

    def closed(self):return bool(self.lib.live_poll(self.handle))

    def close(self):
        if self.handle:
            self.images=[]
            self.lib.live_close(self.handle);self.handle=None
