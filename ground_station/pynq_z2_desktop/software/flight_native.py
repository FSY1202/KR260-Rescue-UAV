"""Optional ARM raster blitter, with OpenCV nearest-neighbour reference fallback."""
import ctypes as ct
from pathlib import Path
import cv2
import numpy as np


class HorizonBlitter:
    def __init__(self):
        self.lib = None
        self.backend = 'opencv_nearest'
        path = Path(__file__).with_name('_flight_instruments.so')
        if path.exists():
            try:
                lib=ct.CDLL(str(path))
                lib.flight_warp.argtypes=[ct.c_void_p,ct.c_int,ct.c_void_p,
                                         ct.c_int,ct.c_int,ct.c_void_p]
                lib.flight_warp.restype=ct.c_int
                self.lib=lib
                self.backend='native_fixed_point'
            except (OSError,AttributeError):
                pass

    def draw(self, frame, texture, matrix):
        if (frame.shape != (280,412,4) or texture.shape != (640,640,4)
                or frame.dtype != np.uint8 or texture.dtype != np.uint8
                or frame.strides[1:] != (4,1) or not texture.flags.c_contiguous):
            raise ValueError('Invalid attitude raster surface')
        if self.lib is None:
            cv2.warpAffine(texture,matrix,(412,280),dst=frame,
                           flags=cv2.INTER_NEAREST,borderMode=cv2.BORDER_REPLICATE)
        else:
            inverse=cv2.invertAffineTransform(matrix)
            fixed=np.rint(inverse*65536).astype(np.int32).ravel()
            result=self.lib.flight_warp(frame.ctypes.data,frame.strides[0],texture.ctypes.data,
                                         640,640,fixed.ctypes.data)
            if result != 280:
                raise RuntimeError('Attitude raster returned an incomplete frame')
