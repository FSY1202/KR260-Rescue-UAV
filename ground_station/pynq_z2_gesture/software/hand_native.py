"""Optional native geometry, with a portable NumPy fallback in classify_hand."""
import ctypes as ct
import os
from pathlib import Path
import numpy as np


class NativeGeometry:
    def __init__(self, path):
        self.lib = ct.CDLL(str(path))
        if self.lib.hand_native_version() != 2:
            raise RuntimeError('Incompatible hand geometry ABI')
        ptr, integer, real = ct.c_void_p, ct.c_int, ct.c_float
        self.lib.hand_peak.argtypes = [ptr, integer, integer, integer, real, ptr]
        self.lib.hand_anchor.argtypes = [ptr, integer, integer, integer, real, real, real, ptr]
        self.lib.hand_local_peak.argtypes = [ptr, integer, integer, integer, integer, integer, real, ptr]
        self.lib.hand_cut.argtypes = [ptr, integer, integer, real, real, real, real, real, ptr]
        self.lib.hand_outer.argtypes = [ptr, integer, integer, real, real, real, ptr, ptr]
        self.lib.hand_contact.argtypes = [ptr,integer,integer,ptr]
        self.lib.hand_border_support.argtypes = [ptr,integer,ptr,integer,integer,ct.c_double,ptr]

    def contact(self,component):
        out=np.empty(2,np.float64);h,w=component.shape
        count=self.lib.hand_contact(component.ctypes.data,w,h,out.ctypes.data)
        return count,out.astype(np.float32) if count else None

    def border_support(self,tips,component,maximum):
        tips=np.ascontiguousarray(tips,dtype=np.float32)
        out=np.empty(len(tips),np.uint8);h,w=component.shape
        self.lib.hand_border_support(tips.ctypes.data,len(tips),component.ctypes.data,
                                      w,h,maximum,out.ctypes.data)
        return out.view(np.bool_)

    def peak(self, distance, threshold):
        out = np.empty(2, np.float64)
        h,w = distance.shape
        count = self.lib.hand_peak(distance.ctypes.data, distance.strides[0]//4,
                                   w,h,threshold,out.ctypes.data)
        return out if count else None

    def anchor(self, distance, contact, threshold):
        out = np.empty(3, np.float32)
        h,w = distance.shape
        self.lib.hand_anchor(distance.ctypes.data, distance.strides[0]//4,
                             w,h,float(contact[0]),float(contact[1]),threshold,out.ctypes.data)
        return int(out[1]), int(out[0]), out[2]

    def local_peak(self, distance, ax, ay, neighbourhood2):
        out = np.empty(3, np.float64)
        h,w = distance.shape
        count = self.lib.hand_local_peak(distance.ctypes.data, distance.strides[0]//4,
                                         w,h,ax,ay,neighbourhood2,out.ctypes.data)
        return float(out[0]), out[1:] if count else None

    def cut(self, component, palm, direction, threshold):
        out = np.empty_like(component)
        h,w = component.shape
        self.lib.hand_cut(component.ctypes.data,w,h,float(palm[0]),float(palm[1]),
                           float(direction[0]),float(direction[1]),threshold,out.ctypes.data)
        return out

    def outer(self, silhouette, palm, threshold):
        radial = np.empty(silhouette.shape, np.float32)
        outer = np.empty_like(silhouette)
        h,w = silhouette.shape
        self.lib.hand_outer(silhouette.ctypes.data,w,h,float(palm[0]),float(palm[1]),
                            threshold,radial.ctypes.data,outer.ctypes.data)
        return radial, outer


native = None
library = Path(__file__).with_name('_hand_geometry.so')
if library.is_file() and os.environ.get('GESTURE_NATIVE', '1') != '0':
    try:
        native = NativeGeometry(library)
    except (OSError, RuntimeError):
        pass
