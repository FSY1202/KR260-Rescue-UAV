"""One bounded local YUYV frame, protected by nonblocking process locks.

The capture/inference owner never waits for a slow display. Shared locks keep
pixels, frame number, contour and label from being combined across frames.
"""
import fcntl
import mmap
import os
from pathlib import Path
import struct
import numpy as np

HEADER = struct.Struct('<4sIIIIddQI')
META_SIZE = 65536
PIXELS = HEADER.size+META_SIZE
SLOT_SIZE = PIXELS+640*480*2
FILE_SIZE = 64
MAGIC = b'LGF3'
LABELS = ('NONE', 'UNKNOWN', 'FIST', 'OPEN', 'V')


class LocalWriter:
    def __init__(self, path):
        path = Path(path)
        self.fd = os.open(path, os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW, 0o644)
        os.fchmod(self.fd, 0o644)
        os.ftruncate(self.fd, FILE_SIZE)
        self.mapping = mmap.mmap(self.fd, FILE_SIZE)
        self.mapping[:5]=MAGIC+b'\xff'
        self.slots=[]
        for index in range(3):
            fd=os.open(str(path)+'.'+str(index),os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o644)
            os.fchmod(fd,0o644);os.ftruncate(fd,SLOT_SIZE)
            mapping=mmap.mmap(fd,SLOT_SIZE)
            mapping[:HEADER.size]=b'\0'*HEADER.size
            self.slots.append((fd,mapping))
        self.current=2
        self.published = self.skipped = 0

    def publish(self, raw, contour, label, fps, frame_id, acquired):
        h,w,c = raw.shape
        if raw.dtype != np.uint8 or c != 2 or not raw.flags.c_contiguous or raw.nbytes > SLOT_SIZE-PIXELS:
            raise ValueError('Local preview requires packed YUYV up to 640x480')
        points = (np.empty((0,1,2),np.int32) if contour is None
                  else np.ascontiguousarray(contour,dtype=np.int32))
        if points.ndim != 3 or points.shape[1:] != (1,2) or points.nbytes > META_SIZE:
            raise ValueError('Local contour metadata exceeds its bounded slot')
        code = LABELS.index(label)
        selected=None
        for step in range(1,4):
            index=(self.current+step)%3
            fd,mapping=self.slots[index]
            try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:continue
            selected=index;break
        if selected is None:
            self.skipped += 1
            return False
        try:
            mapping[PIXELS:PIXELS+raw.nbytes] = memoryview(raw).cast('B')
            if points.nbytes:
                mapping[HEADER.size:HEADER.size+points.nbytes] = memoryview(points).cast('B')
            mapping[:HEADER.size] = HEADER.pack(MAGIC,w,h,len(points),code,
                                                     float(fps),float(acquired),int(frame_id),raw.nbytes)
            # Publish a one-byte slot index while its exclusive lock is held.
            # The reader's shared lock acquires both pixels and metadata.
            self.mapping[4]=selected
            self.current=selected
            self.published += 1
            return True
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)

    def close(self):
        for fd,mapping in self.slots:mapping.close();os.close(fd)
        self.mapping.close()
        os.close(self.fd)


class LocalReader:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDONLY|os.O_NOFOLLOW)
        if os.fstat(self.fd).st_size != FILE_SIZE:
            os.close(self.fd)
            raise ValueError('Incompatible local preview size')
        self.mapping = mmap.mmap(self.fd, FILE_SIZE, access=mmap.ACCESS_READ)
        self.slots=[]
        self.lease=None
        try:
            for index in range(3):
                fd=os.open(str(path)+'.'+str(index),os.O_RDONLY|os.O_NOFOLLOW)
                if os.fstat(fd).st_size!=SLOT_SIZE:
                    os.close(fd);raise ValueError('Incompatible local frame slot size')
                self.slots.append((fd,mmap.mmap(fd,SLOT_SIZE,access=mmap.ACCESS_READ)))
        except Exception:
            self.close();raise

    def release(self):
        if self.lease is not None:
            fcntl.flock(self.slots[self.lease][0],fcntl.LOCK_UN);self.lease=None

    def read(self, last_frame=None, zero_copy=False):
        self.release()
        if self.mapping[:4]!=MAGIC:return None
        index=self.mapping[4]
        if index>=3:return None
        fd,mapping=self.slots[index]
        try:
            fcntl.flock(fd, fcntl.LOCK_SH|fcntl.LOCK_NB)
        except BlockingIOError:
            return None
        try:
            magic,w,h,npoint,code,fps,acquired,frame_id,nraw = HEADER.unpack(mapping[:HEADER.size])
            if magic != MAGIC:
                return None
            if not (npoint*8 <= META_SIZE and code < len(LABELS)
                    and 0 < w <= 640 and 0 < h <= 480 and nraw == w*h*2):
                raise ValueError('Invalid local frame header')
            metadata = {'label':LABELS[code], 'fps':fps, 'frame_id':frame_id,'acquired':acquired}
            token = (frame_id,acquired)
            if token == last_frame:
                return None
            raw = np.frombuffer(mapping, np.uint8, count=nraw, offset=PIXELS).reshape(h,w,2)
            if not zero_copy:raw=raw.copy()
            contour = (np.frombuffer(mapping,np.int32,count=npoint*2,
                                     offset=HEADER.size).reshape(npoint,1,2).copy() if npoint else None)
            if zero_copy:self.lease=index
            return raw, contour, metadata, token
        finally:
            if self.lease is None:fcntl.flock(fd, fcntl.LOCK_UN)

    def close(self):
        self.release()
        for fd,mapping in self.slots:
            try:mapping.close()
            except BufferError:pass # An outstanding NumPy lease defers unmapping until its last view dies.
            os.close(fd)
        self.slots=[]
        self.mapping.close()
        os.close(self.fd)
