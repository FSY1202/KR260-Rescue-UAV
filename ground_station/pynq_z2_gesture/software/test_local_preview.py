import os
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np


@unittest.skipUnless(os.name=='posix','Linux shared frame transport')
class LocalPreviewTests(unittest.TestCase):
    def test_pixels_labels_contours_and_empty_contour(self):
        from local_preview import LocalWriter,LocalReader
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'frame'
            writer=LocalWriter(path);reader=LocalReader(path)
            try:
                self.assertIsNone(reader.read())
                raw=np.full((480,640,2),57,np.uint8)
                contour=np.array([[[3,4]],[[8,9]],[[13,14]]],np.int32)
                self.assertTrue(writer.publish(raw,contour,'FIST',29.7,14,123.5))
                image,actual,metadata,token=reader.read()
                np.testing.assert_array_equal(image,raw)
                np.testing.assert_array_equal(actual,contour)
                self.assertEqual(metadata,dict(label='FIST',fps=29.7,frame_id=14,acquired=123.5))
                self.assertIsNone(reader.read(token))
                raw[:]=190
                self.assertTrue(writer.publish(raw,None,'NONE',30,15,124))
                image,actual,metadata,_=reader.read(token)
                self.assertIsNone(actual)
                self.assertEqual(metadata['label'],'NONE')
                self.assertTrue(np.all(image==190))
            finally:reader.close();writer.close()

    def test_slow_reader_never_blocks_inference(self):
        import fcntl
        from local_preview import LocalWriter,LocalReader
        with tempfile.TemporaryDirectory() as directory:
            writer=LocalWriter(Path(directory)/'frame');reader=LocalReader(Path(directory)/'frame')
            try:
                raw=np.zeros((480,640,2),np.uint8)
                for fd,_ in reader.slots:fcntl.flock(fd,fcntl.LOCK_SH)
                start=time.monotonic()
                self.assertFalse(writer.publish(raw,None,'NONE',30,1,1))
                self.assertLess(time.monotonic()-start,.1)
                for fd,_ in reader.slots:fcntl.flock(fd,fcntl.LOCK_UN)
                self.assertTrue(writer.publish(raw,None,'OPEN',30,2,2))
                self.assertEqual(reader.read()[2]['frame_id'],2)
                self.assertEqual(writer.skipped,1)
            finally:reader.close();writer.close()

    def test_bad_header_rejected(self):
        from local_preview import LocalWriter,LocalReader,HEADER,MAGIC
        with tempfile.TemporaryDirectory() as directory:
            writer=LocalWriter(Path(directory)/'frame');reader=LocalReader(Path(directory)/'frame')
            try:
                writer.slots[0][1][:HEADER.size]=HEADER.pack(MAGIC,640,480,0,0,30,1,1,1)
                writer.mapping[4]=0
                with self.assertRaises(ValueError):reader.read()
            finally:reader.close();writer.close()

    def test_zero_copy_lease_leaves_other_slots_available(self):
        from local_preview import LocalWriter,LocalReader
        with tempfile.TemporaryDirectory() as directory:
            writer=LocalWriter(Path(directory)/'frame');reader=LocalReader(Path(directory)/'frame')
            leased=None
            try:
                raw=np.full((480,640,2),19,np.uint8)
                writer.publish(raw,None,'FIST',30,1,1)
                leased=reader.read(zero_copy=True)
                for index in range(2,12):
                    raw[:]=index
                    self.assertTrue(writer.publish(raw,None,'OPEN',30,index,index))
                    self.assertTrue(np.all(leased[0]==19))
                reader.release();leased=None
                self.assertEqual(reader.read()[2]['frame_id'],11)
                self.assertEqual(writer.skipped,0)
            finally:
                reader.release();leased=None;reader.close();writer.close()


if __name__=='__main__':unittest.main()
