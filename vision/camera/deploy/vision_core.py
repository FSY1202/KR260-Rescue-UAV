"""RGB565 to BGR conversion for the DVP/PL frame capture interface."""
import numpy as np


def rgb565_to_bgr(pixels):
    if pixels.ndim != 2 or pixels.dtype != np.dtype('uint16'):
        raise ValueError('Expected a two-dimensional uint16 RGB565 image')
    blue = pixels & 31
    green = (pixels >> 5) & 63
    red = (pixels >> 11) & 31
    return np.stack(((blue << 3) | (blue >> 2),
                     (green << 2) | (green >> 4),
                     (red << 3) | (red >> 2)), axis=-1).astype(np.uint8)

