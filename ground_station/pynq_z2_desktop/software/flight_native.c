/* Fixed-point nearest-neighbour attitude blit. No device/control interfaces. */
#include <stdint.h>
#include <stddef.h>

int flight_warp(uint8_t *destination, int stride, const uint32_t *source,
                int width, int height, const int32_t *m) {
    if (!destination || !source || !m || stride < 412*4 || width != 640 || height != 640)
        return 0;
    for (int y=0; y<280; ++y) {
        uint32_t *out=(uint32_t*)(destination+(size_t)y*stride);
        int32_t sx=m[1]*y+m[2], sy=m[4]*y+m[5];
        for (int x=0; x<412; ++x) {
            int ix=(sx+32768)>>16, iy=(sy+32768)>>16;
            if (ix<0) ix=0; else if(ix>=width) ix=width-1;
            if (iy<0) iy=0; else if(iy>=height) iy=height-1;
            out[x]=source[iy*width+ix];
            sx+=m[0]; sy+=m[3];
        }
    }
    return 280;
}
