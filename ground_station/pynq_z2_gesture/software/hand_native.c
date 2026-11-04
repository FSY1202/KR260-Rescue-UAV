/* Fused float32 geometry operations for the unchanged wrist_lobes_v3 rules.
 * Build: gcc -O3 -shared -fPIC -ffp-contract=off -o _hand_geometry.so hand_native.c
 * Deliberately no fast-math: match NumPy's ordered float32 operations.
 */
#include <stdint.h>

int hand_native_version(void) { return 2; }

static void border_xy(int index,int w,int h,int *x,int *y) {
    if(index<w) { *x=index;*y=0;return; } index-=w;
    if(index<h-1) { *x=w-1;*y=index+1;return; } index-=h-1;
    if(index<w-1) { *x=w-2-index;*y=h-1;return; } index-=w-1;
    *x=0;*y=h-2-index;
}
static int border_value(const uint8_t *src,int index,int w,int h) {
    int x,y;border_xy(index,w,h,&x,&y);return src[y*w+x]!=0;
}

int hand_contact(const uint8_t *src,int w,int h,double *out) {
    int size=2*w+2*h-4,shift=-1;
    for(int i=0;i<size;++i)if(!border_value(src,i,w,h)){shift=i;break;}
    if(shift<0)return 0;
    int length=0,best=0;uint64_t sx=0,sy=0,bx=0,by=0;
    for(int i=0;i<=size;++i) {
        int index=(shift+i)%size;
        if(i<size && border_value(src,index,w,h)) {
            int x,y;border_xy(index,w,h,&x,&y);sx+=x;sy+=y;++length;
        } else {
            if(length>best){best=length;bx=sx;by=sy;}
            length=0;sx=sy=0;
        }
    }
    if(best){out[0]=(double)bx/best;out[1]=(double)by/best;}
    return best;
}

void hand_border_support(const float *tips,int count,const uint8_t *src,
                         int w,int h,double maximum,uint8_t *out) {
    int size=2*w+2*h-4;
    for(int n=0;n<count;++n) {
        float tx=tips[2*n],ty=tips[2*n+1];out[n]=1;
        if(tx>0 && tx<w-1 && ty>0 && ty<h-1)continue;
        double best=1e300;int nearest=0;
        for(int i=0;i<size;++i) {
            int x,y;border_xy(i,w,h,&x,&y);
            double dx=(double)x-tx,dy=(double)y-ty,distance=dx*dx+dy*dy;
            if(distance<best){best=distance;nearest=i;}
        }
        if(!border_value(src,nearest,w,h)){out[n]=0;continue;}
        int length=1;
        for(int i=1;i<size;++i) {
            int index=nearest-i;if(index<0)index+=size;
            if(!border_value(src,index,w,h))break;
            ++length;if(length>maximum)break;
        }
        if(length>=size){out[n]=0;continue;}
        if(length<=maximum)for(int i=1;i<size;++i) {
            int index=(nearest+i)%size;
            if(!border_value(src,index,w,h))break;
            ++length;if(length>maximum)break;
        }
        out[n]=length<=maximum;
    }
}

int hand_peak(const float *d, int stride, int w, int h, float threshold,
              double *out) {
    uint64_t sx=0, sy=0, count=0;
    for (int y=0; y<h; ++y)
        for (int x=0; x<w; ++x)
            if (d[y*stride+x] >= threshold) { sx+=x; sy+=y; ++count; }
    if (!count) return 0;
    out[0]=(double)sx/count; out[1]=(double)sy/count;
    return 1;
}

void hand_anchor(const float *d, int stride, int w, int h,
                 float cx, float cy, float threshold, float *out) {
    float best=-1; int bx=0, by=0;
    for (int y=0; y<h; ++y) {
        float dy=(float)y-cy; float y2=dy*dy;
        for (int x=0; x<w; ++x) {
            if (d[y*stride+x] < threshold) continue;
            float dx=(float)x-cx; float square=dx*dx+y2;
            if (square > best) { best=square; bx=x; by=y; }
        }
    }
    out[0]=(float)bx; out[1]=(float)by;
    float dx=(float)bx-cx, dy=(float)by-cy;
    out[2]=dx*dx+dy*dy;
}

int hand_local_peak(const float *d, int stride, int w, int h,
                    int ax, int ay, float neighbourhood2, double *out) {
    float radius=0;
    for (int y=0; y<h; ++y) {
        float dy=(float)(y-ay); float y2=dy*dy;
        for (int x=0; x<w; ++x) {
            float dx=(float)(x-ax);
            if (dx*dx+y2 <= neighbourhood2 && d[y*stride+x] > radius)
                radius=d[y*stride+x];
        }
    }
    float threshold=(float)((double)radius*.98);
    uint64_t sx=0,sy=0,count=0;
    for (int y=0; y<h; ++y) {
        float dy=(float)(y-ay); float y2=dy*dy;
        for (int x=0; x<w; ++x) {
            float dx=(float)(x-ax);
            if (dx*dx+y2 <= neighbourhood2 && d[y*stride+x] >= threshold) {
                sx+=x; sy+=y; ++count;
            }
        }
    }
    out[0]=radius;
    if (!count) return 0;
    out[1]=(double)sx/count; out[2]=(double)sy/count;
    return 1;
}

void hand_cut(const uint8_t *src, int w, int h, float px, float py,
              float dx, float dy, float threshold, uint8_t *out) {
    for (int y=0; y<h; ++y) {
        float yr=((float)y-py)*dy;
        for (int x=0; x<w; ++x) {
            float projection=((float)x-px)*dx+yr;
            out[y*w+x]=projection > threshold ? 0 : src[y*w+x];
        }
    }
}

void hand_outer(const uint8_t *src, int w, int h, float px, float py,
                float threshold, float *radial, uint8_t *outer) {
    for (int y=0; y<h; ++y) {
        float dy=(float)y-py; float y2=dy*dy;
        for (int x=0; x<w; ++x) {
            float dx=(float)x-px; float square=dx*dx+y2;
            radial[y*w+x]=square;
            outer[y*w+x]=(src[y*w+x] != 0 && square >= threshold);
        }
    }
}
