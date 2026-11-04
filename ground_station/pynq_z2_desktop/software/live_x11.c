/* Local video presentation: FFmpeg YUYV conversion/scaling and X11 MIT-SHM.
 * Two buffers are reused only after their XShm completion events arrive.
 * Build with pkg-config --cflags --libs libswscale libavutil x11 xext.
 */
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <X11/extensions/XShm.h>
#include <libswscale/swscale.h>
#include <libavutil/pixfmt.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdint.h>
#include <stdlib.h>
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

struct video_slot { XImage *image; XShmSegmentInfo info; int attached,busy,removed; };
struct live_video {
    Display *display; Window window; GC gc; Atom wm_delete;
    int width,height,source_width,source_height,event_base,closed,offset_x,offset_y;
    struct SwsContext *converter;
    uint32_t *converted;
    int *xmap,*ymap;
    struct video_slot slot[2];
};

void live_close(struct live_video *v) {
    if (!v) return;
    if (v->display) {
        XSync(v->display,False);
        for (int i=0;i<2;++i) {
            struct video_slot *s=&v->slot[i];
            if (s->attached) { XShmDetach(v->display,&s->info); XSync(v->display,False); }
            if (s->image) { s->image->data=NULL; XDestroyImage(s->image); }
            if (s->info.shmaddr!=(char*)-1) shmdt(s->info.shmaddr);
            if (s->info.shmid>=0 && !s->removed) shmctl(s->info.shmid,IPC_RMID,NULL);
        }
        if (v->gc) XFreeGC(v->display,v->gc);
        if (v->window) XDestroyWindow(v->display,v->window);
        XCloseDisplay(v->display);
    }
    if (v->converter) sws_freeContext(v->converter);
    free(v->converted);free(v->xmap);free(v->ymap);
    free(v);
}

struct live_video *live_open(int sw,int sh,int w,int h,int fullscreen) {
    struct live_video *v=calloc(1,sizeof(*v));
    if (!v) return NULL;
    for(int i=0;i<2;++i) { v->slot[i].info.shmid=-1;v->slot[i].info.shmaddr=(char*)-1; }
    v->display=XOpenDisplay(NULL);
    if (!v->display || !XShmQueryExtension(v->display)) goto failed;
    int screen=DefaultScreen(v->display);
    v->width=w;v->height=h;v->source_width=sw;v->source_height=sh;
    int window_width=fullscreen?800:w,window_height=fullscreen?600:h;
    v->offset_x=(window_width-w)/2;v->offset_y=(window_height-h)/2;
    v->window=XCreateSimpleWindow(v->display,RootWindow(v->display,screen),0,0,window_width,window_height,0,0,0);
    XStoreName(v->display,v->window,"Gesture Recognition");
    XSelectInput(v->display,v->window,ExposureMask|KeyPressMask|StructureNotifyMask);
    v->wm_delete=XInternAtom(v->display,"WM_DELETE_WINDOW",False);
    XSetWMProtocols(v->display,v->window,&v->wm_delete,1);
    if (fullscreen) {
        XSetWindowAttributes attributes;attributes.override_redirect=True;
        XChangeWindowAttributes(v->display,v->window,CWOverrideRedirect,&attributes);
    }
    Atom bypass=XInternAtom(v->display,"_NET_WM_BYPASS_COMPOSITOR",False);
    unsigned long bypass_value=1;
    XChangeProperty(v->display,v->window,bypass,XA_CARDINAL,32,PropModeReplace,
                    (unsigned char*)&bypass_value,1);
    v->gc=XCreateGC(v->display,v->window,0,NULL);
    v->event_base=XShmGetEventBase(v->display);
    for (int i=0;i<2;++i) {
        struct video_slot *s=&v->slot[i];
        s->image=XShmCreateImage(v->display,DefaultVisual(v->display,screen),
                                DefaultDepth(v->display,screen),ZPixmap,NULL,&s->info,w,h);
        if (!s->image || s->image->bits_per_pixel!=32 || s->image->byte_order!=LSBFirst ||
            s->image->red_mask!=0xff0000 || s->image->green_mask!=0xff00 || s->image->blue_mask!=0xff)
            goto failed;
        s->info.shmid=shmget(IPC_PRIVATE,s->image->bytes_per_line*h,IPC_CREAT|0600);
        if (s->info.shmid<0) goto failed;
        s->info.shmaddr=shmat(s->info.shmid,NULL,0);
        if (s->info.shmaddr==(char*)-1) goto failed;
        s->image->data=s->info.shmaddr;s->info.readOnly=False;
        if (!XShmAttach(v->display,&s->info)) goto failed;
        s->attached=1;XSync(v->display,False);
        if(shmctl(s->info.shmid,IPC_RMID,NULL)==0)s->removed=1;
    }
    v->converter=sws_getContext(sw,sh,AV_PIX_FMT_YUYV422,sw,sh,AV_PIX_FMT_BGRA,
                                SWS_FAST_BILINEAR,NULL,NULL,NULL);
    if (!v->converter) goto failed;
    if(posix_memalign((void**)&v->converted,32,(size_t)sw*sh*4)!=0)goto failed;
    v->xmap=malloc(w*sizeof(int));v->ymap=malloc(h*sizeof(int));
    if(!v->xmap || !v->ymap)goto failed;
    for(int x=0;x<w;++x)v->xmap[x]=((2*x+1)*sw)/(2*w);
    for(int y=0;y<h;++y)v->ymap[y]=((2*y+1)*sh)/(2*h);
    XMapRaised(v->display,v->window);XSync(v->display,False);
    return v;
failed:
    live_close(v);return NULL;
}

int live_poll(struct live_video *v) {
    while (XPending(v->display)) {
        XEvent event;XNextEvent(v->display,&event);
        if (event.type==v->event_base+ShmCompletion) {
            XShmCompletionEvent *done=(XShmCompletionEvent*)&event;
            for(int i=0;i<2;++i) if(v->slot[i].info.shmseg==done->shmseg)v->slot[i].busy=0;
        } else if(event.type==KeyPress) {
            KeySym key=XLookupKeysym(&event.xkey,0);
            if(key==XK_Escape || key==XK_q)v->closed=1;
        } else if(event.type==ClientMessage && (Atom)event.xclient.data.l[0]==v->wm_delete) {
            v->closed=1;
        }
    }
    return v->closed;
}

int live_acquire(struct live_video *v) {
    if (live_poll(v)) return -2;
    for(int i=0;i<2;++i)if(!v->slot[i].busy)return i;
    return -1;
}

uint8_t *live_pixels(struct live_video *v,int index) {
    return (index>=0 && index<2) ? (uint8_t*)v->slot[index].image->data : NULL;
}
int live_stride(struct live_video *v) { return v->slot[0].image->bytes_per_line; }

#ifdef __ARM_NEON
static uint8x8x4_t pixels8(uint8x8_t luma,int16x8_t u,int16x8_t c) {
    int16x8_t y=vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(luma)),vdupq_n_s16(16));
    y=vmaxq_s16(y,vdupq_n_s16(0));
    int16x8_t yt=vaddq_s16(vaddq_s16(vmulq_n_s16(y,74),vshrq_n_s16(y,1)),vdupq_n_s16(32));
    int16x8_t red=vaddq_s16(vmulq_n_s16(c,102),vshrq_n_s16(c,2));
    int16x8_t green=vsubq_s16(vsubq_s16(yt,vmulq_n_s16(u,25)),vmulq_n_s16(c,52));
    uint8x8x4_t out;
    out.val[0]=vqmovun_s16(vshrq_n_s16(vqaddq_s16(yt,vmulq_n_s16(u,129)),6));
    out.val[1]=vqmovun_s16(vshrq_n_s16(green,6));
    out.val[2]=vqmovun_s16(vshrq_n_s16(vqaddq_s16(yt,red),6));
    out.val[3]=vdup_n_u8(255);
    return out;
}
static void neon_yuyv(const uint8_t *raw,int stride,uint32_t *dst,int w,int h) {
    for(int y=0;y<h;++y)for(int x=0;x<w;x+=16) {
        uint8x8x4_t packed=vld4_u8(raw+y*stride+x*2);
        int16x8_t u=vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(packed.val[1])),vdupq_n_s16(128));
        int16x8_t c=vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(packed.val[3])),vdupq_n_s16(128));
        uint8x8x4_t even=pixels8(packed.val[0],u,c),odd=pixels8(packed.val[2],u,c),a,b;
        for(int k=0;k<4;++k) {
            uint8x8x2_t zipped=vzip_u8(even.val[k],odd.val[k]);
            a.val[k]=zipped.val[0];b.val[k]=zipped.val[1];
        }
        vst4_u8((uint8_t*)(dst+y*w+x),a);vst4_u8((uint8_t*)(dst+y*w+x+8),b);
    }
}
#endif

int live_convert(struct live_video *v,int index,const uint8_t *raw,int stride) {
    if(index<0 || index>=2 || v->slot[index].busy)return -1;
#ifdef __ARM_NEON
    if(v->source_width==v->width && v->source_height==v->height && v->width%16==0 &&
       v->slot[index].image->bytes_per_line==v->width*4) {
        neon_yuyv(raw,stride,(uint32_t*)v->slot[index].image->data,v->width,v->height);
        return v->height;
    }
    if(v->source_width%16==0) {
        neon_yuyv(raw,stride,v->converted,v->source_width,v->source_height);
    } else
#endif
    {
        const uint8_t *source[4]={raw,NULL,NULL,NULL};int source_stride[4]={stride,0,0,0};
        uint8_t *destination[4]={(uint8_t*)v->converted,NULL,NULL,NULL};
        int destination_stride[4]={v->source_width*4,0,0,0};
        if(sws_scale(v->converter,source,source_stride,0,v->source_height,destination,destination_stride)!=v->source_height)
            return -1;
    }
    for(int y=0;y<v->height;++y) {
        const uint32_t *src=v->converted+v->ymap[y]*v->source_width;
        uint32_t *dst=(uint32_t*)(v->slot[index].image->data+y*v->slot[index].image->bytes_per_line);
        if(v->source_width==640 && v->width==800) {
            for(int x=0;x<160;++x) {
                dst[0]=src[0];dst[1]=src[1];dst[2]=src[2];dst[3]=src[2];dst[4]=src[3];
                src+=4;dst+=5;
            }
        } else {
            for(int x=0;x<v->width;++x)dst[x]=src[v->xmap[x]];
        }
    }
    return v->height;
}

int live_present(struct live_video *v,int index) {
    if(index<0 || index>=2 || v->slot[index].busy)return 0;
    if(!XShmPutImage(v->display,v->window,v->gc,v->slot[index].image,0,0,v->offset_x,v->offset_y,
                     v->width,v->height,True))return 0;
    v->slot[index].busy=1;XFlush(v->display);return 1;
}
