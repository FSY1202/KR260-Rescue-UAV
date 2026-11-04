"""Same contour rules as V1, with PL-cleaned input and vectorized defects."""
import cv2
import numpy as np

def classify_clean(clean):
    contours,_=cv2.findContours(clean,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    info={'gesture':'NONE','gaps':0,'solidity':0.0}
    if not contours:return info,None
    contour=max(contours,key=cv2.contourArea);area=cv2.contourArea(contour)
    if area<max(250,clean.size*0.025):return info,None
    hull_area=cv2.contourArea(cv2.convexHull(contour))
    if hull_area<=0:return info,None
    x,y,w,h=cv2.boundingRect(contour)
    distance=cv2.distanceTransform(clean,cv2.DIST_L2,5)
    component=np.zeros_like(clean);cv2.drawContours(component,[contour],-1,255,-1)
    _,radius,_,palm=cv2.minMaxLoc(distance,mask=component)
    py,px=np.nonzero((distance>=radius*0.98)&(component>0))
    if len(px):palm=(float(px.mean()),float(py.mean()))
    indices=cv2.convexHull(contour,returnPoints=False);gaps=0
    if indices is not None and len(indices)>=4 and len(contour)>=4:
        try: defects=cv2.convexityDefects(contour,indices)
        except cv2.error: defects=None
        if defects is not None:
            candidates=defects.reshape(-1,4);candidates=candidates[candidates[:,3]>max(8,radius*0.35)*256]
            if len(candidates):
                points=contour[:,0].astype(np.float64)
                start,end,valley=(points[candidates[:,i]] for i in range(3))
                a=start-valley;b=end-valley
                norm=np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)
                cosine=np.divide(np.einsum('ij,ij->i',a,b),norm,out=np.full(len(norm),-1.0),where=norm>0)
                sr=np.linalg.norm(start-palm,axis=1);er=np.linalg.norm(end-palm,axis=1);vr=np.linalg.norm(valley-palm,axis=1)
                near=np.minimum(sr,er)
                gaps=int(np.count_nonzero((norm>0)&(cosine>np.cos(np.deg2rad(100)))&(near>radius*1.4)&(vr<radius*2.3)&(vr<near*0.85)))
    solidity=area/hull_area;extent=area/(w*h)
    if gaps>=3:label='OPEN'
    elif gaps==1 and solidity<0.90:label='V'
    elif gaps==0 and solidity>=0.82 and extent>=0.50 and max(w/h,h/w)<1.65:label='FIST'
    else:label='UNKNOWN'
    info.update(gesture=label,gaps=gaps,solidity=round(solidity,3),bbox=[x,y,w,h])
    return info,contour
