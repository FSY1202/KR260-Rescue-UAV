"""PYNQ telemetry v1 encoder, independent of ROS/OpenCV/PYNQ."""
import hashlib
import hmac
import json
import math

def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)

def encode(packet,key=None):
    def canonical(value):
        return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf-8')
    value=dict(packet)
    if key:value['mac']=hmac.new(key,canonical(value),hashlib.sha256).hexdigest()
    raw=canonical(value)
    if len(raw)>4096:raise ValueError('telemetry packet exceeds 4096 bytes')
    return raw

def ned_to_local_xy(position,origin):
    # PX4 local NED, not a promise of geographic north/GNSS alignment.
    return [round(position[1]-origin[1],4),round(position[0]-origin[0],4)]

def map_to_ned(position,delta,translation):
    c,s=math.cos(delta),math.sin(delta)
    x,y,z=position
    return [c*x+s*y+translation[0],s*x-c*y+translation[1],-z+translation[2]]

def quaternion_euler(q):
    if len(q)!=4 or not all(finite(v) for v in q) or abs(sum(v*v for v in q)-1)>.02:
        raise ValueError('invalid attitude quaternion')
    w,x,y,z=q
    return (math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),
            math.degrees(math.asin(max(-1.,min(1.,2*(w*y-z*x))))))

def sample_spline(points,knots,degree=3,count=32):
    """de Boor sampling for DISPLAY only, not flight path acceptance."""
    import bisect
    if degree!=3 or not 4<=len(points)<=256 or len(knots)!=len(points)+4 or not 2<=count<=64:
        raise ValueError('unsupported spline shape')
    if any(len(p)!=3 or not all(finite(v) for v in p) for p in points) or not all(finite(v) for v in knots):
        raise ValueError('nonfinite spline')
    if any(b<a for a,b in zip(knots,knots[1:])):raise ValueError('unordered knots')
    begin,end=knots[3],knots[len(points)]
    if not 1e-6<end-begin<=60:raise ValueError('invalid spline time span')
    result=[]
    for i in range(count):
        t=begin+(end-begin)*i/(count-1)
        k=min(len(points)-1,max(3,bisect.bisect_right(knots,t)-1))
        d=[list(points[k-3+j]) for j in range(4)]
        for r in range(1,4):
            for j in range(3,r-1,-1):
                left,right=knots[k-3+j],knots[k+1+j-r]
                if right<=left:raise ValueError('zero spline interval')
                alpha=(t-left)/(right-left)
                d[j]=[(1-alpha)*a+alpha*b for a,b in zip(d[j-1],d[j])]
        result.append(d[3])
    return result
