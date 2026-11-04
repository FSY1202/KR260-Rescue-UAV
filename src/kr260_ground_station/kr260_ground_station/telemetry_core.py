"""Bounded latest-value assembly. Invalid/stale inputs are never replaced by demo."""
import math
import secrets

from .wire import finite, ned_to_local_xy, map_to_ned, quaternion_euler, sample_spline


class Assembler:
    def __init__(self,vehicle='uav1',origin=(0.,0.,0.)):
        import re
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,32}',vehicle):raise ValueError('invalid vehicle ID')
        if len(origin)!=3 or not all(finite(v) for v in origin):raise ValueError('invalid NED origin')
        self.vehicle,self.origin=vehicle,list(origin)
        self.session=secrets.token_hex(8)
        self.seq=0
        self.cache={}
        self.reset=None
        self.fault=''
        self.plan=None

    def update(self,kind,stamp,rx,value):
        if not finite(stamp) or not finite(rx) or stamp<=0:return False
        previous=self.cache.get(kind)
        if previous and stamp<=previous[0]:return False
        self.cache[kind]=(stamp,rx,value)
        return True

    def position(self,value,stamp,rx):
        previous=self.cache.get('position')
        if previous and stamp<=previous[0]:return False
        resets=tuple(value[k] for k in ['xy_reset_counter','z_reset_counter','heading_reset_counter'])
        if self.reset is not None and resets!=self.reset:
            self.fault='PX4 position/heading reset; re-align map and restart this sender'
            self.plan=None
            return False
        self.reset=resets
        return self.update('position',stamp,rx,value)

    def fresh(self,kind,wall,mono,limit):
        item=self.cache.get(kind)
        return item is not None and -.01<=wall-item[0]<=limit and 0<=mono-item[1]<=limit

    def planned_spline(self,points,knots,start,run,goal_stamp,traj_id,wall,mono):
        # Use the controller's frozen map->NED transform, never assume LIO=ENU.
        self.plan=None
        if not self.fresh('controller',wall,mono,.6):return False
        context=self.cache['controller'][2]
        if (context.get('run_id')!=run or not context.get('alignment_frozen') or
            goal_stamp!=(context.get('goal_stamp_sec'),context.get('goal_stamp_nanosec'))):return False
        if not context.get('planning_healthy') or context.get('trajectory_id')!=traj_id:return False
        translation=context.get('alignment_translation')
        delta=context.get('alignment_delta_rad')
        if not isinstance(translation,list) or len(translation)!=3 or not all(finite(v) for v in translation) or not finite(delta):return False
        try:
            sampled=sample_spline(points,knots)
            if not finite(start) or not -.05<=wall-start<=60:return False
            expiry=start+knots[len(points)]-knots[3]
            if expiry<=wall:return False
            xy=[ned_to_local_xy(map_to_ned(p,delta,translation),self.origin) for p in sampled]
            if any(abs(v)>100000 for p in xy for v in p):return False
            self.plan=dict(path=dict(id=str(traj_id),valid=True,points=xy),expiry=expiry,
                           run=run,goal=goal_stamp,trajectory=traj_id)
            return True
        except (ValueError,TypeError,OverflowError):return False

    def packet(self,wall,mono):
        if self.fault:return None
        for name,age in [('position',.3),('attitude',.3),('status',2.),('battery',2.)]:
            if not self.fresh(name,wall,mono,age):return None
        p=self.cache['position'][2];a=self.cache['attitude'][2]
        status=self.cache['status'][2];battery=self.cache['battery'][2]
        try:
            pose=[p[k] for k in ['x','y','z','vx','vy','vz','heading']]
            if not all(finite(v) for v in pose):return None
            if not all(p[k] for k in ['xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control']) or p.get('dead_reckoning',False):return None
            if not battery['connected'] or not all(finite(battery[k]) for k in ['voltage_v','current_a','remaining']):return None
            # PX4 current_a=-1 and remaining=-1 are INVALID, not real readings.
            if not 0<battery['voltage_v']<=100 or not 0<=battery['current_a']<=1000 or not 0<=battery['remaining']<=1:return None
            roll,pitch=quaternion_euler(a['q'])
            xy=ned_to_local_xy([p['x'],p['y'],p['z']],self.origin)
            height=self.origin[2]-p['z']
            speed=math.hypot(p['vx'],p['vy']);climb=-p['vz']
            if max(map(abs,xy))>100000 or not -1000<=height<=10000 or speed>200 or abs(climb)>100:return None
            armed=status['arming_state']==2
            mode={2:'POSCTL',14:'OFFBOARD',18:'LAND',4:'MISSION',3:'HOLD'}.get(status['nav_state'],'MODE_'+str(status['nav_state']))
            phase='cruise' if armed else 'standby'
            if armed and speed<.1:phase='hover'
            if status['nav_state']==18:phase='landing'
            if armed and climb>.15 and height<.6:phase='takeoff'
            if self.fresh('land',wall,mono,2) and self.cache['land'][2]['landed']:phase='standby'
            sensors={name:False for name in ['imu','barometer','compass','gnss']}
            if self.fresh('flags',wall,mono,2):
                flags=self.cache['flags'][2]
                sensors['imu']=not flags['attitude_invalid'] and not flags['angular_velocity_invalid']
            for name in ['barometer','compass','gnss']:
                sensors[name]=self.fresh(name,wall,mono,2) and self.cache[name][2]
            result=dict(version=1,data_origin='vehicle',vehicle_id=self.vehicle,session_id=self.session,seq=self.seq,
                position=dict(east_m=xy[0],north_m=xy[1],valid=True),
                attitude=dict(roll_deg=roll,pitch_deg=pitch,heading_deg=math.degrees(p['heading'])%360),
                flight=dict(mode=mode,armed=armed,phase=phase,relative_altitude_m=height,ground_speed_mps=speed,climb_rate_mps=climb),
                battery=dict(voltage_v=battery['voltage_v'],current_a=battery['current_a'],percent=battery['remaining']*100),sensors=sensors)
            plan=dict(id='none',valid=False,points=[])
            if self.plan and self.fresh('controller',wall,mono,.6):
                context=self.cache['controller'][2]
                if (wall<self.plan['expiry'] and context.get('run_id')==self.plan['run'] and
                    context.get('trajectory_id')==self.plan['trajectory'] and context.get('planning_healthy') and
                    self.plan['goal']==(context.get('goal_stamp_sec'),context.get('goal_stamp_nanosec'))):
                    plan=self.plan['path']
            result['planned_path']=plan
            self.seq+=1
            return result
        except (KeyError,ValueError,TypeError,OverflowError):return None
