"""Read PX4/EGO topics and emit LAN telemetry; no flight publishers or commands."""
import ipaddress
import json
import math
from pathlib import Path
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import (VehicleLocalPosition,VehicleAttitude,VehicleStatus,
                         BatteryStatus,FailsafeFlags,VehicleAirData,VehicleMagnetometer,SensorGps,VehicleLandDetected)
from std_msgs.msg import String
from traj_utils.msg import Bspline

from .telemetry_core import Assembler
from .wire import encode, finite


class Sender(Node):
    def __init__(self):
        super().__init__('kr260_ground_station_sender')
        host=str(self.declare_parameter('destination_ip','127.0.0.1').value)
        ipaddress.IPv4Address(host)
        port=int(self.declare_parameter('destination_port',14600).value)
        hz=float(self.declare_parameter('rate_hz',5.).value)
        if not 1<=port<=65535 or not math.isfinite(hz) or not 1<=hz<=10:raise ValueError('invalid rate or port')
        self.core=Assembler(str(self.declare_parameter('vehicle_id','uav1').value),
                            self.declare_parameter('origin_ned_m',[0.,0.,0.]).value)
        key_path=str(self.declare_parameter('key_file','').value)
        self.key=Path(key_path).read_bytes().strip() if key_path else None
        if self.key is not None and len(self.key)<32:raise ValueError('shared key needs >=32 bytes')
        self.endpoint=(host,port)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sent=self.skipped=self.errors=0
        self.pending_plan=None
        self.last_log=0.
        self.subscriptions_owned=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)

        def subscribe(param,default,kind,topic_type,fields):
            topic=str(self.declare_parameter(param,default).value)
            def callback(message):
                stamp=float(getattr(message,'timestamp_sample',0) or message.timestamp)/1e6
                wall=self.get_clock().now().nanoseconds/1e9
                if not -.01<=wall-stamp<=2:return
                value={key:getattr(message,key) for key in fields}
                if kind=='position':self.core.position(value,stamp,time.monotonic())
                else:self.core.update(kind,stamp,time.monotonic(),value)
            self.subscriptions_owned.append(self.create_subscription(topic_type,topic,callback,qos))

        subscribe('position_topic','/fmu/out/vehicle_local_position_v1','position',VehicleLocalPosition,
            ['x','y','z','vx','vy','vz','heading','xy_valid','z_valid','v_xy_valid','v_z_valid',
             'heading_good_for_control','dead_reckoning','xy_reset_counter','z_reset_counter','heading_reset_counter'])
        subscribe('attitude_topic','/fmu/out/vehicle_attitude','attitude',VehicleAttitude,['q'])
        subscribe('status_topic','/fmu/out/vehicle_status_v3','status',VehicleStatus,['arming_state','nav_state'])
        subscribe('battery_topic','/fmu/out/battery_status_v1','battery',BatteryStatus,['connected','voltage_v','current_a','remaining'])
        subscribe('flags_topic','/fmu/out/failsafe_flags','flags',FailsafeFlags,['attitude_invalid','angular_velocity_invalid'])
        subscribe('land_topic','/fmu/out/vehicle_land_detected','land',VehicleLandDetected,['landed'])

        def optional(topic,kind,topic_type,predicate):
            self.subscriptions_owned.append(self.create_subscription(topic_type,topic,
                lambda m:self.core.update(kind,float(getattr(m,'timestamp_sample',0) or m.timestamp)/1e6,
                                          time.monotonic(),bool(predicate(m))),qos))
        optional(str(self.declare_parameter('barometer_topic','/fmu/out/vehicle_air_data').value),
                 'barometer',VehicleAirData,lambda m:finite(m.baro_alt_meter) and finite(m.baro_pressure_pa) and m.baro_pressure_pa>0)
        optional(str(self.declare_parameter('compass_topic','/fmu/out/vehicle_magnetometer').value),
                 'compass',VehicleMagnetometer,lambda m:all(finite(v) for v in m.magnetometer_ga) and any(abs(v)>1e-6 for v in m.magnetometer_ga))
        optional(str(self.declare_parameter('gnss_topic','/fmu/out/sensor_gps').value),
                 'gnss',SensorGps,lambda m:m.fix_type>=3)
        self.subscriptions_owned.append(self.create_subscription(String,'/kr260/ego/status',self.controller,1))
        self.subscriptions_owned.append(self.create_subscription(Bspline,'/kr260/ego/bspline',self.spline,1))
        self.timer=self.create_timer(1/hz,self.send)
        self.get_logger().info(f'Receive-only ROS subscriptions -> UDP {host}:{port}, {hz:g} Hz; not flight control')

    def controller(self,msg):
        if len(msg.data)>16384:return
        try:
            value=json.loads(msg.data)
            stamp=value['timestamp_us']/1e6
            wall=self.get_clock().now().nanoseconds/1e9
            if not isinstance(value,dict) or not finite(stamp) or not -.01<=wall-stamp<=.6:return
            self.core.update('controller',stamp,time.monotonic(),value)
        except (KeyError,ValueError,TypeError,RecursionError):pass

    def spline(self,msg):
        # Bound memory before retaining even a locally received spline message.
        if msg.order==3 and 4<=len(msg.pos_pts)<=256 and len(msg.knots)==len(msg.pos_pts)+4:
            self.pending_plan=msg

    def send(self):
        wall=self.get_clock().now().nanoseconds/1e9
        mono=time.monotonic()
        if self.pending_plan is not None:
            msg=self.pending_plan
            stamp=(msg.kr260_goal_stamp.sec,msg.kr260_goal_stamp.nanosec)
            # A B-spline may arrive before the controller's 5Hz acceptance status.
            # Keep one pending candidate and retry ONLY until context catches up.
            if self.core.plan is None or self.core.plan['trajectory']!=msg.traj_id or self.core.plan['goal']!=stamp:
                self.core.planned_spline([[p.x,p.y,p.z] for p in msg.pos_pts],list(msg.knots),
                    msg.start_time.sec+msg.start_time.nanosec/1e9,msg.kr260_run_id,stamp,msg.traj_id,wall,mono)
        packet=self.core.packet(wall,mono)
        if packet is None:self.skipped+=1
        else:
            try:self.sock.sendto(encode(packet,self.key),self.endpoint);self.sent+=1
            except (OSError,ValueError,TypeError):self.errors+=1
        if mono-self.last_log>=5:
            self.get_logger().info(f'UDP sent={self.sent} skipped_stale_or_invalid={self.skipped} errors={self.errors} fault={self.core.fault or "none"}')
            self.last_log=mono

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=Sender()
        rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
