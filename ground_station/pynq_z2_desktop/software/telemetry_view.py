"""Shared live-data adapters for BTN1 and BTN2; no demo fallback on outage."""
import json
import math
from pathlib import Path

import cv2
import numpy as np

from flight_demo import FlightState
from point_cloud_map import draw_quadcopter
from telemetry import TelemetryReader, flight_state, number, exact_keys


class MapCalibration:
    """Local east/north metres to the existing 800x600 map (no height)."""
    def __init__(self, value):
        exact_keys(value,['origin_px','pixels_per_meter','rotation_deg'])
        origin=value['origin_px']
        if not isinstance(origin,list) or len(origin)!=2:raise ValueError('Invalid map origin')
        number(origin[0],0,799);number(origin[1],38,567)
        number(value['pixels_per_meter'],.01,1000)
        number(value['rotation_deg'],-360,360)
        self.origin=np.asarray(origin,float)
        self.scale=value['pixels_per_meter']
        self.rotation=value['rotation_deg']
        r=math.radians(self.rotation)
        self.matrix=np.array([[math.cos(r),-math.sin(r)],[-math.sin(r),-math.cos(r)]])

    def point(self, east, north):
        return self.origin+self.scale*(self.matrix@np.array([east,north]))

    def heading(self, heading_deg):
        return heading_deg-90-self.rotation

    def inverse(self, pixel):
        return self.matrix.T@((np.asarray(pixel,float)-self.origin)/self.scale)


def banner(frame,text,connected):
    cv2.rectangle(frame,(0,0),(799,36),(21,17,10,255),-1)
    color=(149,230,145,255) if connected else (100,130,255,255)
    cv2.putText(frame,text,(20,24),0,.55,color,1,cv2.LINE_8)


class LiveMapView:
    def __init__(self, base_frame, reader, calibration):
        self.background=base_frame.copy()
        self.reader=reader
        self.calibration=calibration

    def draw(self, frame, now=None):
        snapshot,connected,age=self.reader.read(now)
        np.copyto(frame,self.background)
        packet=snapshot['packet'] if snapshot else None
        if connected and packet.get('planned_path',{}).get('valid'):
            points=[self.calibration.point(*p) for p in packet['planned_path']['points']]
            cv2.polylines(frame,[np.rint(points).astype(np.int32)],False,(255,190,65,255),2,cv2.LINE_8)
        if snapshot:
            segment=[]
            for point in snapshot['trail']+[None]:
                if point is None:
                    if len(segment)>1:
                        cv2.polylines(frame,[np.rint(segment).astype(np.int32)],False,(82,224,205,255),2,cv2.LINE_8)
                    segment=[]
                else:
                    segment.append(self.calibration.point(*point))
        position_valid=connected and packet['position']['valid']
        message='WIFI TELEMETRY | CONNECTED' if connected else 'WIFI TELEMETRY | NO FRESH DATA'
        if connected and packet['data_origin']=='test':
            message='WIFI TEST DATA | CONNECTED'
        if position_valid:
            p=packet['position']
            x,y=self.calibration.point(p['east_m'],p['north_m'])
            if 18<=x<=782 and 38<=y<=567:
                draw_quadcopter(frame,x,y,self.calibration.heading(packet['attitude']['heading_deg']))
            else:
                message='WIFI TELEMETRY | POSITION OUTSIDE MAP'
        elif connected:
            message='WIFI TELEMETRY | POSITION INVALID'
        banner(frame,message,position_valid)
        cv2.rectangle(frame,(0,568),(799,599),(21,17,10,255),-1)
        cv2.putText(frame,'PRESET BACKGROUND | CYAN PLAN / GREEN TRACK | METRES',(20,588),0,.4,(178,172,135,255),1,cv2.LINE_8)
        return connected


class LiveFlightView:
    def __init__(self,reader):
        self.reader=reader
        self.snapshot=None
        self.connected=False

    def sample(self,now=None):
        self.snapshot,self.connected,_=self.reader.read(now)
        if self.snapshot and self.snapshot['packet']:
            return flight_state(self.snapshot['packet'])
        return FlightState('udp_telemetry','standby','WAITING',False,0.,0.,0.,0.,0.,0.,0.,0.,0.,'invalid','not_connected')

    def overlay(self,frame):
        connected=self.connected
        # Keep the Chinese title; this area is unused by the preset header.
        cv2.rectangle(frame,(470,0),(799,47),(48,36,24,255),-1)
        label='WIFI CONNECTED' if connected else 'WIFI NO FRESH DATA'
        if connected and self.snapshot['packet']['data_origin']=='test':
            label='WIFI TEST DATA'
        color=(149,230,145,255) if connected else (100,130,255,255)
        cv2.putText(frame,label,(485,29),0,.47,color,1,cv2.LINE_8)
        packet=self.snapshot['packet'] if self.snapshot else None
        # Never retain static "normal" sensor labels for a live connection.
        cv2.rectangle(frame,(20,538),(779,589),(52,39,26,255),-1)
        health = packet['sensors'] if connected else {}
        statuses=[('IMU',health.get('imu')),('BARO',health.get('barometer')),
                  ('MAG',health.get('compass')),('POS',packet['position']['valid'] if connected else None),
                  ('GNSS',health.get('gnss'))]
        for x,(name,value) in zip((30,180,330,480,630),statuses):
            text=name+': '+('--' if value is None else ('OK' if value else 'UNAVAILABLE'))
            cv2.putText(frame,text,(x,570),0,.35,(178,230,190,255) if value else (100,130,255,255),1,cv2.LINE_8)
        cv2.rectangle(frame,(540,65),(778,95),(59,46,32,255),-1)
        valid=connected and packet['position']['valid']
        cv2.putText(frame,'POSITION: '+('VALID' if valid else 'UNAVAILABLE'),(548,85),0,.38,color,1,cv2.LINE_8)
        if not connected:
            # Blank stale values instead of silently switching back to demo.
            for x,y in [(462,112),(462,250),(462,388),(626,112),(626,250),(626,388)]:
                cv2.rectangle(frame,(x+8,y+43),(x+147,y+93),(56,43,31,255),-1)
                cv2.putText(frame,'--',(x+25,y+79),0,.8,(150,160,170,255),1,cv2.LINE_8)
            cv2.rectangle(frame,(106,65),(530,95),(59,46,32,255),-1)
            cv2.putText(frame,'TELEMETRY UNAVAILABLE',(108,87),0,.48,color,1,cv2.LINE_8)
            cv2.rectangle(frame,(28,120),(439,399),(56,43,31,255),-1)
            cv2.putText(frame,'NO FRESH ATTITUDE',(90,265),0,.55,color,1,cv2.LINE_8)
            cv2.rectangle(frame,(28,420),(439,524),(56,43,31,255),-1)
            cv2.putText(frame,'HEADING / ROLL / PITCH: --',(48,480),0,.43,color,1,cv2.LINE_8)
