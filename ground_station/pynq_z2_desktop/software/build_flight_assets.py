"""Build Chinese label rasters on the PC; no CJK font is needed on the board."""
import argparse
from pathlib import Path
import json
import math

from PIL import Image, ImageDraw, ImageFont


def build(output, font_path):
    output.mkdir(parents=True, exist_ok=True)
    fonts = {size: ImageFont.truetype(str(font_path), size) for size in (13, 14, 15, 16, 17, 18, 24, 26)}
    bg = Image.new('RGB', (800, 600), '#111820')
    d = ImageDraw.Draw(bg)

    def text(pos, value, size=17, fill='#a3b4c5'):
        d.text(pos, value, font=fonts[size], fill=fill)

    d.rectangle((0, 0, 799, 48), fill='#182430')
    text((20, 9), '飞行状态', 26, '#f0f5fa')
    text((150, 21), '四轴飞行器', 15)
    d.rounded_rectangle((20, 61, 780, 100), 8, fill='#202e3b')
    text((32, 70), '飞行模式', 15)
    text((548, 71), '本地定位：正常', 14, '#8ed5b2')
    d.rounded_rectangle((20, 112, 448, 412), 10, fill='#253442', outline='#3d5060', width=1)
    # The instrument texture overwrites only this inner rectangle at runtime.
    d.rectangle((28, 120, 439, 399), fill='#406a90')
    d.rounded_rectangle((20, 420, 448, 469), 8, fill='#202c38')
    d.rounded_rectangle((20, 477, 448, 525), 8, fill='#202c38')
    for x, label in ((30, '航向'), (174, '横滚'), (311, '俯仰')):
        text((x, 481), label, 14)
        text((x+88, 501), 'deg', 13)
    cards = []
    names = [('相对高度', 'm'), ('地速', 'm/s'), ('爬升率', 'm/s'),
             ('电池电压', 'V'), ('电流', 'A'), ('剩余电量', '%')]
    for i, (label, unit) in enumerate(names):
        x, y = (468 if i % 2 == 0 else 628), 112+(i//2)*138
        d.rounded_rectangle((x, y, x+150, y+123), 9, fill='#202d39', outline='#344756')
        text((x+14, y+12), label, 17)
        text((x+15, y+96), unit, 14, '#6cbbdb')
        cards.append([x, y, 150, 123])
    d.rounded_rectangle((20, 538, 780, 589), 8, fill='#1b2934')
    for x, label, value in ((32, 'IMU', '正常'), (181, '气压计', '正常'),
                            (330, '磁罗盘', '正常'), (479, '本地定位', '正常'),
                            (628, 'GNSS', '未接入')):
        text((x, 542), label, 13)
        text((x, 562), value, 15, '#d1b66d' if label == 'GNSS' else '#82ccad')
    bg.save(output/'background.png')

    horizon = Image.new('RGB', (640, 640), '#427ba6')
    hd = ImageDraw.Draw(horizon)
    hd.rectangle((0, 320, 639, 639), fill='#78593e')
    hd.line((0, 320, 639, 320), fill='#eef0df', width=2)
    for pitch in range(-80, 81, 5):
        if not pitch:
            continue
        y = 320-pitch*3
        length = 35 if pitch % 10 == 0 else 17
        color = '#dce4dd'
        hd.line((320-length, y, 310, y), color, 1)
        hd.line((330, y, 320+length, y), color, 1)
        if pitch % 10 == 0:
            hd.text((320+length+7, y-9), str(abs(pitch)), font=fonts[14], fill=color)
    horizon.save(output/'horizon.png')

    compass = Image.new('RGB', (3240, 48), '#202c38')
    cd = ImageDraw.Draw(compass)
    for angle in range(0, 1080, 5):
        x = angle*3
        cd.line((x, 36 if angle % 10 else 31, x, 45), fill='#849bac')
        if angle % 30 == 0:
            label = {0:'N',90:'E',180:'S',270:'W'}.get(angle % 360, str(angle % 360))
            cd.text((x, 4), label, font=fonts[15], anchor='mt', fill='#e1eaf0')
    compass.save(output/'compass.png')

    # Pre-render just the fixed overlay; compact nontransparent pixels are cached.
    overlay = Image.new('RGBA', (412, 280), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    cx, cy = 206, 140
    for roll in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
        angle = math.radians(roll-90)
        r = 119
        x1,y1 = cx+r*math.cos(angle),cy+r*math.sin(angle)
        length = 10 if roll % 30 == 0 else 6
        x2,y2 = cx+(r-length)*math.cos(angle),cy+(r-length)*math.sin(angle)
        od.line((x1,y1,x2,y2), fill='#f4f6ed', width=2)
    od.polygon([(cx, 12), (cx-5, 21), (cx+5, 21)], fill='#ffda64')
    for dx,dy in ((-15,-9),(-15,9),(15,-9),(15,9)):
        od.line((cx,cy,cx+dx,cy+dy),fill='#ffdf62',width=2)
        od.ellipse((cx+dx-4,cy+dy-4,cx+dx+4,cy+dy+4),outline='#ffdf62',width=2)
    od.rectangle((cx-4,cy-4,cx+4,cy+4), fill='#ffdf62')
    od.polygon([(cx,cy-10),(cx-3,cy-4),(cx+3,cy-4)],fill='#fff0aa')
    od.line((cx-63,cy,cx-33,cy),fill='#ffe476',width=2)
    od.line((cx+33,cy,cx+63,cy),fill='#ffe476',width=2)
    overlay.save(output/'overlay.png')

    sprites = Image.new('RGB', (180, 7*30), '#202e3b')
    sd = ImageDraw.Draw(sprites)
    values = ['待机', '起飞', '巡航', '悬停', '降落', '未解锁', '已解锁']
    for i,value in enumerate(values):
        sd.text((2,4+i*30),value,font=fonts[16],fill='#8bd9b4' if i == 6 else '#d5e1e9')
    sprites.save(output/'states.png')
    import cv2
    import numpy as np
    characters='0123456789+-.'
    for name,cw,ch,font_scale,baseline,bgr in [
            ('digits_large',24,36,.91,29,(57,45,32)),
            ('digits_small',17,25,.62,20,(56,44,32))]:
        raster=np.empty((ch,cw*len(characters),4),np.uint8)
        raster[:]=(*bgr,255)
        for i,char in enumerate(characters):
            cv2.putText(raster,char,(i*cw,baseline),0,font_scale,(238,240,235,255),1,cv2.LINE_AA)
        cv2.imwrite(str(output/(name+'.png')),raster)
    (output/'layout.json').write_text(json.dumps(dict(format='FLIGHT_UI_1', cards=cards,
        attitude=[28,120,412,280], sprites=values, font_rasterized=True), indent=2),encoding='utf-8')
    print('Built flight assets:', output)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'data/flight_status')
    p.add_argument('--font',type=Path,default=Path('C:/Windows/Fonts/msyh.ttc'))
    a = p.parse_args()
    build(a.output,a.font)
