# Wi-Fi / 局域网遥测：BTN1 地图与 BTN2 数据页

本次为代码实现及电脑本机回环测试，不连接板卡、不操作无线网卡、不改电脑防火墙，也不连接 KR260 或真实飞控。Wi-Fi 与以太网都使用同一 IPv4 UDP 链路；需要事先将发送端和板卡连接到同一可互访局域网。PYNQ-Z2 没有板载 Wi-Fi，需兼容 USB 无线网卡；摄像头同时使用时通常需要合适的 USB Hub。

## 数据链

```text
无人机上位机/计算机 → Wi-Fi LAN / UDP 14600 → telemetry_receiver.py
                                               ↓ 原子快照（20Hz上限）
                             /run/pynq-telemetry/state.json
                                  ↙                    ↘
                       BTN1 平面位置/轨迹          BTN2 姿态/电池/速度
                                  ↘                    ↙
                                   现有 PS 绘制 + PL HDMI
```

这是自定义 JSON 遥测协议，不是 MAVLink 解码器。真实飞控或 KR260 程序需要把已获得的状态转换为此格式后发送。代码仅接收和展示，没有解锁、模式切换、航点或其他控制指令。

## 协议 1

一个 UDP 数据报对应一份完整状态，UTF-8 JSON，最多 4096 字节。全部字段必填、字段名严格匹配；单位为米、秒、度、伏、安和百分比。

```json
{
  "version": 1,
  "data_origin": "test",
  "vehicle_id": "uav1",
  "session_id": "session1",
  "seq": 0,
  "position": {"east_m": 1.0, "north_m": 2.0, "valid": true},
  "attitude": {"roll_deg": 1.0, "pitch_deg": -0.5, "heading_deg": 359.0},
  "flight": {
    "mode": "HOLD", "armed": true, "phase": "hover",
    "relative_altitude_m": 1.05, "ground_speed_mps": 0.22, "climb_rate_mps": 0.0
  },
  "battery": {"voltage_v": 16.8, "current_a": 2.9, "percent": 86.0},
  "sensors": {"imu": true, "barometer": true, "compass": true, "gnss": false}
}
```

- `data_origin` 为 `test` 或 `vehicle`；测试发送器固定为 test，页面明确显示 WIFI TEST DATA，不将合成数据冒充真实飞机数据。
- `vehicle_id`、`session_id`：1～32 位字母/数字/下划线/短横线。单架飞机默认 uav1。
- `seq`：0～2^53-1 的递增整数，同 session 下重复/倒序包丢弃；重启发送端需要新的 session_id，原会话断流至少 3 秒后接收新会话。
- 航向为真北/局部北 0°，顺时针增加至小于 360°；坐标为局部东/北，地图高度不参与投影。发送者必须统一坐标原点和北向。
- phase 为 standby/takeoff/cruise/hover/landing。mode 是最多 16 位的大写 ASCII 字母、数字、下划线或短横线。
- NaN/Infinity、错误类型、重复 JSON 字段、超长数据报、错误车辆和越界数据均拒绝；拒绝包不会刷新链路超时。
- 链路存活只看接收机 monotonic 时间，不依赖两端时钟同步。默认 2 秒无有效数据即不可用；状态页清空数值、姿态与航向，地图隐藏飞机图标，保留历史轨迹。实时模式不自动切回预设数据。
- 最多保留 1200 个位置样本；无效定位、断流和会话重启会切断历史轨迹，避免画出跨断线的直线。

## 坐标与底图

`data/telemetry_map.json` 定义零点显示像素 origin_px、pixels_per_meter 和北向旋转 rotation_deg。默认示例零点 (400,500)，50 像素/米，东向右、北向上；四轴图标随接收航向旋转。

**现有底图仍是用户提供的预设地图，没有真实米制标定。** 默认比例只是联调示例，不能作为测距或真实定位精度。使用真实坐标前，必须用至少两个已知地标统一比例、原点和朝向。收到位置越出显示范围时明确提示，不把飞机夹到边界伪装成有效位置。

实时地图完整显示预设点云背景，暗色线为原规划路线，亮色线为接收的历史轨迹。这一版不接收点云数据、不执行 SLAM，也不改变原 BTN1 演示模式的逐步扫描逻辑。

## 电脑端运行（无需板卡）

在源码根目录，先安装 requirements-dev.txt，分别打开终端：

```bash
python pynq_z2_desktop/software/telemetry_receiver.py --state preview-output/telemetry.json
python tools/send_telemetry_demo.py --host 127.0.0.1 --duration 30
```

一键回环联调、输出地图和状态截图：

```bash
python tools/check_telemetry_loopback.py
python tools/run_checks.py
```

电脑上的截图工具不需要 X11 或 FPGA。地图/状态主程序的 `--data-source live --telemetry-state 文件` 可以配合 `--render-check` / `--snapshot` 导出当前状态；默认 `--data-source demo` 保持原行为。

## 以后上板时的启用步骤（本次未执行）

1. 将源码更新到原两个同级项目目录，Wi-Fi 连接和地址由操作者预先配置。
2. 参考 `software/telemetry.env.example` 写 `/etc/default/pynq-telemetry`，填写**板卡现有 WLAN 地址** `TELEMETRY_BIND` 和发送端地址 `TELEMETRY_PEER`。没有配置时接收器只监听 127.0.0.1，不对外开放。
3. 同一 WLAN 的另一端运行发送器，`--host` 指向板卡地址。真实数据发送者按协议组包；参考 `telemetry.encode_packet()`。
4. 执行 `sudo bash software/install_telemetry.sh live` 安装可选接收服务及 BTN1/BTN2 drop-in；只重启地图和状态页，不重启硬件所有者、不改 PL 或 HDMI 驱动。
5. 恢复演示：`sudo bash software/install_telemetry.sh demo`。BTN0 摄像头不改变。

UDP 示例端口为 14600；源码不自动开放 Windows 公用网络访问，不做端口映射。

## 接入限制和可选校验

非回环监听必须配置固定发送端 IPv4 白名单。IP 白名单不是身份认证，不应暴露到不可信网络。如需校验，双方使用相同、至少 32 字节的共享密钥文件，接收端设置 TELEMETRY_KEY_FILE，发送器传 --key-file；encode_packet 会添加基于规范 JSON 的 HMAC-SHA256 字段 mac，接收方校验后才接受。

密钥文件不放仓库。这不是加密传输，也没有可信的跨重启防重放时间戳；不要把它用于开放互联网或飞行控制。实际无线距离、丢包、时延和板端性能需要后续实测，本次不作保证。
