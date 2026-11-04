# 硬件与软件架构

## 平台

- TUL PYNQ-Z2，Zynq-7020 / XC7Z020-1CLG400C，双 Cortex-A9，512 MB DDR。
- 已验证 Linux 内核：`6.6.10-xilinx-v2024.1-g3c0eca68c652`，ARMv7；镜像基于 Ubuntu 22.04。
- Vivado 2022.1；100 MHz AXI/图像处理，40 MHz HDMI 像素、200 MHz 串行时钟；800×600 约 60.317 Hz 输出。

## PL

| 模块 | 地址 / 文件 | 作用 |
|---|---|---|
| AXI DMA | `0x40400000` | PS 内存与手势 AXI-Stream 传输 |
| 肤色参数 GPIO | `0x41200000` | Cb/Cr 和 Y 阈值 |
| 图像配置 GPIO | `0x42000000` | ROI 尺寸及 GST2 版本检查 |
| HDMI VDMA | `0x43000000` | 从 DDR 读取 XRGB8888 帧 |
| 用户输入 GPIO | `0x44000000` | bit0..3 BTN0..3，bit4..5 SW0..1 |
| 形态学 | `gesture_v2.v` / `morph3_stream.v` | 开运算和两次闭运算：六级 E,D,D,D,E,E |
| 显示扫描 | `hdmi_scanout.v` | AXI-Stream、异步 FIFO、固定时序、RGB2DVI |

手势核接收每 32 位字 Y0/Cb/Y1/Cr，返回逐像素 8 位清理掩码，整帧 TLAST；GST2 版本值 `0x47535432`。640×480 源帧结合两倍抽取，分析坐标 320×240、ROI 192×192。边界、背压和帧标记以 RTL 与 CPU 参考测试为准。

HDMI 硬件刷新率不等于 PS 应用更新率。没有通用 PL GPU、PL 点云绘制核、PL 仪表旋转核或已验收 CNN 推理硬件。

## PS

- `desktop_hardware.py`：唯一组合 overlay 所有者、显示缓冲/DRM 配置、GPIO 与统一页面选择。
- `gesture_demo.py`：UVC 采集、最新帧处理、DMA、轮廓/掌心/手腕分析、投票。
- `hand_native.c`：CPU 几何计算；`live_x11.c`：CPU NEON 颜色转换与 X11 MIT-SHM。
- `local_preview.py`：LGF3 三缓冲共享原始帧，租约保护避免读取被覆盖帧。
- `map_display.py` / `point_cloud_map.py`：固定种子点云、首次显现时间缓存、按折线弧长匀速移动；不是实时雷达建图。
- `flight_demo.py`：独立循环预设值；`flight_status.py`：仪表与卡片；`flight_native.c`：CPU 定点最近邻纹理采样。
- `gesture_web.py`：板端局域网 HTTP 预览，JPEG 最多 5 FPS，按访问需求编码，与识别速度分离。

## 页面与运行状态

选择文件 `/run/pynq-desktop/view-selection.json` 的 view 为 `desktop`、`camera`、`map`、`flight_status`。统一选择保证页面互斥。

地图首次打开开始计时，隐藏不重置，60 秒后停留；状态页有独立时钟，隐藏后仍继续，每 60 秒循环。隐藏页面停止绘制，后台摄像头识别可以继续。

主要状态文件：`/run/pynq-xfce/map-control-status.json`、`/run/pynq-xfce/flight-status-view.json`、`/home/xilinx/gesture-status.json`。这些文件是运行时数据，不纳入源码。

## 可选网络输入（代码完成，未部署）

`telemetry_receiver.py` 在 PS 接收只读 UDP JSON，严格校验后限频原子写 `/run/pynq-telemetry/state.json`。`telemetry_view.py` 将同一快照接入地图/状态页；`telemetry.py` 管理序号、会话、接收机时钟超时和有界历史轨迹。通过 `--data-source live` 或独立 service drop-in 选择，默认仍为 demo。不会加载新 overlay；PL 显示链与摄像头不变。

网络输入不等于已接入飞控，发送方需提供本协议数据，详见 WIFI_TELEMETRY.md。没有板端或无线链路实测，不能引用历史 FPS 作为此模式性能。
