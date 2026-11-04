# 验证方法

## 主机软件回归

```bash
python -m pip install -r requirements-dev.txt
python tools/run_checks.py
```

运行按键消抖、点云/门框路线、隐藏时钟、状态数值与姿态、分类掩码、网页同帧标签、共享帧和原像素预览测试。ARM 本地库不在主机源码中，因此对应本地核验证会明确跳过；Linux-only 测试在 Windows 上明确跳过，CI 使用 Ubuntu 执行。

这些测试不访问板端 PL 或摄像头，不代表硬件部署验收。CI 文件已经配置，但在 GitHub 执行前不能声称云端 CI 已通过。

新增 `test_telemetry.py` 覆盖 JSON 字段/数值/HMAC、乱序、会话重启、超时、轨迹断点与容量、UDP 回环、地图标定及共享页面数据。`python tools/check_telemetry_loopback.py` 启动真正的接收进程和合成发送进程，检查两个 UI 适配器及断流状态，输出 preview-output/wifi-*.png；这不是板上测试或真实无线测试。

## 预览和演示视频

```bash
python tools/render_previews.py
python tools/export_btn1_video.py
```

分别输出三张地图阶段预览、四张状态页预览，以及 800×600、60 秒、30 FPS H.264 MP4。视频使用与 BTN1 相同的数据和渲染函数，属于离线导出，不是板端实际帧率录屏。

中文位图已附带；若重建可传入有适当许可的 CJK 字体：

```bash
python pynq_z2_desktop/software/build_flight_assets.py --font /path/to/CJK-font.otf
```

字体文件不随本包分发；更换字体后需目视检查中文、单位和卡片边界。

## RTL 仿真

Windows PowerShell，先配置 Vivado 2022.1 bin 路径：

```powershell
$env:VIVADO_BIN = 'C:\Xilinx\Vivado\2022.1\bin'
& .\pynq_z2_gesture\sim\run_v2.ps1
```

V2 testbench 检查阈值、六级形态学、边界、连续帧、随机背压及 TLAST，需看到脚本要求的 PASS 标记。主机 Python 测试不替代 xsim，本次整理未重跑 Vivado 仿真。

## 板端验证

在保持同一组合 overlay 的前提下检查：

1. GPIO 按下/长按/松开及三个页面互斥切换。
2. DMA 连续多帧和 CPU 参考逐像素一致；硬件版本拒绝错误格式。
3. 三类手势、无手、四方向和低置信度/不完整画面拒绝。
4. 地图 60 秒、过两个门框中心、隐藏恢复和结束停留。
5. 状态页两轮、航向跨零、数值范围和隐藏时钟。
6. 连续 120 秒统计采集、复制、DMA、分类、显示、丢帧及延迟；保存原始日志。

避免两个程序同时占用摄像头或重新加载 PL。冷启动、新镜像安装和新的硬件构建需要单独验收。
