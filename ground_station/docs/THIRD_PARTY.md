# 来源、许可与包边界

| 项目 | 来源/用途 | 许可和处理 |
|---|---|---|
| RGB2DVI 1.2 | Digilent，来自本机原 Xilinx/PYNQ v3.0.1 工程 | BSD-3-Clause，保留全部源文件版权头及 THIRD_PARTY_LICENSES.txt；本地接口/时钟约束修改随源码提供 |
| 本地 DRM 驱动 | kernel/pynq_hdmi_drm.c | 源码 SPDX GPL-2.0-only，单独适用；编译产物不放源码包 |
| TUL board files | PYNQ-Z2 硬件预设 | 本机副本未附完整许可来源，不直接分发；请从官方包取得并保留许可 |
| Vivado/Xilinx IP | PS7、AXI DMA、VDMA、GPIO、时钟和复位 | 使用用户自有合法工具环境生成，未打包工具、缓存、生成 IP 或 dcp |
| PYNQ / XRT / Linux / XFCE / Xorg | 目标系统基础环境 | 不随源码分发，保留目标镜像原有许可 |
| NumPy / OpenCV / Pillow / imageio-ffmpeg | 测试、图像和视频工具依赖 | 通过包管理器安装，保留各包许可；不随源码打包第三方 wheel 或 FFmpeg 二进制 |
| 中文静态标签 | build_flight_assets.py 生成的文本位图 | 包含渲染 PNG，不含字体程序；原环境用 Windows 中文字体，公开前确认使用权，亦可用有再分发许可的 CJK 字体重新生成 |
| 地图参考图 | 用户提供的地图和路线图 | 已保留重现所需图片；公开前由提交者确认图片权利归属 |
| 手势 fixtures | 原工程已用的二值掩码回归 | 不含原始人像、原始摄像头帧或身份信息；不是独立准确率数据集 |

QGC 仅为界面信息组织参考，没有复制或安装完整 QGroundControl。本包不打包原工程的 SqueezeNet、AI 原型、训练录像、个人账号、校园网认证、串口密码脚本、原始飞控日志或 KR260 飞行控制工程。
