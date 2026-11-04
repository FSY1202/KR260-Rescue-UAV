# 构建与部署

## 前提

本项目不是通用于所有 Zynq 镜像的一键安装器。使用 PYNQ-Z2、匹配的 PYNQ/XRT 环境、已验证 Linux 内核、Xorg/XFCE 和 HDMI OUT。部署涉及更换 PL 和 root 服务，请保留当前 SD 卡备份，并从板端终端操作。不要使用 KR260 镜像。

Python 解释器预期 `/usr/local/share/pynq-venv/bin/python3`，用户与家目录预期 `xilinx` / `/home/xilinx`。源码没有嵌入密码，认证由操作者终端完成。

板端需要：PYNQ 与 XRT、NumPy、带 V4L2 支持的 OpenCV；系统需要 gcc、make、pkg-config、libx11-dev、libxext-dev、libswscale-dev、libavutil-dev、xauth、x11-xserver-utils、xserver-xorg-core、xserver-xorg-video-modesetting、xfce4-session 和匹配的内核构建树。`nsenter` 由 util-linux 提供。实际镜像可能已经预装，不应盲目升级系统或 PYNQ。

主机验证依赖与板端环境分开，`requirements-dev.txt` 不用于覆盖 PYNQ 镜像环境。

## 1. PL 硬件

获取适用于 Vivado 2022.1 的 TUL PYNQ-Z2 board files，将 `board.xml`、`preset.xml`、`part0_pins.xml` 及官方包要求的其他文件放到 `pynq_z2_gesture/boards/pynq-z2/A.0/`。保留官方许可。Vivado 自带 Xilinx IP 由工具生成，不随源码包直接再分发。

在仓库根目录：

```bash
vivado -mode batch -source pynq_z2_desktop/vivado/build_desktop.tcl
```

输出为 `pynq_z2_desktop/output/desktop_gesture.bit` 与同名 `.hwh`。两个文件必须配套使用。构建后检查时序、资源和 `verify_implemented.tcl` 的验证结果；本轮未重新运行 Vivado。

## 2. 驱动

在与目标内核严格匹配的 ARM 编译环境：

```bash
cd pynq_z2_desktop/kernel
make -C /usr/src/kernel M="$PWD" ARCH=arm CC=arm-linux-gnueabihf-gcc modules
modinfo -F vermagic pynq_hdmi_drm.ko
uname -r
```

编译器前缀、内核树路径按实际环境设置。两个版本必须匹配。驱动 SPDX 为 GPL-2.0-only。源码包不含 `.ko`；另附预编译文件只匹配文档所列内核。

## 3. 放置与安装

将 `pynq_z2_desktop/` 和 `pynq_z2_gesture/` 同级复制到 `/home/xilinx/`，并确保用户可写。中文资源已包含在 data/flight_status，无须在板上安装中文字库。

确认 bit/hwh、驱动已经放到对应目录后：

```bash
cd /home/xilinx/pynq_z2_desktop
sudo bash software/install_desktop.sh
```

整理版安装脚本补装了基础 `gesture-web.service`，然后安装桌面 drop-in、三个页面及 target；这是源码整理后的安装完善，**尚未在新 SD 卡上重跑安装验收**。安装前应备份 `/etc/systemd/system/` 中已有相关服务。

脚本会本机重建 `_hand_geometry.so`、`_live_x11.so`、`_flight_instruments.so`。标准显示路径是 HDMI OUT，输入源应选显示器上对应 HDMI。

## 4. 启动与诊断

```bash
systemctl is-active pynq-desktop-hardware pynq-desktop-xorg pynq-desktop-session
systemctl is-active gesture-web pynq-desktop-camera map-display flight-status
systemctl status --no-pager pynq-desktop.target
journalctl --no-pager -u pynq-desktop-hardware -u flight-status -n 80
```

组合 overlay 由硬件服务唯一管理。桌面运行时不要加载 base、独立 gesture_v2 或其他 overlay，否则 HDMI 会消失。`bootpy.service`、`clear_pl_statefile.service` 必须早于显示硬件服务。保留现有启动依赖，不单独重启硬件所有者。

必要时有序重启整个显示链：

```bash
sudo systemctl restart pynq-desktop.target
```

启动过程中可能执行镜像原有 boot.py，需等待完整启动，不以暂时黑屏判断死机。没有摄像头时摄像头识别服务等待 `/dev/video0`，不应影响地图和桌面。

## 网络预览

默认示例地址为 `http://192.168.137.2:8080/`。该地址必须实际配置在板端；源码不修改电脑网络或 Windows 防火墙。需要其他地址时配置 `GESTURE_WEB_HOST`，不要把示例 IP 当成自动发现结果。

HTTP 预览提供摄像头及诊断端点，没有登录认证，不要映射到互联网。串口使用实际枚举的 UART，115200、8N1；COM 编号不是项目固定属性。

## 回退

运行前先备份原两个项目目录及 systemd 配置。`restore_web_only.sh` 依赖旧版独立手势 bit/hwh 和原桌面快捷方式备份，它们**不在本次源码包中**；只有准备好相应旧资源时才使用。完整回退应恢复自己的 SD 卡或部署前备份，而不是混用不匹配的 overlay。
