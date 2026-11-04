# OV5640 与 YOLO26n 视觉

OV5640 的 DVP 信号由 PL 采集，通过三缓冲区帧接口提供给 PS；Python 输出 JPEG / MJPEG。YOLO26n 使用独立的 NCNN 推理服务，相机采集进程不加载神经网络。

| 目录 | 内容 |
| --- | --- |
| `camera` | DVP / 帧缓冲 RTL、Vivado Tcl、相机寄存器设置、UIO 原生访问和 HTTP 画面 |
| `pl_frame_sources` | 冻结的相机硬件接口参考 |
| `yolo_standard` | 标准预览程序、共享模型目录与导出校验记录 |
| `yolo_fast` | 单方向跨帧搜索、快速预览与任务负载配置 |

`yolo_standard/models/yolo26n.pt` 是项目使用的原权重，SHA256 为 `9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef`。四个 NCNN 模型使用同一权重导出，FP32，输入尺寸分别为 320、384、480、640；模型加载时核对 `.param` 和 `.bin` 的 SHA256。

## 运行

Ubuntu 需具备 `python3-opencv`、`python3-numpy`、`python3-venv` 和匹配的相机 PL / UIO 配置。以项目根目录为当前目录执行：

```bash
bash vision/camera/deploy/build_mmio.sh
bash vision/setup_python.sh
bash vision/start_mission.sh
```

任务配置固定使用 320 输入、单推理线程、每帧一次方向推理；识别失败时在后续新画面尝试其他方向。原始相机预览为 3 FPS。推理不会把历史框叠加到新画面。

- 原始画面：`http://<板卡IP>:8080/raw.jpg`
- 识别画面：`http://<板卡IP>:8081/`
- 检测元数据：`http://127.0.0.1:8081/status.json`

按 Ctrl-C 停止前台推理；相机使用独立服务，停止命令为 `bash vision/camera/deploy/preview.sh stop`。标准、快速与任务识别共用 8081，同一时间只启动一个。

相机原始图像响应包含 `X-Camera-Session`、`X-Camera-Frame-Seq`、`X-Camera-Monotonic-Ns`。YOLO 保留这些字段，由 ROS 桥接传给 EGO 的两帧确认逻辑。源时间取自帧完成时间的硬件年龄估算，不用 HTTP 收到时间刷新旧画面。

相机不是 USB 摄像头，不使用 `VideoCapture(0)`。完整视觉救援接口与执行器启动见 [RESCUE_FLOW.md](../docs/RESCUE_FLOW.md)。

## 许可

模型及 Ultralytics 相关内容遵循随附 `LICENSES/Ultralytics-AGPL-3.0.txt`。OV5640 寄存器表保留 Linux v5.10 驱动的 GPL-2.0-or-later 声明、作者和源文件哈希；NCNN 是外部运行依赖。
