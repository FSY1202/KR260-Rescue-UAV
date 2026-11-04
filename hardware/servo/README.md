# KR260 双通道舵机 PWM

包含 PWM RTL、引脚约束、相机 / UART / 舵机集成 Vivado 工程、设备树、UIO 原生访问、限时动作脚本和软件测试。SRV1 寄存器基址为 `0xA0030000`，设备发现按地址和 ABI 匹配，不固定 `/dev/uioN` 编号。

通道可选 0 或 1，一次只启用一路；软件允许 500–1700 μs、0.2–3 秒、20 ms 周期。初始化关闭两路输出，动作正常完成、收到 SIGINT 或 SIGTERM 时均关闭 PWM。PWM 关闭不等于切断 5 V 电源，也不是机械位置反馈。

以项目根目录为当前目录：

```bash
bash hardware/servo/deploy/build_mmio.sh
sudo bash hardware/servo/deploy/servo.sh status
sudo bash hardware/servo/deploy/servo.sh move --channel 0 --pulse-us 1000 --seconds 1
sudo bash hardware/servo/deploy/servo.sh disable
```

`td8120mg.json` 定义 TD-8120MG 的软件脉宽范围；实际开合位置取决于连杆和装配，不将未标定的脉宽直接换算为角度。

## 硬件构建

在已安装 KR260 板级文件的 Vivado 2022.1 环境执行：

```text
vivado -mode batch -source hardware/servo/vivado/build_candidate.tcl
```

`vendor/base_frame_stream/` 是综合所需的冻结相机 RTL 与约束，保留其文件校验。`deploy/kr260_ov5640.dts` 在相机、IIC、UART 和帧采集节点之外增加 SRV1 UIO 节点。位流、设备树及本机 `.so` 由源文件构建，生成文件不随 Git 提交。

`install_candidate.py` 是带基线哈希、进程检查和回滚的已有系统升级工具；`load_servo.py` 只加载已安装且收据匹配的固件。它们不替代新系统的初次固件安装。相机入口默认定位到本源码树，板端部署到其他位置时使用 `KR260_CAMERA_ENTRY` 和 `KR260_WORKSPACE` 指定绝对路径。

自动任务不调用 PL 加载工具。任务级适配器位于 `src/kr260_ego_pkg/scripts/servo_drop_executor.py`，通过本目录的 `servo_move.py` 执行动作，并将结束与 PWM 关闭结果反馈给 EGO 控制器。详见 [救援流程](../../docs/RESCUE_FLOW.md)。
