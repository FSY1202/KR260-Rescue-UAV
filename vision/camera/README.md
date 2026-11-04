# KR260 continuous OV5640 camera (FRM2)

FRM2 continuous camera capture retains camera pins,
UART1 AA13/AB13, AXI IIC, color statistics, addresses and interrupts.

The camera stream feeds three UltraRAM frame banks (614400 bytes each). Ownership
is `FREE -> WRITING -> READY -> READING -> FREE`. Only the latest unclaimed READY
can be replaced. A claimed READING bank is immutable until release. Frame copying
and the next capture proceed concurrently. Bad frames and FIFO timeout recovery
never publish partial images or modify a claimed frame.

No DDR DMA or new Linux memory carve-out is used. The current 32-bit MMIO path
limits readout to around 28 FPS from measured 35.5 ms copies; this is a theoretical
ceiling, not a promised real frame rate. Sensor output remains approximately
30 FPS. The software uses one-frame bounded prefetch and one JPEG encoding when
YOLO is disabled. YOLO remains disabled by default.

## Register contract

AXI4-Lite base `0xA1000000`, map length 1 MiB. All accesses are aligned 32-bit
volatile loads/stores with barriers; never `struct.pack_into` or byte writes.

| Offset | Meaning |
|---|---|
| 0x00 | FRM2 ID `0x46524d32` |
| 0x04 | ABI `0x20000` |
| 0x08 | command: 1 start continuous; 2 release; 4 abort; 8 claim latest |
| 0x0c | active bit0, claimed bit1, sticky error bit2, latest READY bit3, phase7:4 |
| 0x10 | claimed sequence if claimed, else latest sequence |
| 0x14 | 614400 frame bytes |
| 0x18 / 0x1c | rejected frames / FIFO overflow counters |
| 0x20 | 307200 pixels when claimed; zero otherwise |
| 0x24 | `(480<<16) | 640` |
| 0x28 | last error (1 source invalid, 2 geometry, 3 timeout) |
| 0x2c | current capture phase ticks |
| 0x30 | successful complete frame count |
| 0x34 | unclaimed READY frames replaced by newer frames |
| 0x38 | claimed bank 0..2, or `0xffffffff` |
| 0x3c | latest complete frame sequence |
| 0x40 / 0x48 | claimed frame age in AXI cycles, low / high |
| 0x44 | nominal AXI frequency 100000000 Hz |
| 0x1000..0x96fff | claimed frame pixels, little endian RGB565, row major |

Claim during frame completion selects the previously ready complete frame; the
newly completed frame becomes the next READY. Data reads without a claim return
SLVERR. Release/abort during an unfinished AXI read return SLVERR and retain
ownership. Normal software only releases after its C copy returns with a barrier.

Read age high/low/high and retry on a high-word change. EOF time reconstructed
from the age counter is approximate and is not exposure time. Frames more than
250 ms old are rejected. Historical error flags/counters do not mean a subsequent
valid snapshot is invalid; partial images are never claimable.

## Verification / build

```powershell
python sim/test_stream_grabber.py -v
python sim/test_capture_pump.py -v
vivado -mode batch -source vivado/create_project.tcl
vivado -mode batch -source vivado/resume_adapter.tcl
```

The build is isolated in `build_stream/`; final artifacts go in `output/` only
after setup/hold checks pass. Vivado 2022.1 XPM EMPTY_CHECK has the same known
reset-model false assertion as the baseline; vendor sources are untouched,
with local FIFO/ownership assertions and full functional checks retained.

Deployment and real frame-rate results belong in the dated workspace handoff.
Do not install a candidate with failing simulation, DRC or timing. Stop the
camera service before replacing the FPGA overlay; active localization, mission,
EGO or any UART owner must block replacement. Recheck the UART device after load.
Firmware and camera entry checksum must match. Services remain manual, not
enabled at boot. For the combined camera/UART/servo design use
`../../hardware/servo/vivado/build_candidate.tcl` from the repository root.
