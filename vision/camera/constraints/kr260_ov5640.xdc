## KR260 J21 Raspberry Pi HAT to OV5640 DVP mapping.
## Cross-check the physical header orientation against XTP743 before power-up.

set_property PACKAGE_PIN AC14 [get_ports cam_pclk]
set_property IOSTANDARD LVCMOS33 [get_ports cam_pclk]

set_property PACKAGE_PIN AB15 [get_ports {cam_data[0]}]
set_property PACKAGE_PIN AB14 [get_ports {cam_data[1]}]
set_property PACKAGE_PIN Y14  [get_ports {cam_data[2]}]
set_property PACKAGE_PIN Y13  [get_ports {cam_data[3]}]
set_property PACKAGE_PIN W12  [get_ports {cam_data[4]}]
set_property PACKAGE_PIN W11  [get_ports {cam_data[5]}]
set_property PACKAGE_PIN Y12  [get_ports {cam_data[6]}]
set_property PACKAGE_PIN AA12 [get_ports {cam_data[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {cam_data[*]}]

set_property PACKAGE_PIN Y9  [get_ports cam_vsync]
set_property PACKAGE_PIN AA8 [get_ports cam_href]
set_property IOSTANDARD LVCMOS33 [get_ports {cam_vsync cam_href}]

set_property PACKAGE_PIN AB10 [get_ports cam_reset_n]
set_property PACKAGE_PIN AB9  [get_ports cam_pwdn]
set_property IOSTANDARD LVCMOS33 [get_ports {cam_reset_n cam_pwdn}]

set_property PACKAGE_PIN AE15 [get_ports cam_iic_scl_io]
set_property PACKAGE_PIN AE14 [get_ports cam_iic_sda_io]
set_property IOSTANDARD LVCMOS33 [get_ports {cam_iic_scl_io cam_iic_sda_io}]

## Adapter board UART1. IO13 is RX as confirmed from the board net names.
## Cross-connect at the flight controller: KR260 TX -> FC RX, KR260 RX <- FC TX.
set_property PACKAGE_PIN AA13 [get_ports uart1_tx]
set_property PACKAGE_PIN AB13 [get_ports uart1_rx]
set_property IOSTANDARD LVCMOS33 [get_ports {uart1_tx uart1_rx}]
set_property SLEW SLOW [get_ports uart1_tx]
set_property DRIVE 8 [get_ports uart1_tx]

## The selected AC14 package pin is an HDGC-capable clock input.
create_clock -name cam_pclk_clk -period 20.833 [get_ports cam_pclk]

## Parallel data is launched by the camera and sampled on the PCLK rising edge.
## These bounds leave margin at the supported maximum 48 MHz PCLK.
set_input_delay -clock cam_pclk_clk -min 1.000 [get_ports {cam_data[*] cam_href cam_vsync}]
set_input_delay -clock cam_pclk_clk -max 8.000 [get_ports {cam_data[*] cam_href cam_vsync}]

## Camera and AXI clocks are asynchronous; explicit toggle synchronizers bridge them.
## The PS preset names PL_CLK0 "clk_pl_0" in its generated XDC.
set_clock_groups -asynchronous \
    -group [get_clocks cam_pclk_clk] \
    -group [get_clocks clk_pl_0]
