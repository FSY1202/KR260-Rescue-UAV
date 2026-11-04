## Schematic1 net names, mating connectivity confirmed by owner 2026-10-26.
set_property PACKAGE_PIN AG14 [get_ports cam_pclk] ;# GPIO4
set_property PACKAGE_PIN AB14 [get_ports {cam_data[0]}] ;# GPIO17
set_property PACKAGE_PIN Y14  [get_ports {cam_data[1]}] ;# GPIO18
set_property PACKAGE_PIN AB9  [get_ports {cam_data[2]}] ;# GPIO27
set_property PACKAGE_PIN Y12  [get_ports {cam_data[3]}] ;# GPIO22
set_property PACKAGE_PIN AA12 [get_ports {cam_data[4]}] ;# GPIO23
set_property PACKAGE_PIN Y9   [get_ports {cam_data[5]}] ;# GPIO24
set_property PACKAGE_PIN AE13 [get_ports {cam_data[6]}] ;# GPIO10
set_property PACKAGE_PIN AC13 [get_ports {cam_data[7]}] ;# GPIO9
set_property PACKAGE_PIN AF13 [get_ports cam_vsync] ;# GPIO11
set_property PACKAGE_PIN AA8  [get_ports cam_href] ;# GPIO25
set_property PACKAGE_PIN AC14 [get_ports cam_reset_n] ;# GPIO8
set_property PACKAGE_PIN AH13 [get_ports cam_pwdn] ;# GPIO7
set_property PACKAGE_PIN AE14 [get_ports cam_iic_scl_io] ;# GPIO3
set_property PACKAGE_PIN AE15 [get_ports cam_iic_sda_io] ;# GPIO2
set_property IOSTANDARD LVCMOS33 [get_ports {cam_*}]
## Preserve existing UART1 TX/RX.
set_property PACKAGE_PIN AA13 [get_ports uart1_tx]
set_property PACKAGE_PIN AB13 [get_ports uart1_rx]
set_property IOSTANDARD LVCMOS33 [get_ports {uart1_tx uart1_rx}]
set_property SLEW SLOW [get_ports uart1_tx]
set_property DRIVE 8 [get_ports uart1_tx]
create_clock -name cam_pclk_clk -period 20.833 [get_ports cam_pclk]
## 0x4740[5]=1: sensor updates data on the falling edge; sample rising.
## 1..8 ns are external clock-to-output + board-skew design bounds, not measurements.
set_input_delay -clock cam_pclk_clk -clock_fall -min 1.000 [get_ports {cam_data[*] cam_href cam_vsync}]
set_input_delay -clock cam_pclk_clk -clock_fall -max 8.000 [get_ports {cam_data[*] cam_href cam_vsync}]
set_clock_groups -asynchronous -group [get_clocks cam_pclk_clk] -group [get_clocks clk_pl_0]
## AG14 is regular HDIO, not HDGC. Permit only its general clock route.
## Timing remains mandatory; no DRC severities are globally downgraded.
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets -of_objects [get_pins cam_pclk_IBUF_inst/O]]
