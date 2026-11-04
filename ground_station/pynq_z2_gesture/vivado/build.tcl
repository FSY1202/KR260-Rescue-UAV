set root [file normalize [file join [file dirname [info script]] ..]]
set out [file join $root output]
file mkdir $out
set_param general.maxThreads 8
set_param board.repoPaths [list [file join $root boards]]
create_project gesture [file join $root build] -part xc7z020clg400-1 -force
set board [lindex [get_board_parts -quiet tul.com.tw:pynq-z2:part0:*] 0]
if {$board eq ""} {error "PYNQ-Z2 board files missing under $root/boards"}
set_property board_part $board [current_project]
add_files [file join $root rtl skin_stream.v]
update_compile_order -fileset sources_1
create_bd_design gesture
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:* processing_system7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps
set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {1} CONFIG.PCW_USE_S_AXI_HP0 {1} CONFIG.PCW_EN_CLK0_PORT {1} CONFIG.PCW_EN_RST0_PORT {1} CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100}] $ps
set dma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:* axi_dma_0]
set_property -dict [list CONFIG.c_include_sg {0} CONFIG.c_sg_length_width {26} CONFIG.c_m_axi_mm2s_data_width {64} CONFIG.c_m_axi_s2mm_data_width {64} CONFIG.c_m_axis_mm2s_tdata_width {32} CONFIG.c_s_axis_s2mm_tdata_width {32} CONFIG.c_include_mm2s_dre {1} CONFIG.c_include_s2mm_dre {1}] $dma
set gpio [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:* skin_bounds]
set_property -dict [list CONFIG.C_IS_DUAL {1} CONFIG.C_GPIO_WIDTH {32} CONFIG.C_GPIO2_WIDTH {8} CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_ALL_OUTPUTS_2 {1} CONFIG.C_DOUT_DEFAULT {0xAD857F4D} CONFIG.C_DOUT_DEFAULT_2 {0x00000028}] $gpio
set skin [create_bd_cell -type module -reference skin_stream skin_stream_0]
set ctrl [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:* control_bus]
set_property CONFIG.NUM_MI 2 $ctrl
set mem [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:* memory_bus]
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] $mem
connect_bd_intf_net [get_bd_intf_pins $ps/M_AXI_GP0] [get_bd_intf_pins $ctrl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins $ctrl/M00_AXI] [get_bd_intf_pins $dma/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins $ctrl/M01_AXI] [get_bd_intf_pins $gpio/S_AXI]
connect_bd_intf_net [get_bd_intf_pins $dma/M_AXI_MM2S] [get_bd_intf_pins $mem/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins $dma/M_AXI_S2MM] [get_bd_intf_pins $mem/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins $mem/M00_AXI] [get_bd_intf_pins $ps/S_AXI_HP0]
connect_bd_intf_net [get_bd_intf_pins $dma/M_AXIS_MM2S] [get_bd_intf_pins $skin/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins $skin/M_AXIS] [get_bd_intf_pins $dma/S_AXIS_S2MM]
connect_bd_net [get_bd_pins $gpio/gpio_io_o] [get_bd_pins $skin/bounds]
connect_bd_net [get_bd_pins $gpio/gpio2_io_o] [get_bd_pins $skin/y_min]
set reset [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:* reset_0]
set_property CONFIG.C_EXT_RESET_HIGH 0 $reset
set one [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:* one]
set_property CONFIG.CONST_VAL 1 $one
connect_bd_net [get_bd_pins $one/dout] [get_bd_pins $reset/dcm_locked]
connect_bd_net [get_bd_pins $ps/FCLK_RESET0_N] [get_bd_pins $reset/ext_reset_in]
set clock [get_bd_pins $ps/FCLK_CLK0]
connect_bd_net $clock [get_bd_pins $ps/M_AXI_GP0_ACLK] [get_bd_pins $ps/S_AXI_HP0_ACLK] [get_bd_pins $reset/slowest_sync_clk]
foreach cell [list $dma $gpio $skin $ctrl $mem] {
    foreach pin [get_bd_pins -of_objects $cell -filter {TYPE == clk && DIR == I}] {connect_bd_net $clock $pin}
    foreach pin [get_bd_pins -of_objects $cell -filter {TYPE == rst && DIR == I}] {connect_bd_net [get_bd_pins $reset/peripheral_aresetn] $pin}
}
assign_bd_address
set_property offset 0x40400000 [get_bd_addr_segs $ps/Data/SEG_axi_dma_0_Reg]
set_property offset 0x41200000 [get_bd_addr_segs $ps/Data/SEG_skin_bounds_Reg]
validate_bd_design
save_bd_design
generate_target all [get_files gesture.bd]
make_wrapper -files [get_files gesture.bd] -top
add_files [glob [file join $root build gesture.gen sources_1 bd gesture hdl gesture_wrapper.v]]
set_property top gesture_wrapper [current_fileset]
update_compile_order -fileset sources_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Implementation failed"}
open_run impl_1
report_timing_summary -file [file join $out timing.rpt]
report_utilization -file [file join $out utilization.rpt]
set wns [get_property STATS.WNS [get_runs impl_1]]
if {$wns eq "" || $wns < 0} {error "Timing not met: WNS=$wns"}
file copy -force [file join $root build gesture.runs impl_1 gesture_wrapper.bit] [file join $out gesture.bit]
file copy -force [file join $root build gesture.gen sources_1 bd gesture hw_handoff gesture.hwh] [file join $out gesture.hwh]
puts "BUILD PASSED: $out/gesture.bit; WNS=$wns"
