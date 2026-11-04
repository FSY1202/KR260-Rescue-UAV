# Reuse the board/DDR/DMA setup, without modifying V1 or its artifacts.
set file [open [file join [file dirname [info script]] build.tcl] r]
set script [read $file];close $file
regsub -all {\mgesture\M} $script gesture_v2 script
set script [string map [list \
 {[file join $root build]} {[file join $root build_v2]} \
 {[file join $root build } {[file join $root build_v2 } \
 {add_files [file join $root rtl skin_stream.v]} {add_files [list [file join $root rtl morph3_stream.v] [file join $root rtl gesture_v2.v]]} \
 {-reference skin_stream skin_stream_0} {-reference gesture_pipeline_v2 gesture_pipeline_v2_0} \
 {gesture_wrapper} {gesture_v2_wrapper} \
 {timing.rpt} {timing_v2.rpt} {utilization.rpt} {utilization_v2.rpt} \
 {CONFIG.c_s_axis_s2mm_tdata_width {32}} {CONFIG.c_s_axis_s2mm_tdata_width {8}} \
 {set_property CONFIG.NUM_MI 2 $ctrl} {set_property CONFIG.NUM_MI 3 $ctrl} \
 {foreach cell [list $dma $gpio $skin $ctrl $mem]} {foreach cell [list $dma $gpio $config $skin $ctrl $mem]}] $script]
set insertion {
set config [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:* processing_config]
set_property -dict [list CONFIG.C_IS_DUAL {1} CONFIG.C_GPIO_WIDTH {32} CONFIG.C_GPIO2_WIDTH {32} CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_ALL_INPUTS_2 {1} CONFIG.C_DOUT_DEFAULT {0x00C000C0}] $config
}
set script [string map [list {set ctrl [create_bd_cell} "$insertion\nset ctrl \[create_bd_cell"] $script]
set insertion {
connect_bd_intf_net [get_bd_intf_pins $ctrl/M02_AXI] [get_bd_intf_pins $config/S_AXI]
connect_bd_net [get_bd_pins $config/gpio_io_o] [get_bd_pins $skin/shape]
connect_bd_net [get_bd_pins $skin/ip_version] [get_bd_pins $config/gpio2_io_i]
}
set script [string map [list {set reset [create_bd_cell} "$insertion\nset reset \[create_bd_cell"] $script]
set script [string map [list {validate_bd_design} {set_property offset 0x41210000 [get_bd_addr_segs $ps/Data/SEG_processing_config_Reg]
validate_bd_design}] $script]
set script [string map [list {set_property offset 0x40400000} {set_property offset 0x42000000 [get_bd_addr_segs $ps/Data/SEG_processing_config_Reg]
set_property offset 0x40400000}] $script]
eval $script
