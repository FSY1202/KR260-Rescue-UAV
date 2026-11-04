set desktop_root [file normalize [file join [file dirname [info script]] ..]]
set gesture_root [file normalize [file join $desktop_root .. pynq_z2_gesture]]
set f [open [file join $gesture_root vivado build.tcl] r]
set script [read $f]; close $f
regsub -all {\mgesture\M} $script desktop_gesture script
set script [string map [list \
    {set root [file normalize [file join [file dirname [info script]] ..]]} [list set root $desktop_root] \
    {[file join $root boards]} [file join $gesture_root boards] \
    {add_files [file join $root rtl skin_stream.v]} [list add_files [list [file join $gesture_root rtl morph3_stream.v] [file join $gesture_root rtl gesture_v2.v] [file join $desktop_root vivado hdmi_scanout.v] [file join $desktop_root vivado board_inputs.v]]] \
    {-reference skin_stream skin_stream_0} {-reference gesture_pipeline_v2 gesture_pipeline_v2_0} \
    {CONFIG.c_s_axis_s2mm_tdata_width {32}} {CONFIG.c_s_axis_s2mm_tdata_width {8}} \
    {set_property CONFIG.NUM_MI 2 $ctrl} {set_property CONFIG.NUM_MI 5 $ctrl} \
    {CONFIG.NUM_SI {2}} {CONFIG.NUM_SI {3}} \
    {foreach cell [list $dma $gpio $skin $ctrl $mem]} {foreach cell [list $dma $gpio $config $skin $ctrl $mem $vdma $controls $inputs]} \
    {gesture_wrapper} {desktop_gesture_wrapper}] $script]
set insertion {
set_property ip_repo_paths [file join $root ip] [current_project]
update_ip_catalog
set_property XPM_LIBRARIES {XPM_CDC XPM_MEMORY XPM_FIFO} [current_project]
add_files -fileset constrs_1 [file join $root vivado hdmi.xdc]
}
set script [string map [list {create_bd_design desktop_gesture} "$insertion\ncreate_bd_design desktop_gesture"] $script]
set insertion {
set config [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:* processing_config]
set_property -dict [list CONFIG.C_IS_DUAL {1} CONFIG.C_GPIO_WIDTH {32} CONFIG.C_GPIO2_WIDTH {32} CONFIG.C_ALL_OUTPUTS {1} CONFIG.C_ALL_INPUTS_2 {1} CONFIG.C_DOUT_DEFAULT {0x00C000C0}] $config
set controls [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:* board_controls]
set_property -dict [list CONFIG.C_GPIO_WIDTH {6} CONFIG.C_ALL_INPUTS {1} CONFIG.C_INTERRUPT_PRESENT {0}] $controls
set inputs [create_bd_cell -type module -reference board_inputs board_inputs_0]
connect_bd_net [get_bd_pins $inputs/controls] [get_bd_pins $controls/gpio_io_i]
set buttons [create_bd_port -dir I -from 3 -to 0 buttons]
set switches [create_bd_port -dir I -from 1 -to 0 switches]
connect_bd_net $buttons [get_bd_pins $inputs/buttons]
connect_bd_net $switches [get_bd_pins $inputs/switches]
}
set script [string map [list {set ctrl [create_bd_cell} "$insertion\nset ctrl \[create_bd_cell"] $script]
set insertion {
connect_bd_intf_net [get_bd_intf_pins $ctrl/M02_AXI] [get_bd_intf_pins $config/S_AXI]
connect_bd_intf_net [get_bd_intf_pins $ctrl/M04_AXI] [get_bd_intf_pins $controls/S_AXI]
connect_bd_net [get_bd_pins $config/gpio_io_o] [get_bd_pins $skin/shape]
connect_bd_net [get_bd_pins $skin/ip_version] [get_bd_pins $config/gpio2_io_i]
set vdma [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_vdma:* hdmi_vdma]
set_property -dict [list CONFIG.c_include_s2mm {0} CONFIG.c_include_mm2s_dre {1} CONFIG.c_m_axi_mm2s_data_width {64} CONFIG.c_m_axis_mm2s_tdata_width {32} CONFIG.c_num_fstores {3} CONFIG.c_include_mm2s_genlock {0}] $vdma
connect_bd_intf_net [get_bd_intf_pins $ctrl/M03_AXI] [get_bd_intf_pins $vdma/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins $vdma/M_AXI_MM2S] [get_bd_intf_pins $mem/S02_AXI]
}
set script [string map [list {set reset [create_bd_cell} "$insertion\nset reset \[create_bd_cell"] $script]
set insertion {
set clocks [create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:* hdmi_clocks]
set_property -dict [list CONFIG.PRIM_IN_FREQ {100.000} CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {40.000} CONFIG.CLKOUT2_USED {true} CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {200.000} CONFIG.RESET_TYPE {ACTIVE_LOW}] $clocks
connect_bd_net $clock [get_bd_pins $clocks/clk_in1]
connect_bd_net [get_bd_pins $reset/peripheral_aresetn] [get_bd_pins $clocks/resetn]
set pixel_reset [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:* pixel_reset]
connect_bd_net [get_bd_pins $clocks/clk_out1] [get_bd_pins $pixel_reset/slowest_sync_clk]
connect_bd_net [get_bd_pins $clocks/locked] [get_bd_pins $pixel_reset/dcm_locked]
connect_bd_net [get_bd_pins $reset/peripheral_aresetn] [get_bd_pins $pixel_reset/ext_reset_in]
set raster [create_bd_cell -type module -reference hdmi_scanout hdmi_raster]
connect_bd_net $clock [get_bd_pins $raster/aclk]
connect_bd_net [get_bd_pins $reset/peripheral_aresetn] [get_bd_pins $raster/aresetn]
connect_bd_net [get_bd_pins $clocks/clk_out1] [get_bd_pins $raster/pixel_clk]
connect_bd_net [get_bd_pins $pixel_reset/peripheral_aresetn] [get_bd_pins $raster/pixel_resetn]
connect_bd_intf_net [get_bd_intf_pins $vdma/M_AXIS_MM2S] [get_bd_intf_pins $raster/S_AXIS]
set encoder [create_bd_cell -type ip -vlnv digilentinc.com:ip:rgb2dvi:1.2 hdmi_encoder]
set_property -dict [list CONFIG.kGenerateSerialClk {false} CONFIG.kRstActiveHigh {false}] $encoder
connect_bd_net [get_bd_pins $clocks/clk_out1] [get_bd_pins $encoder/PixelClk]
connect_bd_net [get_bd_pins $clocks/clk_out2] [get_bd_pins $encoder/SerialClk]
connect_bd_net [get_bd_pins $pixel_reset/peripheral_aresetn] [get_bd_pins $encoder/aRst_n]
foreach {source target} {rgb vid_pData de vid_pVDE hsync vid_pHSync vsync vid_pVSync} {
    connect_bd_net [get_bd_pins $raster/$source] [get_bd_pins $encoder/$target]
}
foreach {source name width kind} {TMDS_Clk_p hdmi_clk_p 1 clk TMDS_Clk_n hdmi_clk_n 1 clk TMDS_Data_p hdmi_data_p 3 data TMDS_Data_n hdmi_data_n 3 data} {
    if {$width == 1} {set port [create_bd_port -dir O $name]} else {set port [create_bd_port -dir O -from 2 -to 0 $name]}
    connect_bd_net [get_bd_pins $encoder/$source] $port
}
}
set script [string map [list {assign_bd_address} "$insertion\nassign_bd_address"] $script]
set script [string map [list {validate_bd_design} {set_property offset 0x42000000 [get_bd_addr_segs $ps/Data/SEG_processing_config_Reg]
set_property offset 0x43000000 [get_bd_addr_segs $ps/Data/SEG_hdmi_vdma_Reg]
set_property offset 0x44000000 [get_bd_addr_segs $ps/Data/SEG_board_controls_Reg]
validate_bd_design}] $script]
set script [string map [list {set_property offset 0x41200000 [get_bd_addr_segs $ps/Data/SEG_skin_bounds_Reg]} {set_property offset 0x44000000 [get_bd_addr_segs $ps/Data/SEG_board_controls_Reg]
set_property offset 0x42000000 [get_bd_addr_segs $ps/Data/SEG_processing_config_Reg]
set_property offset 0x41200000 [get_bd_addr_segs $ps/Data/SEG_skin_bounds_Reg]}] $script]
eval $script
