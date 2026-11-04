# Independent FRM2 + servo candidate. All sourced-script variables stay scoped.
namespace eval ::servo_candidate_create {
    variable script_dir [file normalize [file dirname [info script]]]
    variable project_root [file normalize [file join $script_dir ..]]
    variable build_dir [file normalize [file join $project_root build_servo]]
    variable frozen_dir [file join $project_root vendor base_frame_stream]
    variable bd_only
    if {![info exists bd_only]} { set bd_only 0 }

    create_project -force kr260_servo $build_dir -part xck26-sfvc784-2LV-c
    set_property board_part xilinx.com:kr260_som:part0:1.0 [current_project]
    set_property target_language Verilog [current_project]
    set_property simulator_language Mixed [current_project]
    set_property XPM_LIBRARIES {XPM_CDC XPM_FIFO XPM_MEMORY} [current_project]
    set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE true [get_runs impl_1]

    add_files [list \
        [file join $frozen_dir rtl color_detector_core.sv] \
        [file join $frozen_dir rtl color_detector_top.sv] \
        [file join $frozen_dir rtl color_detector_wrapper.v] \
        [file join $frozen_dir rtl dvp_frame_source.sv] \
        [file join $frozen_dir rtl frame_capture_axi.sv] \
        [file join $frozen_dir rtl frame_capture_wrapper.v] \
        [file join $project_root rtl servo_pwm_axi.v]]
    foreach xdc [list [file join $frozen_dir constraints adapter20261026.xdc] \
                       [file join $project_root constraints servo_pwm.xdc]] {
        add_files -fileset constrs_1 $xdc
        set_property PROCESSING_ORDER LATE [get_files $xdc]
        set_property USED_IN_SYNTHESIS false [get_files $xdc]
    }
    update_compile_order -fileset sources_1

    create_bd_design kr260_servo_bd
    set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:* zynq_ultra_ps_e_0]
    apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
        -config {apply_board_preset "1"} $ps
    set_property -dict [list \
        CONFIG.PSU__USE__M_AXI_GP0 {1} \
        CONFIG.PSU__USE__M_AXI_GP1 {0} \
        CONFIG.PSU__USE__IRQ0 {1} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100}] $ps

    set detector [create_bd_cell -type module -reference color_detector_wrapper color_detector_0]
    set capture [create_bd_cell -type module -reference frame_capture_wrapper frame_capture_0]
    set servo [create_bd_cell -type module -reference servo_pwm_axi servo_pwm_0]
    set iic [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_iic:* axi_iic_0]
    set uart [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_uart16550:* axi_uart16550_0]
    set_property -dict [list CONFIG.C_IS_A_16550 {16550}] $uart
    set interconnect [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:* smartconnect_0]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {5}] $interconnect
    set reset_sync [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:* reset_sync]
    set reset_invert [create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic:* reset_invert]
    set_property -dict [list CONFIG.C_OPERATION {not} CONFIG.C_SIZE {1}] $reset_invert
    set lock_one [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:* lock_one]
    set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {1}] $lock_one

    connect_bd_intf_net [get_bd_intf_pins $ps/M_AXI_HPM0_FPD] [get_bd_intf_pins $interconnect/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins $interconnect/M00_AXI] [get_bd_intf_pins $detector/S_AXI]
    connect_bd_intf_net [get_bd_intf_pins $interconnect/M01_AXI] [get_bd_intf_pins $iic/S_AXI]
    connect_bd_intf_net [get_bd_intf_pins $interconnect/M02_AXI] [get_bd_intf_pins $uart/S_AXI]
    connect_bd_intf_net [get_bd_intf_pins $interconnect/M03_AXI] [get_bd_intf_pins $capture/S_AXI]
    connect_bd_intf_net [get_bd_intf_pins $interconnect/M04_AXI] [get_bd_intf_pins $servo/S_AXI]
    connect_bd_net [get_bd_pins $ps/pl_clk0] \
        [get_bd_pins $ps/maxihpm0_fpd_aclk] \
        [get_bd_pins $interconnect/aclk] \
        [get_bd_pins $detector/s_axi_aclk] \
        [get_bd_pins $capture/s_axi_aclk] \
        [get_bd_pins $servo/s_axi_aclk] \
        [get_bd_pins $iic/s_axi_aclk] \
        [get_bd_pins $uart/s_axi_aclk] \
        [get_bd_pins $reset_sync/slowest_sync_clk]
    connect_bd_net [get_bd_pins $ps/pl_resetn0] [get_bd_pins $reset_invert/Op1]
    connect_bd_net [get_bd_pins $reset_invert/Res] [get_bd_pins $reset_sync/ext_reset_in]
    connect_bd_net [get_bd_pins $lock_one/dout] [get_bd_pins $reset_sync/dcm_locked]
    connect_bd_net [get_bd_pins $reset_sync/peripheral_aresetn] \
        [get_bd_pins $interconnect/aresetn] \
        [get_bd_pins $detector/s_axi_aresetn] \
        [get_bd_pins $capture/s_axi_aresetn] \
        [get_bd_pins $servo/s_axi_aresetn] \
        [get_bd_pins $iic/s_axi_aresetn] \
        [get_bd_pins $uart/s_axi_aresetn]

    # Original IRQ bit assignment is unchanged; servo is polled and has no IRQ.
    set irq_concat [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:* irq_concat]
    set_property -dict [list CONFIG.NUM_PORTS {4} CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} CONFIG.IN3_WIDTH {5}] $irq_concat
    set irq_zero [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:* irq_zero]
    set_property -dict [list CONFIG.CONST_WIDTH {5} CONFIG.CONST_VAL {0}] $irq_zero
    connect_bd_net [get_bd_pins $detector/irq] [get_bd_pins $irq_concat/In0]
    connect_bd_net [get_bd_pins $iic/iic2intc_irpt] [get_bd_pins $irq_concat/In1]
    connect_bd_net [get_bd_pins $uart/ip2intc_irpt] [get_bd_pins $irq_concat/In2]
    connect_bd_net [get_bd_pins $irq_zero/dout] [get_bd_pins $irq_concat/In3]
    connect_bd_net [get_bd_pins $irq_concat/dout] [get_bd_pins $ps/pl_ps_irq0]

    set pclk_port [create_bd_port -dir I -type clk -freq_hz 48000000 cam_pclk]
    connect_bd_net $pclk_port [get_bd_pins $detector/cam_pclk]
    connect_bd_net $pclk_port [get_bd_pins $capture/cam_pclk]
    set data_port [create_bd_port -dir I -from 7 -to 0 cam_data]
    connect_bd_net $data_port [get_bd_pins $detector/cam_data]
    connect_bd_net $data_port [get_bd_pins $capture/cam_data]
    set href_port [create_bd_port -dir I cam_href]
    connect_bd_net $href_port [get_bd_pins $detector/cam_href]
    connect_bd_net $href_port [get_bd_pins $capture/cam_href]
    set vsync_port [create_bd_port -dir I cam_vsync]
    connect_bd_net $vsync_port [get_bd_pins $detector/cam_vsync]
    connect_bd_net $vsync_port [get_bd_pins $capture/cam_vsync]
    set reset_port [create_bd_port -dir O cam_reset_n]
    connect_bd_net $reset_port [get_bd_pins $detector/cam_reset_n]
    set pwdn_port [create_bd_port -dir O cam_pwdn]
    connect_bd_net $pwdn_port [get_bd_pins $detector/cam_pwdn]
    make_bd_intf_pins_external [get_bd_intf_pins $iic/IIC]
    set_property name cam_iic [get_bd_intf_ports IIC_0]

    # Preserve adapter UART1: GPIO12 is TX, GPIO13 is RX.
    set uart_tx_port [create_bd_port -dir O uart1_tx]
    connect_bd_net $uart_tx_port [get_bd_pins $uart/sout]
    set uart_rx_port [create_bd_port -dir I uart1_rx]
    connect_bd_net $uart_rx_port [get_bd_pins $uart/sin]
    set uart_modem_inactive [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:* uart_modem_inactive]
    set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {1}] $uart_modem_inactive
    connect_bd_net [get_bd_pins $uart_modem_inactive/dout] \
        [get_bd_pins $uart/ctsn] [get_bd_pins $uart/dcdn] [get_bd_pins $uart/dsrn] [get_bd_pins $uart/rin]
    set uart_freeze_zero [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:* uart_freeze_zero]
    set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {0}] $uart_freeze_zero
    connect_bd_net [get_bd_pins $uart_freeze_zero/dout] [get_bd_pins $uart/freeze]

    set servo0_port [create_bd_port -dir O servo0_pwm]
    connect_bd_net $servo0_port [get_bd_pins $servo/servo0_pwm]
    set servo1_port [create_bd_port -dir O servo1_pwm]
    connect_bd_net $servo1_port [get_bd_pins $servo/servo1_pwm]

    foreach {cell offset size} [list \
        $detector 0xA0000000 64K \
        $iic 0xA0010000 64K \
        $uart 0xA0020000 64K \
        $capture 0xA1000000 1M \
        $servo 0xA0030000 64K] {
        set seg [lindex [get_bd_addr_segs -of_objects [get_bd_intf_pins $cell/S_AXI]] 0]
        if {$seg eq ""} { error "No address segment for $cell/S_AXI" }
        assign_bd_address -target_address_space [get_bd_addr_spaces $ps/Data] \
            $seg -offset $offset -range $size
    }
    validate_bd_design
    save_bd_design
    if {!$bd_only} {
        generate_target all [get_files kr260_servo_bd.bd]
        set wrappers [make_wrapper -files [get_files kr260_servo_bd.bd] -top]
        add_files -norecurse $wrappers
        set_property top kr260_servo_bd_wrapper [current_fileset]
        update_compile_order -fileset sources_1
    }
    puts "Created independent candidate: [file join $build_dir kr260_servo.xpr] (BD-only=$bd_only)"
}
