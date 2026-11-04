# Fast batch entry: create and validate the BD, without synthesis or implementation.
namespace eval ::servo_candidate_check {
    variable script_dir [file normalize [file dirname [info script]]]
    variable project_root [file normalize [file join $script_dir ..]]
    variable evidence_dir [file join $project_root evidence]
    file mkdir $evidence_dir
    namespace eval ::servo_candidate_create { variable bd_only 1 }
    source [file join $script_dir create_project.tcl]

    proc require {condition message} {
        if {![uplevel 1 [list expr $condition]]} { error $message }
    }
    proc same_net {left right} {
        set lnet [get_bd_nets -of_objects [get_bd_pins $left]]
        set rnet [get_bd_nets -of_objects [get_bd_pins $right]]
        require {$lnet ne "" && $lnet eq $rnet} "Net mismatch: $left / $right"
    }
    require {[get_property CONFIG.NUM_MI [get_bd_cells smartconnect_0]] == 5} "SmartConnect must have 5 masters"
    foreach {master cell} {M00_AXI color_detector_0 M01_AXI axi_iic_0 M02_AXI axi_uart16550_0 M03_AXI frame_capture_0 M04_AXI servo_pwm_0} {
        set mnet [get_bd_intf_nets -of_objects [get_bd_intf_pins smartconnect_0/$master]]
        set snet [get_bd_intf_nets -of_objects [get_bd_intf_pins $cell/S_AXI]]
        require {$mnet ne "" && $mnet eq $snet} "AXI master mapping mismatch for $cell"
    }
    set assigned [get_bd_addr_segs -of_objects [get_bd_addr_spaces zynq_ultra_ps_e_0/Data]]
    set rows {}
    foreach {cell offset size} {
        color_detector_0 0xA0000000 0x10000
        axi_iic_0 0xA0010000 0x10000
        axi_uart16550_0 0xA0020000 0x10000
        frame_capture_0 0xA1000000 0x100000
        servo_pwm_0 0xA0030000 0x10000
    } {
        set matching {}
        foreach seg $assigned {
            if {[string match "*SEG_${cell}_*" $seg]} { lappend matching $seg }
        }
        require {[llength $matching] == 1} "Expected one assigned address segment for $cell, got $matching"
        set seg [lindex $matching 0]
        set actual_offset [get_property OFFSET $seg]
        set actual_range [get_property RANGE $seg]
        require {$actual_offset == $offset && $actual_range == $size} "Address mismatch for $cell: $actual_offset / $actual_range"
        lappend rows [format {    {"cell":"%s","offset":"0x%08X","bytes":%d}} $cell $actual_offset $actual_range]
    }
    require {[get_property CONFIG.NUM_PORTS [get_bd_cells irq_concat]] == 4} "IRQ concatenation changed"
    require {[get_property CONFIG.IN3_WIDTH [get_bd_cells irq_concat]] == 5} "Unused IRQ width changed"
    same_net color_detector_0/irq irq_concat/In0
    same_net axi_iic_0/iic2intc_irpt irq_concat/In1
    same_net axi_uart16550_0/ip2intc_irpt irq_concat/In2
    same_net irq_zero/dout irq_concat/In3
    same_net irq_concat/dout zynq_ultra_ps_e_0/pl_ps_irq0
    same_net servo_pwm_0/s_axi_aclk zynq_ultra_ps_e_0/pl_clk0
    same_net servo_pwm_0/s_axi_aresetn reset_sync/peripheral_aresetn
    validate_bd_design
    save_bd_design
    set report [open [file join $evidence_dir bd_validation.json] w]
    puts $report "\{"
    puts $report {  "status":"PASS",}
    puts $report {  "validation_scope":"Vivado BD only; synthesis, implementation, DRC and timing not run",}
    puts $report [format {  "vivado_version":"%s",} [version -short]]
    puts $report {  "smartconnect_masters":5,}
    puts $report {  "servo_irq":false,}
    puts $report {  "address_map":[}
    puts $report [join $rows ",\n"]
    puts $report {  ]}
    puts $report "\}"
    close $report
    puts "PASS: kr260_servo BD validation; original addresses/IRQ preserved; M04 servo A0030000/64K"
    close_project
}
exit
