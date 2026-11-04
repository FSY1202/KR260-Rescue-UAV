# Certify the completed independent implementation, then publish local artifacts.
# Can be sourced after open_project/open_run, or directly after the full build.
namespace eval ::servo_candidate_certify {
    variable script_dir [file normalize [file dirname [info script]]]
    variable project_root [file normalize [file join $script_dir ..]]
    variable output_dir [file join $project_root output]
    variable evidence_dir [file join $project_root evidence]
    file mkdir $evidence_dir
    if {[get_property PROGRESS [get_runs impl_1]] ne "100%" ||
        ![string match "*write_bitstream Complete*" [get_property STATUS [get_runs impl_1]]]} {
        error "Candidate implementation did not complete: [get_property STATUS [get_runs impl_1]]"
    }
    open_run impl_1
    report_timing_summary -delay_type min_max -check_timing_verbose \
        -file [file join $evidence_dir timing_summary.rpt]
    report_drc -file [file join $evidence_dir drc.rpt]
    report_io -file [file join $evidence_dir io.rpt]
    report_clock_utilization -file [file join $evidence_dir clock_utilization.rpt]
    report_utilization -file [file join $evidence_dir utilization.rpt]
    report_exceptions -file [file join $evidence_dir exceptions.rpt]
    report_bus_skew -file [file join $evidence_dir bus_skew.rpt]
    check_timing -verbose -file [file join $evidence_dir check_timing.rpt]
    set timing_file [open [file join $evidence_dir timing_summary.rpt] r]
    set timing_text [read $timing_file]
    close $timing_file
    if {![regexp -line {^\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+([0-9]+)\s+([0-9]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+([0-9]+)\s+([0-9]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+([0-9]+)\s+([0-9]+)\s*$} \
        $timing_text summary_line summary_wns summary_tns setup_fail setup_total \
        summary_whs summary_ths hold_fail hold_total summary_wpws summary_tpws pulse_fail pulse_total]} {
        error "Cannot read the implementation timing summary totals"
    }
    if {$setup_fail != 0 || $hold_fail != 0 || $pulse_fail != 0 || $summary_wpws < 0} {
        error "Candidate has failing timing endpoints: setup=$setup_fail hold=$hold_fail pulse=$pulse_fail"
    }
    set skew_file [open [file join $evidence_dir bus_skew.rpt] r]
    set skew_text [read $skew_file]
    close $skew_file
    if {[string match "*Slack (VIOLATED)*" $skew_text]} {
        error "Candidate violates a bus-skew constraint"
    }
    set skew_matches [regexp -all -inline {Slack \(MET\) :\s+([-0-9.]+)ns} $skew_text]
    if {[llength $skew_matches] == 0} { error "Cannot certify inherited XPM FIFO bus-skew constraints" }
    set worst_skew_slack 1000000
    foreach {match value} $skew_matches {
        if {$value < $worst_skew_slack} { set worst_skew_slack $value }
    }
    foreach delay_type {max min} {
        set paths [get_timing_paths -quiet -delay_type $delay_type -max_paths 1]
        if {[llength $paths] == 0} { error "No $delay_type timing path; cannot certify candidate timing" }
        set slack($delay_type) [get_property SLACK [lindex $paths 0]]
        if {$slack($delay_type) < 0} {
            error "Candidate $delay_type timing failed: worst slack $slack($delay_type) ns"
        }
        report_timing -delay_type $delay_type -from [get_ports {cam_data[*] cam_href cam_vsync}] \
            -max_paths 10 -file [file join $evidence_dir camera_input_${delay_type}_timing.rpt]
    }
    set blocking_drc 0
    set drc_warnings {}
    foreach violation [get_drc_violations -quiet] {
        set severity [string tolower [get_property SEVERITY $violation]]
        if {$severity eq "error" || $severity eq "critical warning"} {
            incr blocking_drc
        } elseif {$severity eq "warning"} {
            lappend drc_warnings $violation
            set rule [lindex [split $violation #] 0]
            if {[lsearch -exact {PLCK-58 PLCK-90 RTSTAT-10} $rule] < 0} {
                error "Unreviewed candidate DRC warning: $violation"
            }
        }
    }
    if {$blocking_drc != 0} { error "Candidate DRC failed: $blocking_drc blocking violations" }

    # Exact external IO contract: 17 original camera/UART pins plus two servos.
    set expected_pins [dict create cam_pclk AG14 \
        {cam_data[0]} AB14 {cam_data[1]} Y14 {cam_data[2]} AB9 {cam_data[3]} Y12 \
        {cam_data[4]} AA12 {cam_data[5]} Y9 {cam_data[6]} AE13 {cam_data[7]} AC13 \
        cam_vsync AF13 cam_href AA8 cam_reset_n AC14 cam_pwdn AH13 \
        cam_iic_scl_io AE14 cam_iic_sda_io AE15 uart1_tx AA13 uart1_rx AB13 \
        servo0_pwm Y13 servo1_pwm AB15]
    if {[llength [get_ports]] != [dict size $expected_pins]} {
        error "Unexpected external port count [llength [get_ports]]; expected [dict size $expected_pins]"
    }
    set pin_receipt [open [file join $evidence_dir pin_contract.tsv] w]
    puts $pin_receipt "port\tpackage_pin\tiostandard"
    dict for {port expected_pin} $expected_pins {
        set object [get_ports -quiet $port]
        if {[llength $object] != 1} { error "Missing/ambiguous port $port" }
        set actual_pin [get_property PACKAGE_PIN $object]
        set standard [get_property IOSTANDARD $object]
        if {$actual_pin ne $expected_pin || $standard ne "LVCMOS33"} {
            error "IO contract failed: $port uses $actual_pin/$standard, expected $expected_pin/LVCMOS33"
        }
        puts $pin_receipt "$port\t$actual_pin\t$standard"
    }
    close $pin_receipt
    set pclk_net [get_nets -of_objects [get_pins cam_pclk_IBUF_inst/O]]
    if {[string tolower [get_property CLOCK_DEDICATED_ROUTE $pclk_net]] ne "false"} {
        error "The documented AG14 camera clock routing exception was not preserved"
    }
    file mkdir $output_dir
    set impl_dir [get_property DIRECTORY [get_runs impl_1]]
    foreach {source_name output_name} {
        kr260_servo_bd_wrapper.bit kr260_servo.bit
        kr260_servo_bd_wrapper.bin kr260_servo.bit.bin
    } {
        set artifact [file join $impl_dir $source_name]
        if {![file exists $artifact]} { error "Missing candidate artifact $artifact" }
        file copy -force $artifact [file join $output_dir $output_name]
    }
    write_hw_platform -fixed -include_bit -force -file [file join $output_dir kr260_servo.xsa]
    set receipt [open [file join $evidence_dir implementation_metrics.tsv] w]
    puts $receipt "key\tvalue"
    puts $receipt "completed_utc\t[clock format [clock seconds] -gmt 1 -format {%Y-%m-%dT%H:%M:%SZ}]"
    puts $receipt "vivado\t[version -short]"
    puts $receipt "part\t[get_property PART [current_project]]"
    puts $receipt "implementation_status\t[get_property STATUS [get_runs impl_1]]"
    puts $receipt "wns_ns\t$slack(max)"
    puts $receipt "whs_ns\t$slack(min)"
    puts $receipt "tns_ns\t$summary_tns"
    puts $receipt "ths_ns\t$summary_ths"
    puts $receipt "wpws_ns\t$summary_wpws"
    puts $receipt "worst_bus_skew_slack_ns\t$worst_skew_slack"
    puts $receipt "blocking_drc_violations\t$blocking_drc"
    puts $receipt "drc_warning_count\t[llength $drc_warnings]"
    puts $receipt "drc_warning_names\t[join $drc_warnings {, }]"
    puts $receipt "external_pin_contract_count\t[dict size $expected_pins]"
    puts $receipt "camera_clock_period_ns\t[get_property PERIOD [get_clocks cam_pclk_clk]]"
    puts $receipt "camera_input_delay_edge\tfalling"
    puts $receipt "camera_input_delay_min_ns\t1.000"
    puts $receipt "camera_input_delay_max_ns\t8.000"
    puts $receipt "camera_clock_dedicated_route\tFALSE (AG14 regular HDIO; unchanged baseline exception)"
    close $receipt
    exec powershell.exe -NoProfile -ExecutionPolicy Bypass -File [file join $script_dir write_build_receipt.ps1]
    puts "PASS: candidate implementation/timing/DRC and all 19 pin contracts; artifacts only in $output_dir"
}
