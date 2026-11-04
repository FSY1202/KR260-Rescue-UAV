set script_dir [file normalize [file dirname [info script]]]
set project_root [file normalize [file join $script_dir ..]]
open_project [file join $project_root build_stream kr260_ov5640.xpr]
set_property USED_IN_SYNTHESIS false [get_files [file join $project_root constraints adapter20261026.xdc]]
reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Implementation failed"}
open_run impl_1
set out_dir [file join $project_root output]
file mkdir $out_dir
report_timing_summary -delay_type min_max -report_unconstrained -file [file join $out_dir timing_summary.rpt]
report_drc -file [file join $out_dir drc.rpt]
report_io -file [file join $out_dir io.rpt]
report_clock_utilization -file [file join $out_dir clock_utilization.rpt]
if {[get_property SLACK [get_timing_paths -delay_type max -max_paths 1]] < 0} {error "Setup timing failed; do not deploy"}
if {[get_property SLACK [get_timing_paths -delay_type min -max_paths 1]] < 0} {error "Hold timing failed; do not deploy"}
set impl_dir [get_property DIRECTORY [get_runs impl_1]]
file copy -force [file join $impl_dir kr260_ov5640_bd_wrapper.bit] [file join $out_dir kr260_ov5640.bit]
file copy -force [file join $impl_dir kr260_ov5640_bd_wrapper.bin] [file join $out_dir kr260_ov5640.bit.bin]
write_hw_platform -fixed -include_bit -force -file [file join $out_dir kr260_ov5640.xsa]
puts "ADAPTER_BUILD_PASS"
exit
