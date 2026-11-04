# Explicit full-build entry. Produces candidate files only; never deploys them.
namespace eval ::servo_candidate_build {
    variable script_dir [file normalize [file dirname [info script]]]
    variable project_root [file normalize [file join $script_dir ..]]
    variable output_dir [file normalize [file join $project_root output]]
    variable evidence_dir [file normalize [file join $project_root evidence]]
    file mkdir $evidence_dir
    namespace eval ::servo_candidate_create { variable bd_only 0 }
    source [file join $script_dir create_project.tcl]
    # The create script has its own namespace; script_dir/output_dir stay local.
    # 16 GiB host: bound parallel runs/threads to keep full implementation stable.
    set_param general.maxThreads 2
    launch_runs impl_1 -to_step write_bitstream -jobs 2
    wait_on_run impl_1
    source [file join $script_dir certify_candidate.tcl]
    close_project
}
exit
