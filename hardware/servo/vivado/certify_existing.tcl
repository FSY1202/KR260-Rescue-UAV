# Recheck an already completed local candidate; no synthesis, deployment or IO.
set candidate_script_dir [file normalize [file dirname [info script]]]
set candidate_root [file normalize [file join $candidate_script_dir ..]]
set_param general.maxThreads 2
open_project [file join $candidate_root build_servo kr260_servo.xpr]
source [file join $candidate_script_dir certify_candidate.tcl]
close_project
exit
