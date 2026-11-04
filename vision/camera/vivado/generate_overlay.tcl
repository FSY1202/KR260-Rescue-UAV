set script_dir [file normalize [file dirname [info script]]]
set project_root [file normalize [file join $script_dir ..]]
set xsa_abs [file join $project_root output kr260_ov5640.xsa]
set xsa [file join output kr260_ov5640.xsa]
set out_dir [file join output device_tree]

# createdts invokes POSIX find; put the copy bundled with Vivado before
# Windows System32's incompatible find.exe.
set posix_bin {D:/Xilinx/Vivado/2022.1/tps/win64/git-2.16.2/usr/bin}
set ::env(PATH) "$posix_bin;$::env(PATH)"

# XSCT on Windows otherwise resolves `find` to C:/Windows/System32/find.exe.
rename exec native_exec
proc exec {args} {
    if {[lindex $args 0] eq "find"} {
        set args [lreplace $args 0 0 \
            {D:/Xilinx/Vivado/2022.1/tps/win64/git-2.16.2/usr/bin/find.exe}]
    }
    return [uplevel 1 [list native_exec {*}$args]]
}

if {![file exists $xsa_abs]} {
    error "Missing hardware platform: $xsa_abs"
}

cd $project_root
file delete -force $out_dir
createdts -hw $xsa \
    -zocl \
    -platform-name kr260_ov5640 \
    -git-branch xlnx_rel_v2022.1 \
    -overlay \
    -out $out_dir
puts "Device-tree sources written to $out_dir (compile deploy/kr260_ov5640.dts with dtc on Linux)"
