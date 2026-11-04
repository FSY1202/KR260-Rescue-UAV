param(
    [string]$VivadoBin = 'D:\Xilinx\Vivado\2022.1\bin'
)
$ErrorActionPreference = 'Stop'
$taskSimDir = $PSScriptRoot
$taskBuildDir = Join-Path $taskSimDir 'build'
New-Item -ItemType Directory -Force -Path $taskBuildDir | Out-Null
Push-Location -LiteralPath $taskBuildDir
try {
    $taskIverilog = Get-Command iverilog -ErrorAction SilentlyContinue
    $taskVvp = Get-Command vvp -ErrorAction SilentlyContinue
    if ($taskIverilog -and $taskVvp) {
        & $taskIverilog.Source -g2012 -s tb_servo_pwm_axi -o servo_pwm_tb.vvp '../tb_servo_pwm_axi.sv' '../../rtl/servo_pwm_axi.v'
        if ($LASTEXITCODE -ne 0) { throw 'iverilog compilation failed' }
        & $taskVvp.Source servo_pwm_tb.vvp
        if ($LASTEXITCODE -ne 0) { throw 'vvp simulation failed' }
    } else {
        foreach ($taskTool in @('xvlog.bat', 'xelab.bat', 'xsim.bat')) {
            if (-not (Test-Path -LiteralPath (Join-Path $VivadoBin $taskTool))) { throw "Missing simulation tool: $taskTool in $VivadoBin" }
        }
        & (Join-Path $VivadoBin 'xvlog.bat') --sv '../../rtl/servo_pwm_axi.v' '../tb_servo_pwm_axi.sv'
        if ($LASTEXITCODE -ne 0) { throw 'xvlog compilation failed' }
        & (Join-Path $VivadoBin 'xelab.bat') tb_servo_pwm_axi -s servo_pwm_tb
        if ($LASTEXITCODE -ne 0) { throw 'xelab elaboration failed' }
        & (Join-Path $VivadoBin 'xsim.bat') servo_pwm_tb -tclbatch '../run_sim.tcl'
        if ($LASTEXITCODE -ne 0) { throw 'xsim simulation failed' }
        # xsim can return 0 even after a SystemVerilog $fatal; require PASS.
        $taskLog = Get-Content -Raw -LiteralPath 'xsim.log'
        if ($taskLog -notmatch 'PASS: servo_pwm_axi' -or $taskLog -match 'FAIL at|simulation watchdog expired|Fatal:') {
            throw 'Simulation did not report PASS; inspect sim/build/xsim.log'
        }
    }
} finally {
    Pop-Location
}
