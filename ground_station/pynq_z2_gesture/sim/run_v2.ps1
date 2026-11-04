param([string]$VivadoBin = $env:VIVADO_BIN)
$ErrorActionPreference='Stop'
if (-not $VivadoBin) { throw 'Set VIVADO_BIN or pass -VivadoBin (Vivado 2022.1 bin)' }
$bin=$VivadoBin
$root=Split-Path -Parent $PSScriptRoot
$work=Join-Path $PSScriptRoot 'xsim_work_v2'
New-Item -ItemType Directory -Force -Path $work | Out-Null
Push-Location $work
try {
 & "$bin\xvlog.bat" -sv "$root\rtl\morph3_stream.v" "$root\rtl\gesture_v2.v" "$PSScriptRoot\tb_gesture_v2.sv"
 if($LASTEXITCODE){throw 'V2 compile failed'}
 & "$bin\xelab.bat" tb_gesture_v2 -s v2_tb
 if($LASTEXITCODE){throw 'V2 elaboration failed'}
 & "$bin\xsim.bat" v2_tb -runall
 if($LASTEXITCODE){throw 'V2 simulation failed'}
 if(-not (Select-String -Path xsim.log -Pattern 'PASS V2:')){throw 'V2 did not pass'}
}finally{Pop-Location}
