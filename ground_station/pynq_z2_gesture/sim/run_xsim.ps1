param([string]$VivadoBin = $env:VIVADO_BIN)
$ErrorActionPreference='Stop'
if (-not $VivadoBin) { throw 'Set VIVADO_BIN or pass -VivadoBin (Vivado 2022.1 bin)' }
$bin=$VivadoBin
$root=Split-Path -Parent $PSScriptRoot
$work=Join-Path $PSScriptRoot 'xsim_work'
New-Item -ItemType Directory -Force -Path $work | Out-Null
Push-Location $work
try {
 & "$bin\xvlog.bat" -sv "$root\rtl\skin_stream.v" "$PSScriptRoot\tb_skin_stream.sv"
 if($LASTEXITCODE){throw 'RTL compile failed'}
 & "$bin\xelab.bat" tb_skin_stream -s gesture_tb
 if($LASTEXITCODE){throw 'RTL elaboration failed'}
 & "$bin\xsim.bat" gesture_tb -runall
 if($LASTEXITCODE){throw 'RTL simulation failed'}
} finally {Pop-Location}
