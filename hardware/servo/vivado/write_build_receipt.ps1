$ErrorActionPreference = 'Stop'
$candidateRoot = Split-Path -Parent $PSScriptRoot
$candidateEvidence = Join-Path $candidateRoot 'evidence'
$candidateOutput = Join-Path $candidateRoot 'output'
function Get-CandidateSha256([string]$Path) {
    # Vivado changes PSModulePath; avoid relying on Get-FileHash autoload.
    $candidateStream = [System.IO.File]::OpenRead($Path)
    $candidateHasher = [System.Security.Cryptography.SHA256]::Create()
    try {
        return [System.BitConverter]::ToString($candidateHasher.ComputeHash($candidateStream)).Replace('-', '').ToLowerInvariant()
    } finally {
        $candidateHasher.Dispose()
        $candidateStream.Dispose()
    }
}
$metrics = [ordered]@{}
Import-Csv -Delimiter "`t" -LiteralPath (Join-Path $candidateEvidence 'implementation_metrics.tsv') | ForEach-Object {
    $metrics[$_.key] = $_.value
}
$resourceText = Get-Content -Raw -LiteralPath (Join-Path $candidateEvidence 'utilization.rpt')
$resources = [ordered]@{}
foreach ($resource in @('CLB LUTs', 'CLB Registers', 'Block RAM Tile', 'URAM', 'DSPs', 'Bonded IOB', 'BUFGCE', 'BUFG_PS')) {
    $resourceMatch = [regex]::Match($resourceText, '(?m)^\|\s*' + [regex]::Escape($resource) + '\s*\|\s*([0-9.]+)\s*\|')
    if (!$resourceMatch.Success) { throw "Missing resource row: $resource" }
    $resources[$resource] = $resourceMatch.Groups[1].Value
}
$inputs = @()
foreach ($sourceDir in @('rtl', 'constraints', 'vendor/base_frame_stream/rtl', 'vendor/base_frame_stream/constraints', 'vivado')) {
    Get-ChildItem -LiteralPath (Join-Path $candidateRoot $sourceDir) -File | Sort-Object Name | ForEach-Object {
        $inputs += [ordered]@{
            file = "$sourceDir/$($_.Name)"
            sha256 = Get-CandidateSha256 $_.FullName
        }
    }
}
$artifacts = @()
$shaLines = @()
foreach ($artifactName in @('kr260_servo.bit', 'kr260_servo.bit.bin', 'kr260_servo.xsa')) {
    $artifactPath = Join-Path $candidateOutput $artifactName
    $artifactFile = Get-Item -LiteralPath $artifactPath
    $artifactHash = Get-CandidateSha256 $artifactPath
    $artifacts += [ordered]@{ file = "output/$artifactName"; bytes = $artifactFile.Length; sha256 = $artifactHash }
    $shaLines += "$artifactHash  $artifactName"
}
$receipt = [ordered]@{
    schema_version = 1
    scope = 'Independent FRM2 + servo candidate; local build only, not deployed or board tested'
    full_build_log = 'evidence/build_candidate.log'
    certification_log = 'evidence/certify_candidate.log'
    metrics = $metrics
    utilization_used = $resources
    pins = @(Import-Csv -Delimiter "`t" -LiteralPath (Join-Path $candidateEvidence 'pin_contract.tsv'))
    inputs = $inputs
    artifacts = $artifacts
}
$receiptJson = ($receipt | ConvertTo-Json -Depth 8) + [Environment]::NewLine
[System.IO.File]::WriteAllText((Join-Path $candidateEvidence 'build_receipt.json'), $receiptJson, [System.Text.UTF8Encoding]::new($false))
$shaLines | Set-Content -Encoding ASCII -LiteralPath (Join-Path $candidateOutput 'SHA256SUMS')
