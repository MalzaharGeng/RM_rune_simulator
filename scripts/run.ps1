param([switch]$Source)
. "$PSScriptRoot/common.ps1"
Import-SimSettings
if ($Source) {
    Import-DevEnvironment
    $exe = "$script:RepoRoot/target/release/daedalus.exe"
} else {
    $runtime = "$script:RepoRoot/.runtime/windows"
    $exe = "$runtime/daedalus.exe"
    $env:PATH = "$runtime/bin;$env:SystemRoot/System32;$env:SystemRoot"
    $env:AMENT_PREFIX_PATH = "$runtime/ros;$runtime/rm_interfaces"
    $env:CMAKE_PREFIX_PATH = $env:AMENT_PREFIX_PATH
}
if (-not (Test-Path -LiteralPath $exe)) {
    throw 'Executable missing. Run scripts/download-runtime.ps1, or build.ps1 then run.ps1 -Source.'
}
Push-Location $script:RepoRoot
try { & $exe; $simExit = $LASTEXITCODE } finally { Pop-Location }
exit $simExit
