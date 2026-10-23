$ErrorActionPreference = 'Stop'
$script:RepoRoot = Split-Path $PSScriptRoot -Parent
function Import-SimSettings {
    $defaults = Get-Content -LiteralPath "$script:RepoRoot/config/defaults.json" -Raw | ConvertFrom-Json
    foreach ($item in $defaults.PSObject.Properties) {
        if (-not [Environment]::GetEnvironmentVariable($item.Name, 'Process')) {
            [Environment]::SetEnvironmentVariable($item.Name, [string]$item.Value, 'Process')
        }
    }
    $local = "$script:RepoRoot/config/local.json"
    if (Test-Path -LiteralPath $local) {
        $settings = Get-Content -LiteralPath $local -Raw | ConvertFrom-Json
        foreach ($item in $settings.PSObject.Properties) {
            [Environment]::SetEnvironmentVariable($item.Name, [string]$item.Value, 'Process')
        }
    }
    if (-not $env:ROS2_HOME) { $env:ROS2_HOME = "$script:RepoRoot/.deps/ros2" }
    if (-not $env:RM_WS) { $env:RM_WS = "$script:RepoRoot/.deps/ros2_ws" }
    $env:ROS_DISTRO = 'jazzy'
    # FastDDS auto-loads the one profile in the repository working directory.
    Remove-Item Env:FASTRTPS_DEFAULT_PROFILES_FILE -ErrorAction SilentlyContinue
    Remove-Item Env:FASTDDS_DEFAULT_PROFILES_FILE -ErrorAction SilentlyContinue
}
function Import-DevEnvironment([switch]$WithoutOverlay) {
    $vswhere = "${env:ProgramFiles(x86)}/Microsoft Visual Studio/Installer/vswhere.exe"
    if (-not (Test-Path -LiteralPath $vswhere)) { throw 'Install Visual Studio 2022 Build Tools with C++ desktop workload.' }
    $vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $vs) { throw 'MSVC x64 C++ tools not found.' }
    $rosSetup = "$env:ROS2_HOME/local_setup.bat"
    if (-not (Test-Path -LiteralPath $rosSetup)) { throw 'ROS 2 not found. Run scripts/bootstrap-windows.ps1 or set ROS2_HOME in config/local.json.' }
    $activation = @('@echo off', ('call "{0}\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul' -f $vs),
        'if errorlevel 1 exit /b 1', ('call "{0}" >nul' -f $rosSetup), 'if errorlevel 1 exit /b 1')
    if (-not $WithoutOverlay) {
        $overlay = "$env:RM_WS/install/local_setup.bat"
        if (-not (Test-Path -LiteralPath $overlay)) { throw 'rm_interfaces overlay missing. Run bootstrap-windows.ps1.' }
        $activation += ('call "{0}" >nul' -f $overlay)
        $activation += 'if errorlevel 1 exit /b 1'
    }
    $activation += 'set'
    $temp = Join-Path ([IO.Path]::GetTempPath()) ('rm-activate-' + [Guid]::NewGuid().ToString('N') + '.cmd')
    try {
        [IO.File]::WriteAllLines($temp, $activation, [Text.Encoding]::Default)
        $lines = & $env:ComSpec /d /c $temp
        if ($LASTEXITCODE -ne 0) { throw 'ROS/MSVC environment activation failed.' }
        foreach ($line in $lines) {
            if ($line -match '^([A-Za-z_][A-Za-z0-9_()]*)=(.*)$') {
                [Environment]::SetEnvironmentVariable($matches[1], $matches[2], 'Process')
            }
        }
    } finally { Remove-Item -LiteralPath $temp -ErrorAction SilentlyContinue }
    $pixiEnv = "$env:ROS2_HOME/.pixi/envs/default"
    $env:PATH = "$env:RM_WS/install/rm_interfaces/bin;$env:ROS2_HOME/bin;$env:ROS2_HOME/Library/bin;$pixiEnv;$pixiEnv/Scripts;$pixiEnv/Library/bin;$env:PATH"
    if (-not $env:LIBCLANG_PATH) { $env:LIBCLANG_PATH = "$env:ProgramFiles/LLVM/bin" }
    if (-not (Test-Path -LiteralPath "$env:LIBCLANG_PATH/libclang.dll")) { throw 'libclang.dll missing. Install LLVM or set LIBCLANG_PATH.' }
    if (-not $env:BINDGEN_EXTRA_CLANG_ARGS) {
        $env:BINDGEN_EXTRA_CLANG_ARGS = (($env:INCLUDE -split ';' | Where-Object { $_ } | ForEach-Object { '-isystem "' + $_ + '"' }) -join ' ')
    }
}
