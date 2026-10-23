param([string]$RosArchive)
. "$PSScriptRoot/common.ps1"
Import-SimSettings
foreach ($command in @('pixi','rustup','cargo')) {
    if (-not (Get-Command $command -ErrorAction SilentlyContinue)) { throw "Install prerequisite first: $command. See docs/ENVIRONMENT.md." }
}
if (-not (Test-Path -LiteralPath "$env:ROS2_HOME/local_setup.bat")) {
    $manifest = Get-Content -LiteralPath "$script:RepoRoot/env/windows/ros2.json" -Raw | ConvertFrom-Json
    $downloads = "$script:RepoRoot/.downloads"
    New-Item -ItemType Directory -Force -Path $downloads | Out-Null
    if (-not $RosArchive) {
        $RosArchive = Join-Path $downloads $manifest.asset
        if (-not (Test-Path -LiteralPath $RosArchive)) { Invoke-WebRequest -UseBasicParsing -Uri $manifest.url -OutFile $RosArchive }
    }
    if ((Get-FileHash -LiteralPath $RosArchive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $manifest.sha256) { throw 'ROS 2 archive SHA256 mismatch.' }
    $unpack = "$script:RepoRoot/.deps/ros2-archive"
    if (Test-Path -LiteralPath $unpack) { throw 'Previous extraction exists at .deps/ros2-archive. Inspect and move it aside first.' }
    Expand-Archive -LiteralPath $RosArchive -DestinationPath $unpack
    $setup = Get-ChildItem -LiteralPath $unpack -Filter local_setup.bat -Recurse | Select-Object -First 1
    if (-not $setup) { throw 'ROS archive has no local_setup.bat.' }
    if (Test-Path -LiteralPath $env:ROS2_HOME) { throw 'ROS2_HOME exists but is incomplete. Choose an empty path.' }
    New-Item -ItemType Directory -Force -Path (Split-Path $env:ROS2_HOME -Parent) | Out-Null
    Move-Item -LiteralPath $setup.Directory.FullName -Destination $env:ROS2_HOME
    Copy-Item -LiteralPath "$script:RepoRoot/env/windows/pixi.toml","$script:RepoRoot/env/windows/pixi.lock" -Destination $env:ROS2_HOME
    & pixi install --manifest-path "$env:ROS2_HOME/pixi.toml" --locked
    if ($LASTEXITCODE -ne 0) { throw 'Pixi dependency installation failed.' }
}
Import-DevEnvironment -WithoutOverlay
$interfaceSource = "$env:RM_WS/src/rm_interfaces"
if (Test-Path -LiteralPath $interfaceSource) {
    $original = "$script:RepoRoot/rm_interfaces"
    foreach ($file in Get-ChildItem -LiteralPath $original -Recurse -File) {
        $relative = $file.FullName.Substring($original.Length).TrimStart('\','/')
        $existing = Join-Path $interfaceSource $relative
        if (-not (Test-Path -LiteralPath $existing)) { throw 'Existing rm_interfaces differs. Choose a new RM_WS.' }
        if ((Get-FileHash -LiteralPath $existing).Hash -ne (Get-FileHash -LiteralPath $file.FullName).Hash) {
            throw 'Existing rm_interfaces differs. Choose a new RM_WS to preserve local changes.'
        }
    }
} else {
    New-Item -ItemType Directory -Force -Path "$env:RM_WS/src" | Out-Null
    Copy-Item -LiteralPath "$script:RepoRoot/rm_interfaces" -Destination $interfaceSource -Recurse
}
$python = "$env:ROS2_HOME/.pixi/envs/default/python.exe"
if (-not (Test-Path -LiteralPath $python)) { throw 'Pixi Python missing. Run pixi install --locked in ROS2_HOME.' }
Push-Location $env:RM_WS
try {
    & $python -m colcon build --packages-select rm_interfaces --cmake-force-configure --cmake-args -DBUILD_TESTING=OFF "-DPython3_EXECUTABLE=$python" "-DPYTHON_EXECUTABLE=$python"
    if ($LASTEXITCODE -ne 0) { throw 'rm_interfaces build failed.' }
} finally { Pop-Location }
Write-Host 'Development environment ready. Run scripts/build.ps1.'
