param([string]$Archive)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$manifest = Get-Content -LiteralPath "$repo/env/runtime.json" -Raw | ConvertFrom-Json
$downloads = "$repo/.downloads"
New-Item -ItemType Directory -Force -Path $downloads | Out-Null
if (-not $Archive) {
    $Archive = Join-Path $downloads $manifest.asset
    if (-not (Test-Path -LiteralPath $Archive)) {
        if (-not (Get-Command gh -ErrorAction SilentlyContinue)) {
            throw 'Install GitHub CLI and run gh auth login, or download the Release zip and pass -Archive <path>.'
        }
        & gh release download $manifest.tag --repo $manifest.repository --pattern $manifest.asset --dir $downloads
        if ($LASTEXITCODE -ne 0) { throw 'Release download failed. For a private repository, verify collaborator access and gh auth login.' }
    }
}
$hash = (Get-FileHash -LiteralPath $Archive -Algorithm SHA256).Hash.ToLowerInvariant()
if ($hash -ne $manifest.sha256) { throw 'Runtime archive SHA256 mismatch.' }
if (Test-Path -LiteralPath "$repo/.runtime/windows") { throw 'Runtime already installed. Move .runtime/windows aside before installing another version.' }
New-Item -ItemType Directory -Force -Path "$repo/.runtime" | Out-Null
Expand-Archive -LiteralPath $Archive -DestinationPath "$repo/.runtime"
if (-not (Test-Path -LiteralPath "$repo/.runtime/windows/daedalus.exe")) { throw 'Unexpected archive layout.' }
Write-Host 'Runtime installed. Run start_rune_simulator.bat.'
