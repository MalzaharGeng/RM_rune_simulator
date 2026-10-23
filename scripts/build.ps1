param([switch]$Test, [switch]$Check)
. "$PSScriptRoot/common.ps1"
Import-SimSettings
Import-DevEnvironment
Push-Location $script:RepoRoot
try {
    if ($Test) { & cargo test --release --locked }
    elseif ($Check) { & cargo check --release --locked }
    else { & cargo build --release --locked }
    if ($LASTEXITCODE -ne 0) { throw "Cargo failed: $LASTEXITCODE" }
} finally { Pop-Location }
