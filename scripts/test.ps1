param(
    [switch]$BuildWheel
)

$ErrorActionPreference = 'Stop'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$env:PYTHONPATH = Join-Path $repoRoot 'src'

Push-Location $repoRoot
try {
    python -m unittest discover -s tests -v
    python -m spherecalib demo --iterations 200 --output results/demo.json
    if ($BuildWheel) {
        python -m pip wheel --no-deps --wheel-dir dist .
    }
}
finally {
    Pop-Location
}
