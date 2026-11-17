param(
    [string]$CalibRGBDRoot = 'data/calibrgbd/Synth',
    [int]$Trials = 100,
    [int]$EqualProposalIterations = 300,
    [switch]$SkipPublicDataset,
    [string]$SHRECRoot = 'data/shrec2022/repo/dataset/test'
)

$ErrorActionPreference = 'Stop'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$env:PYTHONPATH = Join-Path $repoRoot 'src'

Push-Location $repoRoot
try {
    python -m unittest discover -s tests -v
    python -m spherecalib benchmark --trials $Trials --iterations $EqualProposalIterations `
        --budget equal-proposals --output results/synthetic_equal_proposals.csv
    python -m spherecalib benchmark --trials $Trials --budget equal-confidence `
        --confidence 0.999 --inlier-probability 0.75 --orientation-reliability 0.95 `
        --output results/synthetic_equal_confidence.csv
    python -m spherecalib example examples/example1 --radius-max 3 `
        --output results/example1.json
    if (-not $SkipPublicDataset) {
        if (-not (Test-Path -LiteralPath $CalibRGBDRoot)) {
            throw "CalibRGBD root not found: $CalibRGBDRoot"
        }
        python -m spherecalib evaluate-calibrgbd $CalibRGBDRoot `
            --output results/calibrgbd.csv
        if (Test-Path -LiteralPath $SHRECRoot) {
            python -m spherecalib evaluate-shrec $SHRECRoot `
                --output results/shrec2022.csv
        }
    }
}
finally {
    Pop-Location
}
