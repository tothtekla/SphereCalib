param(
    [string]$Destination = 'data/shrec2022/repo'
)

$ErrorActionPreference = 'Stop'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$destinationPath = [IO.Path]::GetFullPath((Join-Path $repoRoot $Destination))
if (-not $destinationPath.StartsWith($repoRoot + [IO.Path]::DirectorySeparatorChar)) {
    throw 'Destination must remain inside the repository.'
}

if (-not (Test-Path -LiteralPath (Join-Path $destinationPath '.git'))) {
    git clone --depth 1 --filter=blob:none --sparse `
        https://github.com/chiararomanengo/SHREC2022.git $destinationPath
}

Push-Location $destinationPath
try {
    git sparse-checkout set dataset/test/GTpointCloud
    $gtRoot = Join-Path $destinationPath 'dataset/test/GTpointCloud'
    $ids = foreach ($file in Get-ChildItem -LiteralPath $gtRoot -Filter 'GTpointCloud*.txt') {
        $values = @(Get-Content -LiteralPath $file.FullName |
            ForEach-Object { $_ -split '\s+' } | Where-Object { $_ -ne '' })
        if ([int][double]$values[0] -eq 3) {
            [int]([regex]::Match($file.BaseName, '\d+').Value)
        }
    }
    $patterns = @('/dataset/test/GTpointCloud/')
    $patterns += $ids | ForEach-Object { "/dataset/test/pointCloud/pointCloud$_.txt" }
    git sparse-checkout set --no-cone -- $patterns
    Write-Output "SHREC 2022 sphere subset ready: $($ids.Count) cases at $destinationPath"
}
finally {
    Pop-Location
}
