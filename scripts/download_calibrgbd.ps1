param(
    [string]$Destination = 'data/calibrgbd'
)

$ErrorActionPreference = 'Stop'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$destinationPath = [IO.Path]::GetFullPath((Join-Path $repoRoot $Destination))
if (-not ($destinationPath -eq $repoRoot -or
          $destinationPath.StartsWith($repoRoot + [IO.Path]::DirectorySeparatorChar))) {
    throw 'Destination must remain inside the repository.'
}

$url = 'https://www.rfai.lifat.univ-tours.fr/PublicData/calibrgbd/CalibRGBD_fichiers/data/Synth.zip'
$expectedBytes = 191083527
$expectedSha256 = '7E4AE863FA614745FC9CD2308C0333775FD2A0E8D510540BB55FB52A0A7A7198'
$archivePath = Join-Path $destinationPath 'Synth.zip'
$extractPath = Join-Path $destinationPath 'Synth'

New-Item -ItemType Directory -Path $destinationPath -Force | Out-Null
if (-not (Test-Path -LiteralPath $archivePath)) {
    curl.exe -L --fail --retry 3 --output $archivePath $url
}
if ((Get-Item -LiteralPath $archivePath).Length -ne $expectedBytes) {
    throw 'CalibRGBD archive size does not match the verified 2019_03 download.'
}
if ((Get-FileHash -Algorithm SHA256 -LiteralPath $archivePath).Hash -ne $expectedSha256) {
    throw 'CalibRGBD archive SHA-256 mismatch.'
}

Add-Type -AssemblyName System.IO.Compression.FileSystem
$archive = [IO.Compression.ZipFile]::OpenRead($archivePath)
$targetRoot = [IO.Path]::GetFullPath($extractPath)
$invalid = @($archive.Entries | Where-Object {
    $entryPath = [IO.Path]::GetFullPath((Join-Path $targetRoot $_.FullName))
    -not ($entryPath -eq $targetRoot -or
          $entryPath.StartsWith($targetRoot + [IO.Path]::DirectorySeparatorChar))
})
$archive.Dispose()
if ($invalid.Count -gt 0) {
    throw 'Archive contains a path outside the extraction target.'
}

New-Item -ItemType Directory -Path $extractPath -Force | Out-Null
tar.exe -xf $archivePath -C $extractPath
Write-Output "CalibRGBD synthetic data ready at $extractPath"
