param([string]$CubeMX = 'C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX.exe')
$ErrorActionPreference = 'Stop'
$workspace = Split-Path $PSScriptRoot -Parent
$project = Join-Path $workspace 'firmware-f407'
$java = Join-Path (Split-Path $CubeMX -Parent) 'jre/bin/java.exe'
if (-not (Test-Path -LiteralPath $CubeMX) -or -not (Test-Path -LiteralPath $java)) {
    throw 'CubeMX and its bundled Java runtime are required.'
}
$runId = (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [Guid]::NewGuid().ToString('N').Substring(0, 8)
$stage = Join-Path $workspace ('build/f407-generation/' + $runId)
New-Item -ItemType Directory -Path $stage | Out-Null
$scriptFile = Join-Path $stage 'generate.txt'
$ioc = (Join-Path $project 'fetal-baby-f407.ioc').Replace('\','/')
$core = (Join-Path $project 'Core').Replace('\','/')
[IO.File]::WriteAllLines($scriptFile,
    [string[]]@("config load `"$ioc`"", "generate one_file_per_ip", "generate code `"$core`"", 'exit'),
    [Text.UTF8Encoding]::new($false))
$log = Join-Path $stage 'generate.log'
& $java -jar $CubeMX -q $scriptFile 2>&1 | Tee-Object -FilePath $log | Out-Null
if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $log -SimpleMatch 'Code succesfully generated')) {
    throw "F407 generation failed. Inspect $log"
}
Write-Output "F407 peripheral code generated; shared application sources remain outside CubeMX: $log"
