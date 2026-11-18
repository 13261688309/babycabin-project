param(
    [string]$CubeMX = 'C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX.exe',
    [string]$Python = 'python',
    [string]$Compiler
)
$ErrorActionPreference = 'Stop'
$workspace = Split-Path $PSScriptRoot -Parent
$source = Join-Path $workspace 'fetal-baby'
$java = Join-Path (Split-Path $CubeMX -Parent) 'jre/bin/java.exe'
if (-not (Test-Path -LiteralPath $CubeMX) -or -not (Test-Path -LiteralPath $java)) {
    throw 'CubeMX and its bundled Java runtime are required.'
}
$runId = (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [Guid]::NewGuid().ToString('N').Substring(0, 8)
$stage = Join-Path $workspace ('build/cubemx/' + $runId)
$project = Join-Path $stage 'fetal-baby'
New-Item -ItemType Directory -Path $project | Out-Null
$names = @('Core','Drivers','BabyCabin','Middlewares','.cproject','.project','fetal-baby.ioc','STM32H743ZITX_FLASH.ld','STM32H743ZITX_RAM.ld')
$original = @{}
foreach ($name in $names) {
    $path = Join-Path $source $name
    $files = if (Test-Path -LiteralPath $path -PathType Container) {
        Get-ChildItem -LiteralPath $path -Recurse -File
    } else { Get-Item -LiteralPath $path }
    foreach ($file in $files) {
        $relative = $file.FullName.Substring($source.Length + 1)
        $original[$relative] = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash
    }
    Copy-Item -LiteralPath $path -Destination $project -Recurse
}
New-Item -ItemType Directory -Path (Join-Path $stage 'scripts') | Out-Null
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'build-firmware.ps1') -Destination (Join-Path $stage 'scripts')
$scriptFile = Join-Path $stage 'generate.txt'
$ioc = (Join-Path $project 'fetal-baby.ioc').Replace('\','/')
$core = (Join-Path $project 'Core').Replace('\','/')
[IO.File]::WriteAllLines($scriptFile,
    [string[]]@("config load `"$ioc`"", "generate one_file_per_ip", "generate code `"$core`"", 'exit'),
    [Text.UTF8Encoding]::new($false))
$log = Join-Path $stage 'generate.log'
Write-Output "Generating code in isolated project: $project"
& $java -jar $CubeMX -q $scriptFile 2>&1 | Tee-Object -FilePath $log | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'CubeMX process failed. Original project is unchanged.' }
$generated = @('Core/Src/main.c','Core/Inc/main.h','Core/Src/stm32h7xx_hal_msp.c',
    'Core/Src/stm32h7xx_it.c','Core/Inc/stm32h7xx_it.h','Core/Inc/stm32h7xx_hal_conf.h',
    'Core/Src/tim.c','Core/Inc/tim.h','Core/Src/usart.c','Core/Inc/usart.h','Core/Src/gpio.c','Core/Inc/gpio.h',
    'Core/Src/usb_device.c','Core/Inc/usb_device.h','Core/Src/usbd_conf.c','Core/Inc/usbd_conf.h',
    'Core/Src/usbd_desc.c','Core/Inc/usbd_desc.h','Core/Src/usbd_cdc_if.c','Core/Inc/usbd_cdc_if.h')
$allowed = @('fetal-baby.ioc') + $generated
$userBlock = '(?s)/\* USER CODE BEGIN (?<name>[^\r\n]*?) \*/(?<body>.*?)/\* USER CODE END \k<name> \*/'
foreach ($relative in $original.Keys) {
    $before = Join-Path $source $relative
    $after = Join-Path $project $relative
    if ((Get-FileHash -LiteralPath $before -Algorithm SHA256).Hash -ne $original[$relative]) {
        throw "Original project changed during validation: $relative"
    }
    if ($allowed -notcontains $relative.Replace('\','/')) {
        if ((Get-FileHash -LiteralPath $after -Algorithm SHA256).Hash -ne $original[$relative]) {
            throw "Unexpected generated change: $relative"
        }
    } elseif ($relative -match '\.(c|h)$') {
        $newText = Get-Content -LiteralPath $after -Raw
        foreach ($match in [regex]::Matches((Get-Content -LiteralPath $before -Raw), $userBlock)) {
            if (-not $match.Groups['body'].Value.Trim()) { continue }
            $newBlocks = @([regex]::Matches($newText, $userBlock) | Where-Object {
                $_.Groups['name'].Value -eq $match.Groups['name'].Value
            })
            if ($newBlocks.Count -ne 1 -or $newBlocks[0].Groups['body'].Value.Trim() -ne $match.Groups['body'].Value.Trim()) {
                throw "User code was not preserved: $relative / $($match.Groups['name'].Value)"
            }
        }
    }
}
& $Python (Join-Path $workspace 'tests/test_project_config.py') --project $project -v
if ($LASTEXITCODE -ne 0) { throw 'Generated configuration contracts failed.' }
foreach ($configuration in @('Debug','Release')) {
    & (Join-Path $stage 'scripts/build-firmware.ps1') -Configuration $configuration -Compiler $Compiler
}
$report = [ordered]@{
    passed = $true
    scope = 'isolated code-only regeneration; no upload or physical validation'
    project = $project
    cubeMX = $CubeMX
    sourceHashes = $original
    driverFilesUnchanged = $true
    nonGeneratedFilesUnchanged = $true
    nonemptyUserBlocksPreserved = $true
    target = 'STM32H743ZIT6'
    configurationContracts = 8
    builds = @('Debug','Release')
}
$report | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $stage 'report.json') -Encoding UTF8
Write-Output "CubeMX validation passed. Evidence: $stage"
