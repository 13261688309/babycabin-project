param([string]$Compiler, [ValidateSet('Debug', 'Release')][string]$Configuration = 'Debug',
    [ValidateSet('H743', 'F407')][string]$Target = 'H743', [string]$OutputDirectory,
    [switch]$VisionUplink)
$ErrorActionPreference = 'Stop'
if ($VisionUplink -and $Target -ne 'H743') { throw 'Vision uplink is H743 USB only.' }
$workspace = Split-Path $PSScriptRoot -Parent
$shared = Join-Path $workspace 'fetal-baby'
$project = if ($Target -eq 'F407') { Join-Path $workspace 'firmware-f407' } else { $shared }
if (-not $Compiler) {
    $installed = Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue
    if ($installed) { $Compiler = $installed.Source }
    else {
        $Compiler = Get-ChildItem 'C:/ST' -Filter arm-none-eabi-gcc.exe -Recurse -ErrorAction SilentlyContinue |
            Select-Object -First 1 -ExpandProperty FullName
    }
}
if (-not $Compiler -or -not (Test-Path -LiteralPath $Compiler)) { throw 'ARM GCC not found. Supply -Compiler.' }
$output = if ($OutputDirectory) {
    $resolved = [IO.Path]::GetFullPath($OutputDirectory)
    if (Test-Path -LiteralPath $resolved) { throw 'Explicit build output must be a new directory.' }
    $resolved
} elseif ($Target -eq 'F407') { Join-Path $workspace "build/firmware/F407/$Configuration" }
    else { Join-Path $workspace "build/firmware/$Configuration" }
New-Item -ItemType Directory -Force -Path $output | Out-Null
$family = if ($Target -eq 'F407') { 'F4' } else { 'H7' }
$includes = @('Core/Inc',"Drivers/STM32${family}xx_HAL_Driver/Inc","Drivers/STM32${family}xx_HAL_Driver/Inc/Legacy",
    "Drivers/CMSIS/Device/ST/STM32${family}xx/Include",'Drivers/CMSIS/Include') |
    ForEach-Object { '-I' + (Join-Path $project $_) }
$includes += @(('-I' + (Join-Path $shared 'Core/Inc')), ('-I' + (Join-Path $shared 'BabyCabin')))
if ($Target -eq 'H743') {
    $includes += @('Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
        'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc') |
        ForEach-Object { '-I' + (Join-Path $project $_) }
}
$common = if ($Target -eq 'F407') {
    @('-mcpu=cortex-m4','-mfpu=fpv4-sp-d16','-DSTM32F407xx','-DHAL_I2C_MODULE_ENABLED')
} else {
    @('-mcpu=cortex-m7','-mfpu=fpv5-d16','-DUSE_PWR_LDO_SUPPLY','-DSTM32H743xx','-DBABY_USB_OBSERVER')
}
$common += @('-mthumb','-mfloat-abi=hard','-DUSE_HAL_DRIVER','-ffunction-sections','-fdata-sections')
if ($VisionUplink) { $common += '-DBABY_USB_VISION_UPLINK' }
[string[]]$optimization = if ($Configuration -eq 'Debug') { @('-Og','-g3','-DDEBUG') } else { @('-Os') }
$sourceRoots = @('Core/Src',"Drivers/STM32${family}xx_HAL_Driver/Src")
$sources = foreach ($root in $sourceRoots) {
    Get-ChildItem -LiteralPath (Join-Path $project $root) -Filter '*.c' -File -Recurse
}
$sources += Get-ChildItem -LiteralPath (Join-Path $shared 'BabyCabin') -Filter '*.c' -File -Recurse
if ($Target -eq 'H743') {
    $templates = @('usbd_conf_template.c','usbd_desc_template.c','usbd_cdc_if_template.c')
    $sources += Get-ChildItem -LiteralPath (Join-Path $project 'Middlewares/ST/STM32_USB_Device_Library') -Filter '*.c' -File -Recurse |
        Where-Object { $templates -notcontains $_.Name }
}
if ($Target -eq 'F407') {
    $sharedModules = @('actuator_interlock','baby_cabin_bridge','baby_cabin_bsp_hal','baby_care_app',
        'dc_motor_pwm','debug_uart_tx','dht11','esp32_camera_link','servo_gimbal','system_heartbeat','voice_interaction')
    $sources += foreach ($name in $sharedModules) { Get-Item -LiteralPath (Join-Path $shared "Core/Src/$name.c") }
}
# Do not let host culture or .NET collation change object/link order.
$orderedSources = [Collections.Generic.SortedDictionary[string, IO.FileInfo]]::new([StringComparer]::Ordinal)
foreach ($source in $sources) { $orderedSources[$source.FullName] = $source }
$sources = @($orderedSources.Values)
$objects = @()
foreach ($source in $sources) {
    $relative = $source.FullName.Substring($workspace.Length + 1)
    $object = Join-Path $output ([IO.Path]::ChangeExtension($relative, '.o'))
    New-Item -ItemType Directory -Force -Path (Split-Path $object -Parent) | Out-Null
    [string[]]$vendorWarnings = @()
    if ($Target -eq 'F407' -and $relative -like 'firmware-f407\Drivers\STM32F4xx_HAL_Driver\Src\*') {
        # Official single-bank F4 HAL leaves the Banks parameter unused.
        $vendorWarnings = @('-Wno-unused-parameter')
    }
    & $Compiler @common @optimization @includes -std=gnu11 -Wall -Wextra -Werror @vendorWarnings -fstack-usage -c $source.FullName -o $object
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $relative" }
    $objects += $object
}
$startup = if ($Target -eq 'F407') { Join-Path $project 'Core/Startup/startup_stm32f407xx.s' }
    else { Join-Path $project 'Core/Startup/startup_stm32h743zitx.s' }
$startupObject = Join-Path $output 'startup.o'
& $Compiler @common -c $startup -o $startupObject
if ($LASTEXITCODE -ne 0) { throw 'Startup compilation failed' }
$objects += $startupObject
$elf = Join-Path $output 'fetal-baby.elf'
$map = Join-Path $output 'fetal-baby.map'
$linker = if ($Target -eq 'F407') { 'STM32F407ZGTX_FLASH.ld' } else { 'STM32H743ZITX_FLASH.ld' }
$linkArguments = @(('-T' + (Join-Path $project $linker)),
    '--specs=nosys.specs','--specs=nano.specs','-Wl,--gc-sections','-Wl,-u,_printf_float',
    "-Wl,-Map=$map",'-Wl,--start-group','-lc','-lm','-Wl,--end-group','-o',$elf)
& $Compiler @common @objects @linkArguments
if ($LASTEXITCODE -ne 0) { throw 'Firmware link failed' }
$compilerDirectory = Split-Path $Compiler -Parent
& (Join-Path $compilerDirectory 'arm-none-eabi-size.exe') $elf
if ($LASTEXITCODE -ne 0) { throw 'Firmware size check failed' }
& (Join-Path $compilerDirectory 'arm-none-eabi-objcopy.exe') -O ihex $elf (Join-Path $output 'fetal-baby.hex')
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
Write-Output "Firmware build passed ($Target; no upload): $elf"
