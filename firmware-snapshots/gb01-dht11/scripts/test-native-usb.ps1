param([string]$Compiler, [Parameter(Mandatory=$true)][string]$OutputDirectory,
    [switch]$CameraConfigureOnBoot)
$ErrorActionPreference = 'Stop'
$workspace = Split-Path $PSScriptRoot -Parent
if (-not $Compiler) { $Compiler = Join-Path $workspace '.tools/tcc/tcc.exe' }
if (-not (Test-Path -LiteralPath $Compiler)) { throw 'Host compiler required; no installation or device access.' }
$output = [IO.Path]::GetFullPath($OutputDirectory)
if (Test-Path -LiteralPath $output) { throw 'USB test output must be a new directory.' }
New-Item -ItemType Directory -Path $output | Out-Null
$project = Join-Path $workspace 'fetal-baby'
$usb = Join-Path $project 'Middlewares/ST/STM32_USB_Device_Library'
$modules = @('usb_observer_tx','usbd_cdc_if','baby_care_app','baby_cabin_bridge','voice_interaction',
    'esp32_camera_link','dc_motor_pwm','servo_gimbal','debug_uart_tx','actuator_interlock','system_heartbeat')
$sources = @($modules | ForEach-Object { Join-Path $project "Core/Src/$_.c" })
$sources += @('services/state_machine.c','services/power_manager.c','services/alert_manager.c',
    'protocol/device_protocol.c','drivers/comm_bridge.c') | ForEach-Object { Join-Path $project "BabyCabin/$_" }
$includes = @('tests','tests/hal','fetal-baby/Core/Inc','fetal-baby/BabyCabin') |
    ForEach-Object { '-I' + (Join-Path $workspace $_) }
$defines = @('-DBABY_USB_OBSERVER')
if ($CameraConfigureOnBoot) { $defines += '-DBABY_CARE_ESP32_CONFIGURE_ON_BOOT=1' }
$executable = Join-Path $output 'usb-observer-tests.exe'
$mock = '-I' + (Join-Path $workspace 'tests/usb/mocks')
& $Compiler -Wall -Werror @defines $mock @includes (Join-Path $workspace 'tests/usb/test_usb_observer.c') @sources -o $executable
if ($LASTEXITCODE -ne 0) { throw 'USB observer host compilation failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'USB observer host regression failed' }
$stackIncludes = @('Core/Inc','Class/CDC/Inc') | ForEach-Object { '-I' + (Join-Path $usb $_) }
$stackSources = @('Core/Src/usbd_core.c','Core/Src/usbd_ctlreq.c','Core/Src/usbd_ioreq.c','Class/CDC/Src/usbd_cdc.c') |
    ForEach-Object { Join-Path $usb $_ }
$executable = Join-Path $output 'actual-usb-stack-tests.exe'
& $Compiler -Wall -Werror @defines -include (Join-Path $workspace 'tests/usb/host_port.h') @includes @stackIncludes `
    (Join-Path $workspace 'tests/usb/test_usb_stack.c') @sources @stackSources -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Actual ST USB stack host compilation failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Actual ST USB stack host regression failed' }
Write-Output 'USB host regression passed; MCU HAL/USB LL simulated, no hardware access.'
