param([string]$Compiler, [ValidateSet('H743', 'F407')][string]$Target = 'H743',
    [string]$OutputDirectory, [switch]$CameraConfigureOnBoot)
$ErrorActionPreference = 'Stop'
$workspace = Split-Path $PSScriptRoot -Parent
if (-not $Compiler) { $Compiler = Join-Path $workspace '.tools/tcc/tcc.exe' }
if (-not (Test-Path -LiteralPath $Compiler)) { throw 'Host C compiler not found. Supply -Compiler (TinyCC or GCC).' }
$output = if ($OutputDirectory) {
    $resolved = [IO.Path]::GetFullPath($OutputDirectory)
    if (Test-Path -LiteralPath $resolved) { throw 'Explicit host-test output must be a new directory.' }
    $resolved
} else { Join-Path $workspace "build/tests/$Target" }
New-Item -ItemType Directory -Force -Path $output | Out-Null
$sources = @('tests/test_firmware.c','fetal-baby/Core/Src/baby_care_app.c',
    'fetal-baby/Core/Src/baby_cabin_bridge.c','fetal-baby/Core/Src/voice_interaction.c',
    'fetal-baby/Core/Src/esp32_camera_link.c','fetal-baby/Core/Src/dc_motor_pwm.c',
    'fetal-baby/Core/Src/servo_gimbal.c','fetal-baby/BabyCabin/services/state_machine.c',
    'fetal-baby/BabyCabin/services/power_manager.c','fetal-baby/BabyCabin/services/alert_manager.c',
    'fetal-baby/BabyCabin/protocol/device_protocol.c',
    'fetal-baby/BabyCabin/drivers/comm_bridge.c',
    'fetal-baby/Core/Src/debug_uart_tx.c',
    'fetal-baby/Core/Src/actuator_interlock.c',
    'fetal-baby/Core/Src/system_heartbeat.c') | ForEach-Object { Join-Path $workspace $_ }
$includes = @('tests/hal','fetal-baby/Core/Inc','fetal-baby/BabyCabin') |
    ForEach-Object { '-I' + (Join-Path $workspace $_) }
$executable = Join-Path $output 'firmware-tests.exe'
[string[]]$defines = if ($Target -eq 'F407') { @('-DSTM32F407xx') } else { @() }
if ($CameraConfigureOnBoot) { $defines += '-DBABY_CARE_ESP32_CONFIGURE_ON_BOOT=1' }
& $Compiler -Wall @defines @includes @sources -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Host test build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Firmware regression failed' }
$clockExecutable = Join-Path $output 'esp-task-clock-tests.exe'
$driverSources = @($sources | Select-Object -Skip 1)
$clockHarness = Join-Path $workspace 'tests/test_esp_task_clock.c'
$clockHook = Join-Path $workspace 'tests/esp_task_clock_hook.h'
& $Compiler -Wall @defines @includes -include $clockHook $clockHarness @driverSources -o $clockExecutable
if ($LASTEXITCODE -ne 0) { throw 'ESP clock regression build failed' }
& $clockExecutable
if ($LASTEXITCODE -ne 0) { throw 'ESP fresh-IRQ clock regression failed' }
$gapExecutable = Join-Path $output 'esp-remote-gap-tests.exe'
$gapHarness = Join-Path $workspace 'tests/test_esp_remote_gap.c'
& $Compiler -Wall @defines @includes $gapHarness @driverSources -o $gapExecutable
if ($LASTEXITCODE -ne 0) { throw 'ESP gap regression build failed' }
& $gapExecutable
if ($LASTEXITCODE -ne 0) { throw 'ESP gap regression failed' }
$dhtExecutable = Join-Path $output 'dht-driver-tests.exe'
$dhtIncludes = @('tests/dht_hal','fetal-baby/Core/Inc') |
    ForEach-Object { '-I' + (Join-Path $workspace $_) }
$dhtSources = @('tests/test_dht_driver.c','fetal-baby/Core/Src/dht11.c') |
    ForEach-Object { Join-Path $workspace $_ }
& $Compiler -Wall -Werror @defines @dhtIncludes @dhtSources -o $dhtExecutable
if ($LASTEXITCODE -ne 0) { throw 'DHT11 host test build failed' }
& $dhtExecutable
if ($LASTEXITCODE -ne 0) { throw 'DHT11 driver regression failed' }
if ($Target -eq 'H743') {
    & (Join-Path $PSScriptRoot 'test-native-usb.ps1') -Compiler $Compiler `
        -OutputDirectory (Join-Path $output 'native-usb') -CameraConfigureOnBoot:$CameraConfigureOnBoot
}
