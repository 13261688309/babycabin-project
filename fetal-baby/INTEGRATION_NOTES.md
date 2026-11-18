# STM32H743 baby care integration notes

The user corrected the board to the STM32H743ZIT6 ENIG core board. This directory
is the current delivery target: LQFP144, two 1MB Flash banks, HSI 64MHz/no PLL.
Build with `scripts/build-firmware.ps1 -Target H743` (the default target).
The F407 directory remains a reference only; never flash its image to this board.
Safety/heartbeat use PF0..PF5, not the historical PC6..PC11 wiring. Keep the
onboard FPC1 camera connector empty because it shares PA4/PA5/PA6.
Current manufacturer profile: `../hardware/H743ZI沉金核心板与连线核对.md`.

This CubeIDE project has been integrated with the STM32H7 HAL demo drivers in
`Core/Inc` and `Core/Src`.

## Expansion interlock draft (2026-10-11)

The current firmware requires PF0 STOP_OK and PF1 HW_ARMED (pulldown inputs),
PF2 ACT_EN and PF3 MCU_READY (reset-low outputs). Unwired STOP/ARM blocks motion.
PF4 POWER_GOOD is a pulldown input. Power must become good within a provisional
200ms after the first ACT_EN request; repeat requests never extend the deadline.
Loss of power after a good sample latches reason 4 and requires manual recovery.
PF5 MCU_HEARTBEAT is reset LOW and toggled at least every 50ms at the end of
the complete foreground task (nominal falling-edge period 100ms). Interrupts
never feed it; late tasks do not emit catch-up edges. The external watchdog
is mandatory for this signal to remove actuator enables; no MCU reset is issued.
PWM timers may already run during power-up: a real switched-rail signal isolation
stage is mandatory. A missing PF4 wire alone does not prove zero startup motion.
`actuator_interlock.c` qualifies recovery for 50ms but trips on a negative input
without a debounce wait. RESET does not substitute for the physical ARM edge;
successful latch recovery discards normal queued commands/partial frames while
preserving urgent STOP/ALARM and in-flight TX ownership. Module/source-side
delayed packets are not authenticated or age-checked by this mechanism.

GPIO setup is synchronized with the IOC. These are draft logical MCU pins,
not verified development-board header numbers. Do not flash or bypass by tying
feedback high. The KiCad interlock subassembly under `../hardware/interlock-draft`
is unrouted and lacks the dual-rail power/disconnect and signal-isolation stage.
Software GPIO state is not proof of actual power removal or mechanical stopping.
`../hardware/power-stage-draft` V0.3 adds main-supply PG comparators, an independent
startup timer and an always-enabled external watchdog to the retained ARM/fault
logic. Historical J4 PC labels are obsolete. Current V0.6 J600 logical pins 3..8
are PF2, PF3, PF5, PF0, PF1, PF4; they are NOT core-board header numbers.
DC divider and PG level calculations include resistance/leakage corners and
remain conditional on supplies. Slow transitions, latch timing, actual timer
response, clamps, footprints and physical wiring remain unverified.
Watchdog auto-recovery does not clear the fault latch. The chip's timeout is
not a bound on the complete power stage or the mechanical stopping time.
ERC passing does not make it safe to build or energize. When qualified power and
ARM drop together, the firmware records POWER first; STOP retains top priority.

## BabyCabin_H743 module integration

The `BabyCabin` module set from `C:/stm32/BabyCabin_H743` is now included in
this project and registered in `.cproject` for Debug and Release builds.

Current safe runtime path:

- `BabyCare_Init()` initializes `BabyCabinBridge_Init()`.
- `BabyCare_Task()` calls `BabyCabinBridge_Task()`.
- The bridge feeds the imported state machine, alert manager, and JSON device
  protocol from the existing `BabyCare_StatusTypeDef`.
- BabyCabin protocol lines are emitted on USART1 with the prefixes
  `POST /api/device/report ` and `POST /api/device/alert `. These are line
  envelopes, not an implemented HTTP network connection.

Hardware-sensitive imported modules that need ADC/I2C/UART peripherals remain
compiled but disabled through `BabyCabin/inc/app_config.h`. Enable them one at
a time after adding matching CubeMX peripheral init and BSP implementations.

The imported `BabyCabin/app/app_task()` was made non-blocking for this project,
and `alert_manager` was fixed to size its per-status alert table for all
defined `baby_status_t` values.

## Demo pin map

```text
DHT11 data: PB0
DC motor PWM: TIM2_CH3 / PB10
DC motor IN1/IN2: PA4 / PA5
Servo gimbal pan: TIM3_CH1 / PA6
Servo gimbal tilt: TIM3_CH2 / PA7
Voice module UART: USART2 TX/RX = PD5 / PD6, 115200 baud
Debug UART: USART1 TX/RX = PA9 / PA10, 115200 baud
ESP32 camera WiFi UART: USART3 TX/RX = PD8 / PD9, 115200 baud
```

## Runtime behavior

- The motor timer must already be configured and the physical actuator supply
  inhibited before DC_Motor_Init. Init clears the compare and requests PWM Stop;
  only HAL_OK permits preloading both direction ODR bits LOW before GPIO output
  configuration. Failed initialization leaves initialized=0 and blocks driver
  commands. A failed Stop is not evidence that an old channel is physically off.
- Motor zero-speed patterns are DC_MOTOR_STOP_INPUTS_LOW/HIGH, not generic coast
  or brake guarantees. DC_Motor_SetStopMode returns status and rejects invalid
  handles/patterns. The old COAST/BRAKE enum names were removed; no in-tree callers
  used them. Verify the actual driver module's PWM/STBY truth table before wiring.
  BabyCare_StopOutputs revokes ACT_EN first, but physical stopping remains untested.
- `BabyCare_Init()` is called after all `MX_*_Init()` functions.
- `BabyCare_Task()` is called in the main loop.
- DHT11 requests are spaced by at least 2100 ms. The driver enforces a 1000 ms
  initialization settling period; the application waits 2100 ms before its first
  request. The 20 ms start-low interval spans main-loop calls. Service Task at
  20-29 ms; a late call at 30 ms or later aborts and releases the bus. This does
  not bound the physical low pulse if the foreground stalls. Bit reads remain
  timing-sensitive and require target oscilloscope and IRQ-latency validation.
- Temperature fraction bit 7 is the negative sign for the reviewed Aosong V1.3
  format. Invalid fields/range return DHT11_ERROR_DATA even with a valid checksum.
  The physical sensor revision and voltage/pullup compatibility are unverified.
  Successful receive time is not the sensor's exact conversion timestamp.
- USART1 prints status lines.
- USART1 reports, alerts and logs share a bounded interrupt-driven TX owner.
  Four normal slots and two reserved alert slots own copied payloads until
  completion. Alert priority applies between complete lines, not mid-frame.
  Failed starts retry with backoff; in-flight timeouts abort and insert a
  newline before another line. Failed aborts do not release active storage.
  Enqueue success is not a delivery acknowledgement.
- USART2 receives and sends five-byte voice frames in interrupt mode. The
  module owns its TX frame until completion and aborts a 200 ms timeout.
- USART2/3 RX errors or failed receive re-arms discard ordinary queued commands
  and partial input, but preserve already accepted voice STOP/ALARM and APP STOP.
  Late receive callbacks are ignored while recovery is pending. Foreground
  recovery only re-arms after HAL_UART_AbortReceive returns HAL_OK; abort/re-arm
  failures keep recovery pending without skipping TX timeout or heartbeat work.
  Receive setup and recovery-state publication share a short IRQ-masked section
  and restore the caller's PRIMASK. An ESP32 configuration awaiting ACK fails on
  a receive re-arm error rather than joining a pre-error reply to later bytes.
  This does not age/authenticate module-side packets, revoke already dispatched
  commands, or prove that UART failure physically stops ongoing motion.
- USART3 sends configuration and AP/STA IP queries in interrupt mode, receives
  the responses, and prints status through USART1. Configuration steps advance
  only after both TX completion and a standalone newline-terminated `OK` reply;
  busy sends never overwrite active data. A 3-second ACK deadline, `fail` reply,
  or RX error ends configuration without replaying a possibly applied command.
  Queries cannot overlap configuration. Only failed TX starts retry (at most 3).
  Config result 2 means all six commands were acknowledged, not read back or
  proven persistent. Exact reply framing still requires actual firmware capture.
- USART3 accepts APP transparent-control frames from the ESP32 module. Servo
  frames follow the supplied Yahboom APP protocol: `$A090#` is tilt, `$B090#`
  is pan, and the three digits are a 0-180 degree target before soft limits.
  Consecutive commands preserve both axis targets. The factory APP stop frame
  `$7,0,0,0#` is normalized to STOP and has priority even when the queue is full.
  Other vehicle movement frames are not mapped to cradle motion.
- Voice commands switch the demo mode between idle, sleep, soothe, and alert.
- Boot leaves motor and servo PWM disabled. Alert and hardware faults latch
  starts off. `$STOP#` stops outputs; `$RESET#` clears an alert only if hardware
  fault flags are clear, and does not start outputs.
- Soothe requests expire after 60 seconds; repeated requests do not extend the
  deadline. Remote servo holding expires after 30 seconds.
- Motor reversal ramps to zero and waits before changing direction. The
  provisional ramp and pause settings still require load testing.
- Servo calibration endpoints remain 0-180 degrees for pulse mapping, while
  provisional mechanical soft limits are pan 60-120 and tilt 75-105 degrees.
  Targets advance one degree per 20 ms step. These ranges are not a substitute
  for physical travel calibration.
- ESP32 configuration is scheduled without startup delays. IP reports expire
  after 15 seconds; IP availability is not a baby detection.
- Unknown/disabled observations are JSON null. DHT11 measures environment
  temperature, not body temperature. No sensor is fabricated from a mode.
- The dormant BabyCabin command decoder now accepts only complete JSON command
  objects or `commands` arrays with an exact `command` field. Durations must be
  positive integers, cannot exceed 300 seconds, and duplicate/conflicting fields
  reject the whole packet. The production bridge still has no command transport
  or acknowledgement path, so this is parser hardening rather than remote-control
  enablement.

The supplied CI1302 asset package documents 5V power and 115200 UART; its
Python driver uses the same `AA 55 type id FB` frame. Verify the actual board,
signal voltage, and flashed command table before hardware connection.

## Host vision companion

The root `companion` package loads the existing four-class local YOLO model.
It outputs metadata-only JSONL with model hash, timestamps, raw detections,
and temporally confirmed observations. The optional USART1 reader is read-only;
vision does not start actuators or feed fabricated sensor observations into the
firmware state machine. The optional dashboard binds only to `127.0.0.1`, has
no actuator controls, clears stale or terminal observations to Unknown, and hides
stale/offline telemetry. Failed or timed-out browser requests clear old values;
successful subsequent requests recover the display. See
the root README for commands and limitations.

For APP servo remote control, `ESP32_CAM_AI_MODE` is set to `0` to match the
Yahboom example's normal transparent mode. The tilt axis is reversed in
`ServoGimbal_ParseRemoteFrame()` to match the example behavior.

## CubeMX note

The IOC now records the active peripherals. Generated peripheral ownership is
in `tim.c`, `usart.c`, and `gpio.c`; application calls and UART callbacks remain
inside `main.c` user sections. Do not add duplicate MSP functions inside a user
section. Preserve the following configuration:

- Enable TIM2 channel 3 as PWM on PB10.
- Enable TIM3 channels 1 and 2 as PWM on PA6 and PA7.
- Enable USART1 asynchronous mode on PA9 and PA10 at 115200, with USART1 global
  interrupt enabled at priority 6 (below USART2/3 receive priority 5).
- Enable USART2 asynchronous mode on PD5 and PD6 at 115200, with USART2 global
  interrupt enabled.
- Enable USART3 asynchronous mode on PD8 and PD9 at 115200, with USART3 global
  interrupt enabled.
- Keep PB0 as GPIO input with pull-up for DHT11.

Also preserve PA4/PA5 reset-low outputs, HSI 64 MHz with no PLL, TIM2 PSC=63 and
ARR=99, and TIM3 PSC=63 and ARR=19999 (both servo CCR=1500 initially; PWM is not
started by initialization). GPIO alternate functions keep low output speed.

Run the root `scripts/test-cubemx-regeneration.ps1` for isolated code-only
generation, user-block and library hash checks, seven static configuration
contracts, and Debug/Release builds. It never promotes generated files or uploads
firmware. Tested with the locally installed CubeMX 6.17.0-RC5; full GUI project
generation and library/toolchain migration are not validated. The IOC retains
the original FW_H7 V1.12.1 metadata and the vendored HAL was not upgraded.
