# Voice to H743 receive-only capture

This workspace tool covers the next voice-to-MCU test, after the external
interface and wiring have been independently checked. It reads the H743's own
USB CDC logs, not the voice board USB port or the unknown USB-TTL adapter.
The operator is currently unavailable for measurements; do not run a live
capture until fresh availability and wiring confirmations have been obtained.

## Preconditions

1. Verify replacement meter-lead contact and repeat the relevant supply and
   voice TX1 measurements. Prior readings are provisional. An idle-voltage
   reading alone does not qualify logic thresholds, power sequencing or
   backfeeding when either board is off.
2. Independently qualify the receive-only interface and any level-conversion
   hardware before connecting voice TX1 to H743 PD6 (USART2 RX). Keep PD5 and
   voice RX1 disconnected. This tool does not authorize direct wiring or make
   an unbuilt adapter candidate acceptable for use.
3. Use each board's own USB supply only. Do not bridge the boards' 5V/3V3 rails
   or add another supply. Follow the qualified interface's grounding and
   power-sequencing requirements. Do not rearrange wires while powered.
4. Disconnect motor/servo power and signals, camera, and USB-TTL from the MCU.
   Do not operate ARM or send motion commands. If ST-Link remains connected,
   it must not provide a second power source; this capture tool does not use it.
5. Physically identify the H743 native USB and obtain its current port, USB
   serial number and USB location. Do not reuse COM24 or another historical
   port without checking. Close competing readers before this capture.

The four CLI confirmations attest to this run's operator checks, not to
software-verifiable electrical qualification. Do not use
`capture_mcu_usb.py --isolated-usb-confirmed` after attaching the voice UART;
that existing tool remains restricted to an isolated MCU.

## Run when ready

From the workspace, inspect options without accessing hardware:

```powershell
.\.tools\bench-python\Scripts\python.exe tools\capture_voice_to_mcu.py --help
```

Run `tools/capture_voice_to_mcu.py` with all of the following values, only after
the preconditions have actually been met:

| Option | Value |
| --- | --- |
| `--port` | Current native H743 COM port |
| `--expected-serial` | Current native USB serial number |
| `--expected-location` | Current native USB location |
| `--output` | New, unused capture directory |
| `--seconds` | Capture window, normally 60; maximum 300 |
| `--native-board-confirmed` | Fresh physical device confirmation |
| `--voice-rx-harness-confirmed` | Qualified, receive-only voice interface |
| `--actuators-disconnected-confirmed` | Isolation described above |
| `--separate-usb-power-confirmed` | No power-rail bridge or second board supply |

Wait for `Native USB opened; receive-only voice test capture started.` before
prompting the operator to wake the voice module and say the STOP command.
Do not send host commands or reset the MCU during capture. The capture has a
1 MiB default byte limit and does not reconnect automatically after a failure.

A useful run contains `voice=0x00` before the command and `voice=0x01` or
`voice=0x02` afterward. The voice field persists: a run that starts with STOP
already present cannot demonstrate a new reception. Investigate the saved
baseline before arranging any separately authorized reset or repeat test.

## Results

- `telemetry/received.bin`: exact received bytes, including status lines.
- `telemetry/decoded.jsonl` and `summary.json`: decoded reports and capture limits.
- `voice-review.json`: offline report-sequence and voice-state checks.
- `capture.json`: current endpoint expectations, operator declarations,
  source/capture hashes and outcome. A failed review retains the raw capture.

Exit 0 means normal reception and the expected saved-log state transition;
exit 1 means reception occurred but the voice-state check needs attention;
exit 2 means refusal, interrupted/failed capture or failed saved-file review.
Silence, existing STOP state, enabled interlock flags, tick resets, interrupted
captures and malformed/inconsistent records cannot produce a positive result.

Even exit 0 does not prove the flashed firmware identity, UART electrical
qualification, fresh raw voice frames, command latency or physical stopping.
OS USB class requests, including DTR, may occur; no application bytes are sent.

This is an additive workspace tool. The frozen servo-clock firmware/PC ZIP,
PCB and historical bench evidence remain unchanged; the new tool is not part
of that frozen ZIP. The tests use simulated ports only, not a hardware result.
