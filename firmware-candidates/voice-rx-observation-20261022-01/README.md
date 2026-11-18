# Voice RX observation candidate

Date: 2026-10-22. This candidate addresses the next physical UART test's lack of
fresh-reception evidence. It does not repeat camera or standalone voice tests.
The selected production source, manifest, firmware and flashed H743 remain unchanged.

## Changes

- Count each complete AA/55/type/id/FB parser frame before queue delivery decisions.
- Count command-type STOP_1 and STOP_2 separately from ALARM and system frames.
- Preserve counts across queue flush and receive recovery. Voice_Init resets them.
- Snapshot counters, timestamps, UART-error callback count and queue-drop count
  within one short critical section, restoring the caller's interrupt state.
- Emit a separate periodic `voice_rx` log line through the existing debug/USB
  observation path, without changing the existing status format or motion logic.

Example:

```text
voice_rx frames=2 stops=2 alarms=0 last_ms=20 stop_ms=20 errors=0 dropped=0
```

These are unsigned 32-bit counters and local HAL tick values. They wrap modulo
2^32. A zero timestamp can be a real tick or an initial value; interpret it with
the counters and a known boot session. Compare only observations from the same
boot, and do not infer counts across a reset or unknown sampling gap.

`frames` means structurally complete parser frames, not authenticated or semantically
accepted commands. Counts include frames coalesced or dropped by the queue.
`stops` is received STOP-frame count, not application STOP executions or physical
stops. `errors` counts matching UART error callbacks, which may include TX errors;
it is not a pure RX electrical-error counter. Timestamps mark parser completion,
not speech onset, application handling time or motor power removal. No physical
stop latency claim can be made from these fields.

## Verification

Latest result: `build/verification/voice-rx-observation-20261022-03/verification.json`.
Six host campaigns cover parser/resynchronization, application STOP behavior and
new diagnostics in both default and explicit camera-configuration branches.
The new campaign also checks actual periodic log contents: two STOP inputs produce
count 2 on two successive reports while modeled actuator outputs remain disabled.
Four Cortex-M7 object compilations cover both changed C files with -O0 and -Os.

Run `verify.py NEW_OUTPUT_DIRECTORY ARM_GCC_EXE` with the project's bench Python.
The verifier refuses an existing output directory. The initial test-harness compile
failure (missing arguments to the reused regression main) is retained in the -01
output; the harness was corrected before passing campaigns.

Full H743 Debug and Release linking also passed in a separate 200-input source
snapshot with exactly these three candidate overlays. ELF/HEX files and their
hashes are recorded in
`build/verification/voice-rx-full-build-20261022-01/verification.json`.
Production sources and selected firmware remain unchanged.

Not yet done: production promotion, real USB log observation,
physical voice UART reception, electrical qualification and physical STOP testing.
Do not flash these source files or substitute them into a frozen package manually.
Integrate and link them once the next firmware update is being prepared; do not
request a separate flash merely to repeat already-passed standalone tests.
