NEXT INTEGRATED H743 IMAGE; NOT FLASHED

Use handoff/next-firmware.json for the built image and matched PC package.
The package contains a complete source snapshot and Debug/Release ELF/HEX/MAP.
The frozen historical selected package and production source are unchanged.

This candidate starts with the latest selected baby_care_app source, retaining
ServoGimbal_Task(..., HAL_GetTick()) and all motor/interlock behavior. It reuses
the already reviewed voice RX counter driver, adding a separate foreground
voice STOP handling counter. BabyCare_Stop called by another source does not
increment the voice handling counter. Queue coalescing may make RX and handled
counts differ. Neither count proves a physical power cut.

Wire format: voice_rx v=2 frames=... stops=... alarms=... last_ms=... stop_ms=...
errors=... dropped=... applied=... apply_ms=... now_ms=...
All fields are uint32 decimal; counters/uptime may wrap. Stats are periodic,
not a per-command acknowledgement. UART errors include the shared UART callback.

The PC rebaselines at first observation, missing/stale diagnostics, disconnect,
reconnect-generation change, long uptime gaps and detected regressions. It only
reports counter deltas within continuous fresh intervals; no authenticated boot
identity or physical-stop claim is made. Old 7-field diagnostics and old firmware
without diagnostics remain readable, but do not prove a fresh repeated STOP.

Rebuild without devices:
python tools/build_integrated_firmware.py --output <new workspace directory>

Do not flash until the operator approves the new image and the backup/isolation
procedure. No motion, manufacturing, purchase or protection bypass is authorized.
