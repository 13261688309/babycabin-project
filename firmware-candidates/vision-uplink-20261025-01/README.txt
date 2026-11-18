H743 OBSERVATION UPLINK; COMPLETE APP RETAINED; NOT FLASHED

Use handoff/next-firmware.json for the complete built image and matched PC.
Existing motor, gimbal, STOP priority, voice, DHT11 and interlocks are retained.
This new USB OUT path accepts observations only, never actuator commands.
The PC default remains read-only. After an approved backup/update/verification,
the operator may explicitly select Start-Demo.cmd --vision-uplink.

Wire: exactly 64 bytes, little endian (<4sIIIIHBB32sII in Python struct).
Offsets: 0 magic BV01; 4 nonzero host session; 8 sequence; 12 MCU expiry tick;
16 source age ms; 20 confidence per mille; 22 class; 23 confirmed flag;
24 full model SHA256 bytes; 56 source=1 (local PC network-video observer);
60 standard CRC32 of bytes 0..59. Class IDs: Unknown=0 Cry=1 Happy=2 Neutral=3 Sleep=4.
Unknown requires confidence/flag=0. Other classes require flag=1 and score>=600.

Every 500ms the MCU issues an uptime anchor in vision_link. The PC sets expiry
to that anchor + 1500ms - current source age. The MCU checks the exact issued
anchor, remaining budget, model, source, CRC, session and forward-only sequence.
Thus delayed USB/foreground data cannot acquire a new 1500ms lifetime on arrival.
age_ms in returned vision_link is consumed validity budget, not camera exposure
age; network source timestamps still mean multipart start/decode completion.
CRC, model hashes and USB metadata are not cryptographic authentication.

USB callbacks only copy <=64-byte chunks into a bounded queue. Foreground parsing
handles fragmentation, noise and partial timeouts. Overflow invalidates state.
USB init/deinit/control-session changes invalidate immediately. Expiry, Unknown,
bad frames and discontinuities reset continuous visual Cry tracking.
Three seconds of continuous accepted fresh visual Cry generates one warning
per episode: visual_cry / check_live_view. This is not an audio cry diagnosis,
medical judgment, automatic soothing or proof of physical power removal.

The firmware archive includes full source, Debug/Release ELF/HEX/MAP, hashed host
inputs and logs. Ten host campaigns include a Python TemporalState/encoder ->
actual C receiver/app -> PC decoder/dashboard replay. Model inference and HAL
are simulated in that replay. Existing-model CPU inference was checked separately
on the labelled synthetic image, not as an accuracy or live-camera qualification.

USB log bursts are buffered independently of USART1 in this image. Four reserved
telemetry slots coalesce pending status/report/voice/vision topics; two general
and two alert slots are bounded. Alerts take priority, with one normal frame
after at most two alerts. The active USB/ZLP buffer is never overwritten.
DTR/session changes purge pending logs. UART errors do not block the USB queue.
The actual ST core/CDC host harness reproduced the old 1-of-5-topic loss, then
verified all five topics, bounded overload, coalescing, fairness and real app
framing. USB LL and HAL remain simulated; this does not replace a real capture.

Rebuild, no device access:
python tools/build_vision_link_firmware.py --output <new workspace directory>

Before real motion, finish the actual power, STOP/ARM/PG/watchdog and actuator
checks. Do not bridge feedback inputs or bypass interlocks. No new flash,
procurement, fabrication or actuator power operation is authorized by these files.
