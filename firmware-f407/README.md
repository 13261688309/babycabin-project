# F407 Firmware Target

Target: STM32F407ZGT6 (LXB core board, physical revision still unverified).
Reference target only. The user corrected the actual board to STM32H743ZIT6 ENIG;
the current delivery target is `../fetal-baby` with the default `-Target H743`.
Never flash this F407 image to the existing H743 board.

Generated peripheral code is local. Application sources and BabyCabin services are
compiled directly from `../fetal-baby`, not copied into this target. The official
HAL/CMSIS/startup sources come from the locally installed STM32Cube FW_F4 V1.28.3;
their license files are retained in `Drivers` and `Package_license.md`.

Use the workspace build and host-test scripts with `-Target F407`. Outputs are in
`../build/firmware/F407/Debug` and `Release`. The build does not upload firmware.
The default script target is H743 for the actual project hardware. Pass the F407
argument only when deliberately testing this reference, not for project delivery.

Clock: nominal HSI-derived 84 MHz; APB1 42 MHz, TIM2/3 84 MHz, APB2 84 MHz.
TIM2 PSC=83/ARR=99, TIM3 PSC=83/ARR=19999. Safety/heartbeat pins are PF0..PF5,
not the TF-card-connected PC8..PC11. The full wiring review and version limitations
are in `../hardware/F407迁移与连线核对.md`.

The I2C HAL is compiled to preserve the shared voice API, but no I2C peripheral
is initialized or connected. Voice communication remains USART2.

CubeMX code-only generation: `scripts/generate-f407.ps1` from the workspace.
Isolated regeneration, shared-source/vendor hash checks, configuration tests and
Debug/Release builds: `scripts/test-f407-regeneration.ps1`.
No complete CubeIDE GUI project, board electrical acceptance, PCB fabrication
release or on-device timing validation is claimed.
