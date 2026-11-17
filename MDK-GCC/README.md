# STM32 for VSCode build

The workspace uses `bmd.stm32-for-vscode` and `STM32-for-VSCode.config.yaml`.
Run **STM32: Build STM32 project** or the **Build STM** task. The build produces
`build/debug/standard_robot.elf`, `.hex`, and `.bin`.

The source list follows `MDK-ARM/standard_robot.uvprojx`. GCC-specific startup,
FreeRTOS port, CMSIS compiler header, and fast sine/cosine sources are in this
directory. The original Keil project and its libraries remain available.
The linker limits code to flash sectors 0 through 8; calibration sector 9 at
`0x080A0000` is excluded from the code region.

GCC support sources:

- ST startup: https://github.com/STMicroelectronics/cmsis-device-f4/blob/master/Source/Templates/gcc/startup_stm32f407xx.s
- FreeRTOS V10.3.1: https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V10.3.1-kernel-only/portable/GCC/ARM_CM4F
- CMSIS GCC header: https://github.com/ARM-software/CMSIS_5/blob/5.4.0/CMSIS/Core/Include/cmsis_gcc.h
- CMSIS DSP: https://github.com/ARM-software/CMSIS_4/tree/master/CMSIS/DSP_Lib/Source

The DSP table file contains only `sinTable_f32`, used by the two included
fast-math functions. Original copyright and licence notices are retained.
`include/` also supplies case-sensitive aliases for legacy Keil include names.

# Pitch configuration

Pitch uses CAN2, GM6020 ID 2 (feedback `0x206`, voltage command `0x1FF` bytes
2 and 3, zero based). Right switch middle selects manual absolute-angle control.
Parameters are in `application/gimbal_task.h`:

- `PITCH_RC_SEN`: negative polarity matches the operator's physical up/down check.
- `PITCH_GYRO_ABSOLUTE_PID_*`: manual angle controller.
- `PITCH_SPEED_PID_*`: manual rate controller and voltage output limit.
- `PITCH_SOFT_MIN_ECD`, `PITCH_SOFT_MAX_ECD`: replacement motor encoder stops.

Raising the head decreases the encoder value on this installation, so
`PITCH_SOFT_MIN_ECD` is the physical upper stop and `PITCH_SOFT_MAX_ECD` is the
physical lower stop. The operator selected the highest allowed pose at encoder
6377 through 6387 (median 6379). The upper stop is 6400, leaving about one degree
inside that pose. The lower stop remains 8070. This expands the previous upper
stop of 7200 by about 35 degrees, matching the selected pose.

These are operator-selected soft stops, **not a full mechanical calibration**.
`PITCH_USE_FIXED_ENCODER_LIMITS=0` selects the stored calibration instead.

On startup, gimbal control waits for pitch feedback and a working IMU. Every
entry into manual mode returns pitch to `PITCH_MID_HOME_ECD=7211`, the encoder
position selected by the operator (33 stable samples). The return setpoint ramps
at `PITCH_MID_RETURN_STEP`; it is resolved from encoder and current IMU feedback,
so it does not depend on the power-on pose. Home-only integral compensation
handles the observed steady holding load. Once settled, that holding effort is
retained and home integration stops. Stick or mouse input overrides the return
immediately. Missing pitch feedback zeros its output after 50 ms.

Host regression check:

```sh
cc -std=c99 -Wall -Wextra -Werror -Iapplication tests/test_gimbal_pitch_control.c -lm -o /tmp/test_pitch
/tmp/test_pitch
```

Building does not flash the board. The separately configured flash task resets
the controller and should only be run when the hardware is ready for testing.
