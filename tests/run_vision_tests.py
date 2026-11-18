#!/usr/bin/env python3
"""Host checks; links production control code with simulated hardware only."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def run(args):
    result = subprocess.run(args, cwd=ROOT, text=True, capture_output=True)
    if result.returncode:
        print(result.stdout, end="")
        print(result.stderr, end="")
        raise SystemExit(result.returncode)
    print(result.stdout, end="")


with tempfile.TemporaryDirectory(prefix="hero-vision-tests-") as directory:
    protocol = str(Path(directory) / "protocol")
    run(["cc", "-std=c99", "-O1", "-Wall", "-Wextra", "-Werror",
         "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-Iapplication",
         "tests/test_vision_protocol.c", "application/vision_protocol.c", "-lm", "-o", protocol])
    run([protocol])

    # Reuse the firmware's include paths; hardware headers are present, but no
    # hardware function runs. Their ARM-only unused code is garbage-collected.
    config = (ROOT / "STM32-for-VSCode.config.yaml").read_text()
    section = config.split("includeDirectories:\n", 1)[1].split("sourceFiles:", 1)[0]
    includes = []
    for line in section.splitlines():
        if line.startswith("- "):
            includes.extend(["-I", line[2:].strip()])
    decisions = str(Path(directory) / "decisions")
    run(["cc", "-std=c99", "-O1", "-fcommon", "-ffunction-sections", "-fdata-sections",
         "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast", "-Wno-overflow",
         "-Wno-implicit-function-declaration", "-DSTM32F407xx", "-DUSE_HAL_DRIVER",
         "-DARM_MATH_CM4", "-D__FPU_PRESENT=1U", *includes,
         "tests/test_vision_decisions.c", "application/vision_protocol.c",
         "application/gimbal_behaviour.c", "application/shoot.c",
         "components/controller/pid.c", "components/algorithm/user_lib.c",
         "-Wl,--gc-sections", "-lm", "-o", decisions])
    run([decisions])

    transports = str(Path(directory) / "transports")
    run(["cc", "-std=c99", "-O1", "-fcommon", "-ffunction-sections", "-fdata-sections",
         "-Wno-pointer-to-int-cast", "-Wno-int-to-pointer-cast", "-Wno-overflow",
         "-Wno-implicit-function-declaration", "-DSTM32F407xx", "-DUSE_HAL_DRIVER",
         "-DARM_MATH_CM4", "-D__FPU_PRESENT=1U", *includes,
         "tests/test_vision_transports.c", "application/vision_protocol.c",
         "-Wl,--gc-sections", "-lm", "-o", transports])
    run([transports])
