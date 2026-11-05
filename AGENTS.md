# Repository Guidelines

## Project Structure & Module Organization

This repository contains STM32G474VETx firmware built with STM32 HAL and Keil µVision.

- `Core/Src/`: application and peripheral code. `main.c` initializes hardware and handles timer-driven control; `sampling.c` processes ADC data; `single_rectifier_control.c` implements the voltage PLL. `Control_Tools.c`, `Keyboard.c`, and `oled.c` provide supporting functions.
- `Core/Inc/`: matching headers and configuration. Keep implementation-only PLL state in `single_rectifier_control_private.h`.
- `Drivers/`: vendor HAL/CMSIS code; avoid unrelated changes.
- `MDK-ARM/`: Keil project, startup files, and build outputs.
- `tests/`: host checks and peripheral mocks.
- `g474_code.ioc`: CubeMX configuration. OLED font assets are embedded in `oled.c`.

## Build, Test, and Development Commands

Run from the repository root:

```powershell
UV4.exe -b MDK-ARM\g474_code.uvprojx -t g474_code -o build.log
cmd /c tests\run_checks.cmd
```

The firmware project selects Arm Compiler 6.24. If µVision is unavailable on PATH, open the project and build target `g474_code` with F7. Inspect the build log for errors and warnings. Download through the configured debug probe.

The host runner selects MSVC or GCC and writes executables under `%TEMP%`. Currently, `inverter_check.c` references removed inverter modules, so the suite needs reconciliation before it can validate this checkout.

## Coding Style & Naming Conventions

Follow surrounding C code: two-space indentation in generated sections and four spaces in custom modules. Preserve CubeMX `USER CODE` blocks. Use uppercase macros, module-prefixed public functions, and `static` internal helpers and state. No repository formatter or linter is configured. Add new compilation units explicitly to the Keil project; synchronize peripheral changes with `.ioc` and pin definitions.

## Testing Guidelines

Existing standalone C checks use assertions and `*_check.c` names; no coverage threshold is configured. Run applicable checks and build firmware after code changes. Add test, monitoring, or protection code only when explicitly requested. Record board, wiring, and measurements separately: host checks and successful builds do not establish electrical behavior.

## Commit & Pull Request Guidelines

History uses short descriptive English and Chinese subjects without a fixed prefix. Keep commits focused; exclude generated binaries and personal IDE settings. PRs should explain behavior changes, affected peripherals or pins, validation, and relevant issues. Include hardware photos for display changes when available, and state when hardware testing was not performed.

## Agent Instructions

Re-read current files and inspect `git status` before reviewing or editing. Preserve manual and unrelated changes. Implement only the requested scope; avoid speculative refactoring or additional safeguards.
