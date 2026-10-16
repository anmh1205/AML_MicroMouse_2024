# AML MicroMouse 2024

A Micromouse robotics development repository combining STM32-based firmware variants, maze-solving algorithms, hardware design, simulation, and engineering records from the 2024 project.

## Project overview

Micromouse is a robotics challenge that combines mapping an unknown maze with planning and accurately driving a route to the destination. The 2024 project explores both sides of that problem: embedded-control software for the physical robot and software tools for developing and testing maze-solving behavior.

Unlike a single-board firmware example, this repository preserves **multiple hardware and firmware variants**, development experiments, technical references, and documentation. A directory name may refer to an earlier board revision rather than the final target.

## What is included

| Area | Examples found in the repository |
| --- | --- |
| Embedded firmware | STM32 projects, startup files, HAL drivers, MCU configurations |
| MCU variants | STM32F407-related projects and an STM32H743-based all-in-one board project |
| Real-time software | FreeRTOS experiments and related MCU project material |
| Navigation | Maze-solving and simulator-oriented development |
| Supporting devices | ESP32 screen and configuration-tool experiments |
| Hardware | PCB designs, drawings, and component references |
| Project records | Planning documents, calibration spreadsheets, and presentations |

The repository's `main` branch is an archival development snapshot. Individual board variants may require different MCU targets, clocks, pin assignments, and toolchains.

## Finding the correct firmware project

The main firmware workspace is under [`1. MicroMouse Code/`](1.%20MicroMouse%20Code/). For example:

- `2. MicroMouse - STM32F407VGT6/PCB_V1/` includes STM32 project files and an `.ioc` MCU configuration.
- `2. MicroMouse - STM32F407VGT6/PCB_AIO_V2_CODE/` includes a separate project configured for an STM32H743 device.
- `2. MicroMouse - STM32F407VGT6/FreeRTOS/` holds RTOS-related experiments.

Select a project **by the MCU and board configuration inside it**, not just by its parent folder name.

### Build workflow

1. Identify the matching PCB revision and microcontroller.
2. Open the relevant STM32CubeIDE/STM32CubeMX or CMake project according to the files in that variant.
3. Confirm the toolchain, generated configuration, linker script, libraries, and debugger interface.
4. Compile before connecting the physical robot; then verify pin mapping, sensor interfaces, and motor outputs before flashing.

Because the repository includes separate historical experiments and generated build artifacts, there is no single verified command that builds every directory.

## Repository guide

- [Examples and references](0.%20Example/) — datasheets, sample code, tool downloads, and experiments.
- [Firmware variants](1.%20MicroMouse%20Code/) — controller projects, ESP32-related code, and maze-solving work.
- [Electronics](2.%20MicroMouse%20PCB/) — PCB and supporting hardware files.
- [Drawings](3.%20Drawing/) — mechanical design materials.
- [Technical documentation](4.%20Document/) — reports, planning spreadsheets, and supporting materials.

## Relationship to the earlier project

[AML_MicroMouse](https://github.com/anmh1205/AML_MicroMouse) documents an earlier STM32F407VET6 Micromouse project, with competition footage and simulator materials. This repository preserves the 2024 development work and its additional hardware and firmware experiments.

## Scope and reuse

The included code and documents are engineering reference material, not a prepackaged firmware release. Third-party resources under the examples and simulator directories may have their own licenses. No repository-wide license is currently declared.
