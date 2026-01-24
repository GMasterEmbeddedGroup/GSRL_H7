# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**GSRL_H7** is the GMaster Standard Robot Library, an embedded C++ robotics control library being migrated from STM32F4 to STM32H7 microcontrollers. It provides reusable control components for DJI Robomaster robots as middleware between hardware abstraction and user applications (chassis/gimbal logic).

**Important**: GSRL is NOT a complete robot application - it's a library that user applications build upon.

## Build Commands

```bash
# Configure build (from workspace root)
cmake --preset Debug

# Build project
cmake --build --preset Debug

# Flash to target via OpenOCD
openocd -f flash.cfg
```

- Build presets are in `CMakePresets.json` (Debug/Release)
- Toolchain: `CubeMX_BSP/cmake/gcc-arm-none-eabi.cmake`
- Output: `build/Debug/` with `compile_commands.json` for IDEs

## Architecture

Three-layer design:

1. **Algorithm Layer** (`GSRL/Algorithm/`)
   - Controllers: SimplePID implementing `Controller` interface
   - Filters: KalmanFilter, LowPassFilter, MovingAverageFilter implementing `Filter<T>` template
   - AHRS: Mahony filter for attitude estimation
   - Math: Matrix operations, quaternion math

2. **Device Layer** (`GSRL/Device/`)
   - Motor abstractions: MotorGM6020, MotorM3508, MotorDM4310 inheriting from `Motor` base class
   - IMU: BMI088 integration
   - Combines drivers with control algorithms

3. **Driver Layer** (`GSRL/Driver/`)
   - CAN/FDCAN communication with callback-driven architecture
   - SPI, UART, GPIO wrappers around STM32 HAL

**Data Flow**: CubeMX BSP (HAL) → GSRL Drivers → GSRL Devices → Algorithm/Controllers → User Application Tasks

## Critical Patterns

### Motor Control Abstraction

All motors inherit from `Motor` base class and must implement:
- `convertControllerOutputToMotorControlData()` - Convert control output to CAN message data
- `decodeCanRxMessage()` - Parse CAN feedback and update connection status

Example: `GSRL/Device/inc/dvc_motor.hpp` shows inheritance pattern for motor classes.

### CAN/FDCAN Communication

Callback-driven architecture:
1. Initialize CAN with `CAN_Init(hfdcan, rxCallbackFunction)` from `drv_can.h`
2. RX messages queued via `canRxQueueHandle` (FreeRTOS queue)
3. Callbacks process messages in device objects (motors, sensors)

Use `can_rx_message_t` structure with `FDCAN_RxHeaderTypeDef` + 8-byte data array.

### Interface-Based Design

- Controllers implement `Controller` interface with `controllerCalculate()`
- Filters implement `Filter<T>` template interface
- Standardized method signatures across implementations

## Important Conventions

### File Creation
- **Always use templates in `GSRL/Template/`** - they contain required copyright headers and standard structure
- Use `#pragma once` for C++ headers (not `#ifndef` guards)

### Peripheral Configuration
Edit `GSRL/Include/board_config.h` to enable/disable peripherals:
```cpp
#define USE_FDCAN1  // CAN bus 1
#define USE_FDCAN2  // CAN bus 2
#define USE_SPI1    // SPI interface 1
```

### Type Usage
Use standard type aliases from `std_typedef.h`:
- `fp32` / `fp64` (not `float` / `double`)
- `bool_t` (not `bool`)

### STM32CubeMX Integration
1. Edit `CubeMX_BSP/GSRL_H7.ioc` in STM32CubeMX GUI
2. Regenerate code (preserves USER CODE sections)
3. No CMake regeneration needed - HAL sources auto-discovered

## STM32F4 → H7 Migration

This project is migrating from STM32F4 to STM32H7. Key differences:
- `hcan` → `hfdcan` (H7 uses FDCAN peripheral)
- `HAL_CAN_*` → `HAL_FDCAN_*` API changes
- Different clock tree and higher frequencies
- Verify DMA and cache settings for H7's architecture

## Dependencies

- **Eigen**: Header-only linear algebra, compiled with `EIGEN_NO_MALLOC` to prevent dynamic allocation
- **CMSIS-DSP**: ARM DSP library in `GSRL/Dependence/CMSIS-DSP/`
- **FreeRTOS**: CMSIS-RTOS v1 wrapper, configured in `CubeMX_BSP/Core/Inc/FreeRTOSConfig.h`

## RTOS Integration

GSRL is designed for FreeRTOS. User applications implement their own tasks using GSRL components. Example task patterns (from `CubeMX_BSP/Core/Src/freertos.c`):
- Realtime priority: High-frequency control loops (gimbal)
- High priority: Chassis/motion control
- AboveNormal: Communication protocols
- Normal: Diagnostics, UI, logging

## Common Tasks

### Adding New Motor Type
1. Create class inheriting from `Motor` in `GSRL/Device/inc/` and `GSRL/Device/src/`
2. Implement virtual functions: `convertControllerOutputToMotorControlData()`, `decodeCanRxMessage()`
3. Register CAN callback in device initialization
4. Update `board_config.h` if using new CAN bus

### File Headers
All files must include:
```cpp
/**
 * @attention
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 */
```
