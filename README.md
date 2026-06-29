# STM32 FreeRTOS Six-Axis Robot Arm Motion Control System

> 基于STM32F407 + FreeRTOS的六轴机械臂运动控制系统

[![Platform](https://img.shields.io/badge/Platform-STM32F407VET6-blue)](https://www.st.com/)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-brightgreen)](https://www.freertos.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

A fully embedded six-axis robot arm motion control system. All kinematics algorithms, trajectory planning, and closed-loop control run directly on the **STM32F407** MCU — **no PC-side computation required**. The system supports Cartesian linear motion, multi-axis synchronized control, and serial port command interface, achieving industrial-grade real-time performance on a low-cost embedded platform.

📺 **Demo Video**: [Douyin Link](https://v.douyin.com/_v1MuLGhd9w/)

> **Note**: The mechanical structure is inspired by open-source hardware designs. All software algorithms, control logic, and embedded code in this repository were independently developed from scratch.

---

## Why This Project?

Most hobbyist robot arms rely on a PC to compute inverse kinematics and merely forward servo commands from the microcontroller. This project pushes **everything** — kinematics, trajectory interpolation, PID control — down to the STM32, achieving true standalone embedded control with a **20 ms real-time cycle**.

---

## System Architecture

```
┌──────────────────────────────────────────────────┐
│                Application Layer                  │
│  Serial Command Parser  │  Action Sequence Engine  │
│  (USART protocol)       │  (app_robot)            │
├──────────────────────────────────────────────────┤
│                Control Layer (FreeRTOS)            │
│  ┌────────────┬──────────────┬────────────────┐  │
│  │ Trajectory │ Kinematics   │ Motor Control  │  │
│  │ Planner    │ Engine       │ (PID Loop)     │  │
│  │ (S-curve)  │ (DH + DLS IK)│                │  │
│  │ 20ms task  │              │                │  │
│  └────────────┴──────────────┴────────────────┘  │
├──────────────────────────────────────────────────┤
│                Driver Layer                       │
│  CAN Driver (HAL)  │  USART Driver (HAL)          │
├──────────────────────────────────────────────────┤
│                Hardware                           │
│  STM32F407VET6     │  6× Emm_V5 Closed-Loop       │
│  (168MHz + FPU)    │  Stepper Motors (CAN bus)     │
└──────────────────────────────────────────────────┘
```

## Key Features

### Kinematics Engine
- **Forward Kinematics**: Modified DH parameter model for 6-DOF serial manipulator
- **Inverse Kinematics**: DLS (Damped Least Squares) numerical solver — replaces traditional analytical solutions, resolves singularity divergence issues
- Trajectory precision: ≤ 1 mm straight-line error, ±0.5° repeatability

### Trajectory Planning
- **Quintic S-curve interpolation**: Position/Velocity/Acceleration all C²-continuous (no jerk discontinuity)
- **RPY shortest-path attitude interpolation**: Synchronized orientation blending during Cartesian motion
- **Pose-hold linear motion**: End-effector orientation maintained while moving along straight line

### Real-Time Control (FreeRTOS)
- 3-priority task scheduling: Trajectory (20 ms) > CAN Receive > Serial Parse
- Thread-safe communication via message queues + mutexes
- Joint velocity P-controller with velocity limiting + slope limiting
- Encoder error calibration every 100 ms — zero-impact start/stop

### CAN Bus Motor Control
- HAL-based CAN driver for Emm_V5 closed-loop stepper motors
- 13 command types: position mode, velocity mode, homing, enable, parameter R/W, etc.
- CAN frame protocol: Address + Function Code + Parameters + Checksum
- Calibrated gear ratios, direction mapping, and CAN ID assignment for all 6 joints

### Communication
- **CAN Bus** (motor control): Differential signaling, multi-node, hardware CRC — ideal for distributed joint topology
- **USART** (host interface): Command parsing for angle control, trajectory execution, parameter tuning

## Repository Structure

```
six-axis-robot-arm/
├── Core/
│   ├── Inc/                        # Headers
│   │   ├── robot_kinematics.h      # DH forward kinematics
│   │   ├── robot_ik_dls.h          # DLS inverse kinematics
│   │   ├── trajectory_planner.h    # S-curve + RPY interpolation
│   │   ├── MotorControl.h          # Joint PID + motor abstraction
│   │   ├── Emm_V5.h                # Emm_V5 CAN protocol
│   │   ├── app_robot.h             # Application logic
│   │   ├── can.h                   # CAN HAL driver
│   │   ├── usart.h                 # USART HAL driver
│   │   └── FreeRTOSConfig.h        # RTOS configuration
│   └── Src/                        # Sources
│       ├── robot_kinematics.c
│       ├── robot_ik_dls.c
│       ├── trajectory_planner.c
│       ├── trajectory_planner_port.c
│       ├── MotorControl.c
│       ├── Emm_V5.c
│       ├── app_robot.c
│       ├── freertos.c              # Task creation + scheduling
│       ├── can.c
│       ├── usart.c
│       └── main.c
├── Middlewares/
│   └── Third_Party/FreeRTOS/       # FreeRTOS kernel
├── STM32F4xx_HAL_Driver/           # STM32 HAL library
├── CMSIS/                          # Cortex-M4 CMSIS
├── Startup/                        # Startup code (startup_stm32f407vetx.s)
└── 机器人Freertos.ioc              # STM32CubeIDE project file
```

## Performance Metrics

| Metric | Value |
|--------|-------|
| Control cycle | 20 ms (50 Hz) |
| CAN bus speed | 1 Mbps |
| Straight-line trajectory error | ≤ 1 mm |
| Repeat positioning accuracy | ± 0.5° |
| Joint velocity control | P-controller + limiting + slope guard |
| Kinematics solver | DLS damped least squares |
| Trajectory interpolation | 5th-order S-curve (C² continuous) |

## Development Environment

| Tool | Version/Purpose |
|------|----------------|
| IDE | STM32CubeIDE |
| MCU | STM32F407VET6 (168 MHz, Cortex-M4 + FPU) |
| RTOS | FreeRTOS (CMSIS-RTOS v2) |
| Library | STM32F4 HAL |
| Language | C |
| Debugger | ST-Link / Serial print |

## Quick Start

1. **Clone and open**: Import the `.ioc` project file into STM32CubeIDE
2. **Build**: Compile all source files (no errors, no warnings)
3. **Flash**: Connect ST-Link, download firmware to STM32F407
4. **Run**: Power on — arm auto-initializes to home position
5. **Control**: Send commands via USART serial terminal (115200 baud)

### Serial Command Examples
```
# Single joint angle control
J1 45.0     # Move joint 1 to 45 degrees

# Cartesian linear motion (pose-hold)
L X150 Y0 Z200 ROLL0 PITCH-90 YAW0 SPEED50

# Execute stored action sequence
A GRAB      # Run predefined "grab" sequence
```

## Technical Stack

**Core**: C language, STM32F407 (HAL), FreeRTOS, CMSIS-RTOS v2

**Algorithms**: DH kinematics, DLS inverse kinematics, 5th-order S-curve interpolation, RPY Euler angle interpolation, PID joint control

**Communication**: CAN bus (motor control), USART (host interface)

**Build**: STM32CubeIDE, ARM GCC toolchain

## Future Roadmap

- [x] 6-DOF DH forward kinematics + DLS inverse kinematics
- [x] S-curve trajectory planner with RPY attitude interpolation
- [x] FreeRTOS multi-task real-time scheduling
- [x] CAN bus Emm_V5 motor protocol (13 commands)
- [x] Serial command parser + action sequence engine
- [ ] PC visualization & tuning tool (in progress)
- [ ] ROS2 + MoveIt integration for trajectory generation
- [ ] Vision-guided grasping extension

## Related Projects

- [ROS Omnidirectional Mobile Manipulator](https://github.com/mz0210/ros-omnidirectional-arm-robot) — Raspberry Pi + ROS + OpenCV + Mecanum chassis
- Same author, complementary tech stack: **MCU bare-metal** (this repo) vs **Linux ROS integration** (above)

## License

MIT License — freely usable for learning, research, and non-commercial development. Attribution appreciated.

---

**Author**: Meng Haozhe (孟浩哲)  
**GitHub**: [@mz0210](https://github.com/mz0210)  
**Contact**: Open an Issue for questions or collaboration.
