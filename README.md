# DFRobot Multi-Gas Transmitter

An industrial-grade, multi-phase gas monitoring system implementing the Modbus RTU protocol over RS-485. This repository serves as a monorepo containing everything from the initial rapid hardware prototypes to production-ready firmware and hardware fabrication files.

---

## 📂 Repository Structure

The project separates hardware development from firmware development ecosystems across distinct execution phases:

```text
.
├── fabrication/                 # PCB design files, schematics, and manufacturing outputs (Gerbers)
└── firmware/
    ├── arduino_nano_firmware/   # Phase 1: Rapid evaluation prototype built using PlatformIO
    └── bluepill_firmware/       # Phase 2: Production STM32 system (Bare-metal / FreeRTOS)
