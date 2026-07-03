# DFRobot Multi-Gas Transmitter

An industrial-grade, multi-phase gas monitoring system implementing the Modbus RTU protocol over RS-485. This repository serves as a monorepo containing everything from the initial rapid hardware prototypes to production-ready firmware and hardware fabrication files.

---

## Repository Structure

The project separates hardware development from firmware development ecosystems across distinct execution phases:

```text
.
├── fabrication/                 
└── firmware/
    ├── arduino_nano_firmware/   
    └── bluepill_firmware/       
