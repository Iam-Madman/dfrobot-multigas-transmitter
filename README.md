A multi-phase industrial instrumentation project developing a Modbus RTU-enabled gas monitoring system.

##  Repository Structure

The project is structured as a monorepo splitting hardware development from firmware ecosystems:

```text
.
├── fabrication/                 # PCB design files, schematics, and Gerber outputs
└── firmware/
    ├── arduino_nano_firmware/   # Phase 1: Rapid prototype built on PlatformIO
    └── bluepill_firmware/       # Phase 2: Production STM32 system (Bare-metal/FreeRTOS)

## Getting Started

Because this project is target-compiled for **clone STM32F103 microcontrollers** and debugged natively via the **Black Magic Probe**, special debugger configuration limits must be applied before launching.

Please navigate into the firmware folder for detailed step-by-step flashing, memory mapping, and debugging instructions:

 **[Go to Firmware Documentation & Setup Guide](./firmware/bluepill_firmware/README.md)**

