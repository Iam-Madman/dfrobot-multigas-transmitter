
## Debugging Clone STM32F103 Chips via Black Magic Probe (BMP)

This project utilizes a **clone/non-STMicroelectronics microcontroller** (such as a CKS32F103 or CS32F103 with Designer ID `0x80e` and Part ID `0x410`) on a Bluepill form factor.

Because the Black Magic Probe (BMP) identifies this target as an unknown ARM Cortex-M device, it cannot natively parse the chip's flash size or memory layout. To debug or attach to FreeRTOS tasks successfully without GDB or the IDE extension crashing, you **must manual override the memory map and layout settings.**

---

##  Prerequisites & Hardware Setup

### 1. Black Magic Probe Initialization

Before attempting to debug, make sure your probe is correctly provisioned, updated, and that your local user has permissions to access the serial ports (`/dev/ttyBmpGdb` and `/dev/ttyBmpTarg`).

* Refer to the official documentation for first-time native setup: [Black Magic Probe Getting Started Guide](https://black-magic.org/getting-started.html)

### 2. Project Configuration via STM32CubeMX

When updating peripheral layouts, pinouts, or clock trees:

* Open the `.ioc` configuration file in **STM32CubeMX**.
* Under **System Core** -> **SYS**, ensure that **Debug** is explicitly configured to **Serial Wire** (otherwise, the MCU will disable its own SWD pins on the first boot, locking you out).
* Regenerate the code template to cleanly merge changes into your existing initialization structure.

Here is the section updated to explicitly include both the **CLI (Command Line Interface)** and **GUI (Graphical User Interface)** options for STM32CubeProgrammer:

---

### 3. Flashing & Recovering via STM32CubeProgrammer (GUI or CLI)

Because clone chips can exhibit erratic flash controller behaviors during active debug sessions, GDB may occasionally fail to erase or program the chip. Use **STM32CubeProgrammer** via an ST-LINK V2 interface as an out-of-band utility to completely wipe or force-flash the target binary.

* **Using the GUI Application:**
1. Open the **STM32CubeProgrammer** graphical interface.
2. Set the connection type to **ST-LINK** and interface to **SWD**.
3. Change the **Mode** dropdown to **Under Reset** (this pulls the physical NRST line low to bypass execution crashes or active locks).
4. Connect, use the **Full chip erase** button (trash can icon) if the chip is completely locked up, or open your built `.elf`/`.bin` file to program and run.


##  Required Manual GDB Sequence

When initializing a raw GDB session (`arm-none-eabi-gdb`), execute the following commands in order to connect, configure the memory spaces, and attach to the target:

```text
# 1. Connect to the Black Magic Probe GDB Server port
target extended-remote /dev/ttyBmpGdb

# 2. Scan the SWD interface for the target micro
monitor swdp_scan

# 3. Attach to the discovered target ID (usually 1)
attach 1

# 4. Disable GDB restriction on unmapped memory regions
set mem inaccessible-by-default off

# 5. Force-define the Flash (Read-Only) and RAM (Read-Write) boundaries for the Clone
mem 0x08000000 0x0801FFFF ro
mem 0x20000000 0x20004FFF rw

# 6. Enable hardware breakpoints automatically for Read-Only memory locations
set breakpoint auto-hw on

# 7. Enable clean, formatted structure printouts
set print pretty on

```

---