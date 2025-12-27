# STM32 UART Bootloader

A production-grade custom bootloader for STM32F407VG microcontroller with Python-based firmware update tool, enabling wireless firmware updates over UART without requiring an external programmer.

## Features

- **Dual-Program Architecture**: 32KB bootloader region + 992KB application space with sector-aligned memory partitioning
- **UART Communication**: 115200 baud serial protocol with ACK-based handshaking for reliable data transfer
- **Flash Management**: Erase and write operations using STM32 HAL drivers with proper error handling
- **Boot Mode Selection**: Hardware button-based bootloader entry for firmware update mode
- **Python Automation**: Complete firmware update script with progress reporting and error detection
- **User-Friendly Interface**: Command menus and status messages for both bootloader and application modes

## Hardware Requirements

- STM32F407VG-DISC1 Development Board
- CP2102 USB-to-TTL Serial Adapter
- USB Hub (if using single USB port)
- Mini USB Cable (for ST-Link programming)

## Software Requirements

- STM32CubeIDE (for building firmware)
- Python 3.x with `pyserial` library
- Serial terminal (PuTTY, Tera Term, or similar)
- STM32CubeProgrammer (for initial flashing)

## Memory Layout
```
0x0800 0000 ┬─ Bootloader (32KB, Sectors 0-1)
0x0800 8000 ┴─ Application (992KB, Sectors 2-11)
```

## Installation

### 1. Install Python Dependencies
```bash
pip install pyserial
```

### 2. Hardware Connections

**CP2102 Wiring:**
```
CP2102 GND  →  STM32 GND
CP2102 TXD  →  STM32 PA3 (RX)
CP2102 RXD  →  STM32 PA2 (TX)
```

### 3. Build Firmware

**In STM32CubeIDE:**
1. Import both `Bootloader` and `Application` projects
2. Build both projects (generates `.hex` files in `Debug/` folders)

### 4. Flash Initial Firmware

**Using STM32CubeProgrammer:**
1. Flash `Bootloader.hex` to address `0x08000000`
2. Flash `Application.hex` to address `0x08008000`

## Usage

### Normal Operation

1. **Power on** → Bootloader runs briefly → Application starts
2. Green LED blinks (application running)

### Firmware Update Mode

**Enter Bootloader:**
1. Hold USER button (blue button)
2. Press RESET button
3. Release USER button
4. Orange LED stays ON (bootloader mode)

**Update Firmware:**
```bash
python scripts/firmware_updater.py
```
- Enter COM port (e.g., COM4)
- Enter path to `.bin` file
- Wait for update to complete
- Press RESET to run new firmware

### Manual Commands (via Serial Terminal)

Connect to bootloader at 115200 baud:

- `E` - Erase application flash
- `W` - Write firmware (used by Python script)
- `T` - Test flash write with pattern
- `R` - Reset to application

## Architecture

### Bootloader Jump Mechanism

- Vector table relocation (`SCB->VTOR`)
- Stack pointer reconfiguration (`__set_MSP`)
- Complete HAL deinitialization
- SysTick timer reset

### Communication Protocol

1. Python sends 'W' command
2. Bootloader responds with ACK
3. Python sends 8-byte header (address + length)
4. Bootloader responds with HDR_OK
5. Python sends firmware data (256-byte chunks)
6. Bootloader writes to flash and responds OK/FAIL

## Project Structure
```
stm32-uart-bootloader/
├── Bootloader/          # Bootloader firmware
│   ├── Core/
│   └── Drivers/
├── Application/         # Application firmware
│   ├── Core/
│   └── Drivers/
├── scripts/
│   └── firmware_updater.py   # Python update tool
└── README.md
```

## Technical Details

- **Microcontroller**: STM32F407VGT6 (ARM Cortex-M4, 168MHz)
- **Flash**: 1MB organized in 12 sectors
- **UART**: USART2 (PA2/PA3, 115200 baud, 8N1)
- **Protocol**: Custom ACK-based with error detection
- **Chunk Size**: 256 bytes per packet

## Troubleshooting

**Bootloader not responding:**
- Verify USER button held during reset
- Check orange LED is ON
- Confirm correct COM port

**Firmware update fails:**
- Ensure board is in bootloader mode first
- Check CP2102 wiring (TX↔RX crossover)
- Verify `.bin` file path is correct

**Application doesn't run after update:**
- Check `.bin` file was built correctly
- Verify update completed 100%
- Try manual flash with STM32CubeProgrammer

## References

- [RM0090 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020.pdf) - STM32F407 peripheral details
- [PM0214 Programming Manual](https://www.st.com/resource/en/programming_manual/dm00046982.pdf) - ARM Cortex-M4 core
- [AN2606 Application Note](https://www.st.com/resource/en/application_note/cd00167594.pdf) - STM32 bootloader principles

## License

This project is provided as-is for educational purposes.

## Author

Built as a learning project to demonstrate embedded systems firmware development, bootloader design, and Python automation for hardware interfaces.