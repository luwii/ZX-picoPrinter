





# ZX-picoPrinter

**ZX-picoPrinter** is a modern hardware emulator and DIY replacement for the vintage Sinclair ZX Printer. Built around a custom circuit board and a Raspberry Pi Pico 2, it restores native printing capabilities to the ZX Spectrum by translating legacy signals for modern thermal POS printers and optional display screens.

https://github.com/user-attachments/assets/f9424bd6-6a39-4c2c-87f9-4fa740e79a30
https://github.com/user-attachments/assets/79f8de29-5155-4037-83c5-f92d454931f6

---

## Key Features

- **Full Command Compatibility**: Works seamlessly with native ZX Spectrum Sinclair BASIC commands, including `LPRINT`, `LLIST`, and `COPY`.

- **Thermal POS Printing**: Outputs directly to standard, cost-effective thermal receipt printers via a serial connection.

- **Optional Preview Display**: Real-time on-screen display of print jobs.

- **Modular Hardware Design**: The display screen and thermal printer operate independently—you can run either device alone or both simultaneously.

---

## Hardware Requirements

- **Microcontroller**: Raspberry Pi Pico 2

- **Printer**: Compatible thermal POS receipt printer

- **Power**: Dedicated power supply matching your thermal printer's specifications (usually you need a lot of current)

- **Interface**: Custom ZX-picoPrinter circuit board and associated logic ICs

- **Cables**: USB connection cable to program the pico.

- **Display (*Optional*)**: Supported screen module for print previews

---

## Getting Started

### 1. Prerequisites

Install your preferred development environment for the Raspberry Pi Pico[cite: 1]. (This project was developed using **Visual Studio Code** with the official **Raspberry Pi Pico Extension**).

### 2. Clone the Repository

```bash

git clone [https://github.com/luwii/ZX-picoPrinter.git](https://github.com/luwii/ZX-picoPrinter.git)

```

### 3. Assemble and Wire Hardware

Assemble the custom PCB, install the logic chips, and complete the wiring following the diagrams in the [Documentation](https://github.com/luwii/ZX-picoPrinter/blob/main/Documentation/zxpicoprinter.pdf).

> **Note**: The display screen and thermal printer operate independently; you only need to connect the output device(s) you plan to use.

### 4. Flash the Firmware

Build and upload the provided firmware to the Raspberry Pi Pico using your IDE or by copying the compiled `.uf2` file.

### 5. Verify and Test

Power on the ZX Spectrum and verify functionality by executing:

```basic

LPRINT "ZX-picoPrinter Ready"

```

---

## Documentation & Resources

- [ZX-picoPrinter Manual (PDF)](https://github.com/luwii/ZX-picoPrinter/blob/main/Documentation/zxpicoprinter.pdf)[cite: 1]

- [Official Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)[cite: 1]

---

## License

This project is open-source software licensed under the [MIT License](LICENSE).
