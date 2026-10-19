
# ZX-picoPrinter

## Project Description
ZX-picoPrinter is a DIY replacement for the ZX printer that works with the original Spectrum LPRINT, LLIST and COPY command. It is built on a custom circuit board and a raspberry Pi is used to emulate the printer and inteface with the POS printer and screen.

https://github.com/user-attachments/assets/79f8de29-5155-4037-83c5-f92d454931f6

## Features
- Optional screen to preview the print

## Hardware Requirements
- Raspberry Pi Pico
- Compatible thermal printer
- Power supply for the printer
- Connecting cables (USB or serial)
- Custom circuit board 
- Logic chips

## Getting Started Guide
1. **Prerequisites**: Make sure you have installed the necessary software tools for Raspberry Pi Pico development. (VSC with official PI plugin was used)
2. **Clone the Repository**: ```git clone https://github.com/luwii/ZX-picoPrinter.git``` 
3. **Connect the Hardware**: Follow the wiring instructions provided in the documentation. The screen and printer work indepently, you only need one of them
4. **Upload the Code**: Use your preferred development environment to upload the provided codes to the Raspberry Pi Pico.
5. **Run the Application**: Start the application and test with LPRINT

## Documentation References
- [Printer Manual](https://github.com/luwii/ZX-picoPrinter/blob/main/Documentation/zxpicoprinter.pdf)
- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)


## License
This project is licensed under the MIT License. See the LICENSE file for more details.
