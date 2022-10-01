# UART-FILE-TRANSFER

Wired and wireless file transfer via UART in STM32 based microcontroller.

The aim of this project is to perform the file transfer task in the model satellite system. Xbee modules with UART interface were used for file transfer. In the system, TEA is the ground station computer and LENNA is the file transfer computer on the satellite.

TEA features:
- Receive telemetry from the main flight control computer and send commands
- Communicate with the Python GUI app via the serial port
- Receive file from the GUI app, save it to the SD card, and then send this file to Lenna

LENNA features:
- Receive file from ground station and send it on another frequency

## Demo

linkedin hesabi

