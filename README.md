# UART-FILE-TRANSFER

Wired and wireless file transfer via UART in STM32 based microcontroller.

The aim of this project is to perform the file transfer task in the model satellite system we have developed as Yıldız Rocket Team. Xbee modules with UART interface were used for file transfer. In the system, TEA is the ground station computer and LENNA is the file transfer computer on the satellite.

TEA features:
- Receive telemetry from the main flight control computer and send commands
- Communicate with the Python GUI app via the serial port
- Receive file from the GUI app, save it to the SD card, and then send this file to Lenna

LENNA features:
- Receive file from ground station and send it on another frequency

## Demo

[demo](https://www.linkedin.com/posts/do%C4%9Fukan-yal%C3%A7in-1a32b021a_stm32-satellite-embedded-activity-6981952442833756160-C_kC?utm_source=share&utm_medium=member_desktop)



