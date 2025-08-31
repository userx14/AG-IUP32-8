# Steps for reprogramming the PIC16 microcontroller

## Preparation
Solder a 3x2 pin header to the underside of the board, the casing has enough space to accomodate this in the assembled state.
Disconnect the D-SUB-15 mixed input connector, make sure the board is not powered. Connect GND, MCLR, PGC, PGD, 5V from the PIC programmer to the board.

<image src="./ICP_header.svg">

## Backup the original firmware
In case you want to restore the original firmware create a backup of flash, eeprom and config bits. 
The PIC should not be read protected.
  
## Compile and upload
Use MPLAB X IDE to compile the firmware. In the `dist\default\production` folder you will find the firmware `.hex` file. 
Program eeprom, flash and the config bits. Verify that the flash was programmed and disconnect the programmer. 
