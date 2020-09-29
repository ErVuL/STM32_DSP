# STM32_DSP
 
STM32F407G audio I2S DSP + USB Serial Communication. 
STM32CubeIDE 1.4.0 project.

Audio device
============

The audio device used here is a PmodI2S2.
Audio is set to 24 bits with a 93.75 kHz frequency sampling.  
The peripheral is connected as follow:
 _________________       ______________
 |  STM32F407VG  |       |  PmodI2S2  |
 |               |       |            |
 |            PC2|-------|A/D SDOUT 10|
 |            PC3|-------|D/A SDIN 4  |
 |            PC6|-------|MCLK 1&7    |
 |           PB10|-------|SCLK 3&9    |
 |           PB12|-------|LRCK 2&8    |
 |            3v3|-------|VCC 6 or 12 |
 |            GND|-------|GND 5 or 11 |
 |_______________|       |____________|
          
Serial Communication
====================

Serial communication is enable through the microUSB with CDC virtual com port.
Blue led is set if the host port is open.

Serial COM proprieties :
    - Baudrate = 115200
    - data bits = 1 byte
    - stop bits = 1
    - parity = none
    - Flow control: XON/XOFF


Potential issues
================
If the *.ioc file is used to modify/generate code, there may have some errors in the generated code.
It depend on the version of STMCubeIDE you use.
To correct the code, please refer to the BUG_XX.png pictures in the screenshot folder.
