# Joe's Drive, MK3Remote:SHADOW :  Small Handheld Arduino Droid Operating Wand for BB8
* Shared Code Base for Joe's Drive MK1-MK3 
* Initial creation: 3/21/2019
* Last Revised Date: 10/01/2022
* Written By: Joe Latiola
* Adapted to Shadow by: James VanDusen
* Inspired by KnightShade for R2 Astromech development
* learn more at http://jimmyzsbb8.blogspot.com or http://jimmyzsr2.blogspot.com
* utilizes 2x Arduino Mega and MP3050 based CPU and IMU units
* This is written to be a UNIVERSAL Sketch - supporting multiple controller options
* Land, Sea and Air control schemas supporting Wifi, BT and more...
* This will contain all the code for my Joe's drive up to MK3 version
* for the Joe Latiola BB8 build version MK3...


## Software setup
This drive includes all of Joes' Original code converted verbatum back to SHADOW methods for use with Xbox or PS3 Move Navigation controllers similar to what exists in the astromech world with R2 units.

### FUTURE
QWiiC integration between Drive, MPU and SHADOW controller cards

###  You will need libraries:
* i2cdevlib: https://github.com/jrowberg/i2cdevlib
* i2cdevlib: https://github.com/jrowberg/i2cdevlib
* SSD1306Ascii Text Only: https://github.com/greiman/SSD1306Ascii
* EepromEX: https://github.com/thijse/Arduino-EEPROMEx
* USB Shield 2.0 Library: https://github.com/felis/USB_Host_Shield_2.0
* SFX Module Library: https://github.com/adafruit/Adafruit_Soundboard_library
* OPTIONAL: RF nRF24L01 Module Library: https://github.com/nRF24/RF24
* Support Tutorial on RF nRF24L01 by Dejan Nedelkovski
* https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
* SHADOW Reference information: https://github.com/ti9327/SHADOW
* Modified Low PowerLab: https://travis-ci.org/LowPowerLab/RFM69  
* EasyTransfer: https://github.com/madsci1016/Arduino-EasyTransfer

