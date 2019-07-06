Joe's Drive, MK3Remote:SHADOW :  Small Handheld Arduino Droid Operating Wand for BB8

Last Revised Date: 03/21/2019
Written By: Joe Latiola
Adapted to Shadow by: James VanDusen
Inspired by KnightShade for R2 Astromech development
learn more at http://jimmyzsbb8.blogspot.com or http://jimmyzsr2.blogspot.com
This is written to be a UNIVERSAL Sketch - supporting multiple controller options
for the Joe Latiola BB8 build version MK3...
//   see: www.facebook.com/joesdrive
//   learn more at http://jimmyzsbb8.blogspot.com or http://jimmyzsr2.blogspot.com
//
//   Latest Changes
//      - MK1 to MK3 adaptability
//   Working on
//      - QWiiC integration between Drive, MPU and SHADOW controller cards
//





//  You will need libraries: 
// i2cdevlib: https://github.com/jrowberg/i2cdevlib
// SSD1306Ascii Text Only: https://github.com/greiman/SSD1306Ascii
// EepromEX: https://github.com/thijse/Arduino-EEPROMEx
// USB Shield 2.0 Library: https://github.com/felis/USB_Host_Shield_2.0
// SFX Module Library: https://github.com/adafruit/Adafruit_Soundboard_library
// OPTIONAL: RF nRF24L01 Module Library: https://github.com/nRF24/RF24
// Support Tutorial on RF nRF24L01 by Dejan Nedelkovski
// https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
// SHADOW Reference information: https://github.com/ti9327/SHADOW
// Modified Low PowerLab: https://travis-ci.org/LowPowerLab/RFM69  
// EasyTransfer: https://github.com/madsci1016/Arduino-EasyTransfer

// Serial0 = Debug
// Serial1 = Main board over 115200
// Serial3 = SFX over 9600
// SPI     = RF nRF24L01 transcievers for DOME
// =======================================================================================
