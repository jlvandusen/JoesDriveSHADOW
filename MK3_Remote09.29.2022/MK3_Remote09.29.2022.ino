// =======================================================================================
//  Joe's Drive, MK3Remote:SHADOW :  Small Handheld Arduino Droid Operating Wand for BB8
// =======================================================================================
//                          Last Revised Date: 9/29/2022
//                             Written By: Joe Latiola
//                             Adapted to Shadow by: James VanDusen
//                             Inspired by KnightShade for R2 Astromech development
//     learn more at http://jimmyzsbb8.blogspot.com or http://jimmyzsr2.blogspot.com
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it .
//         This program is distributed in the hope that it will be useful,
//         but WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//         This entire project was masterminded by an average Joe, your mileage may vary. 
// =======================================================================================
//   Note: You will need a Adafruit Feather to run this sketch,
//
//   This is written to be a UNIVERSAL Sketch - supporting multiple controller options
//   for the Joe Latiola BB8 build version MK3...
//   see: www.facebook.com/joesdrive
//   learn more at http://jimmyzsbb8.blogspot.com or http://jimmyzsr2.blogspot.com
//
//   Latest Changes
//      - MK1 to MK3 adaptability
//   Working on
//      - QWiiC integration between Drive, MPU and SHADOW controller cards
//
// =======================================================================================
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
// Adafruit Soundboard: https://github.com/adafruit/Adafruit_Soundboard_library
// RFM69: https://github.com/LowPowerLab/RFM69
// SPIFLASH: https://github.com/LowPowerLab/SPIFlash

// Serial0 = Debug
// Serial1 = Main board over 115200
// Serial3 = SFX over 9600
// SPI     = RF nRF24L01 transcievers for DOME
// =======================================================================================

// =======================================================================================
// Configuration Options
// =======================================================================================
#define SHADOW
//#define MK2
#define MK3
//#define serialSound   //Uncomment this if you are using Serial3 for sounds in the body
//#define disablePSIflash // Uncomment to remove the PSI flash.           
//#define TiltDomeForwardWhenDriving // uncomment this if you want to tilt the dome forward when driving. 
//#define DEBUG
//#define debugPSI

#define droidName "BB-8"
#define battPin       A9
#define enablePIN     1
#define lJoySelectPIN 13
#define Joy2XPIN      A1
#define Joy2YPIN      A0
#define lBut1PIN      10
#define lBut2PIN      11
#define lBut3PIN      12
#define rJoySelectPIN 0
#define Joy1XPIN      A4
#define Joy1YPIN      A5
#define rBut1PIN      6
#define rBut2PIN      5
#define rBut3PIN      9    //   This is the only button / pin that must not change. 
#define Joy3XPIN      A3
#define Joy4XPIN      A2
#define dataDelay     0
#define sendDelay     20
#define recDelay      2

#ifdef SHADOW
// =======================================================================================
// These only matter if you're using SHADOW for sounds
// =======================================================================================
  #define SFX_RST   37
  #define SFX_ACT   34          // Pin connected to ACT on soundboard
  #define reset_Pin 40
  #define fadePin   A2          // Connected to + of one channel on sound board(use resistor to ground)
  #define soundpin1 26          // Connected to sound pin 0
  #define soundpin2 28          // Connected to sound pin 1
  #define soundpin3 30          // Connected to sound pin 2
  #define soundpin4 32          // Connected to sound pin 3
  #define soundpin5 46          // Connected to sound pin 4
  #define soundpin6 44          // Connected to sound pin 5

// =======================================================================================
// These only matter if you're using SHADOW for sounds
// =======================================================================================
#define numberOfVoice   50        // This is the number of 'voice' sound files NOT INCLUDING Music and such
#define numberOfMusic   6         // This is the number of 'music' files

// Below are used for the multipress button sounds. 
// Pressing button 1 on the left or button 3 on the right once plays a speach track at random
// pressing 2-6 times will play quickVoice1-5. 
#define quickVoice1     6
#define quickVoice2     8
#define quickVoice3     20
#define quickVoice4     22
#define quickVoice5     1

// Below are used for the multipress button sounds. 
// Pressing button 2 on the left once plays a sound at random, pressing 2-6 times will play quickMusic1-5. 
#define quickMusic1     33
#define quickMusic2     34
#define quickMusic3     35
#define quickMusic4     36
#define quickMusic5     38

#endif

#include "Arduino.h"
#include <EEPROMex.h>  

#ifndef SHADOW 
#include <Wire.h>
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;
#endif

#ifdef SHADOW
#include "Adafruit_Soundboard.h"  
Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial3, NULL, SFX_RST);

#include <adk.h>
#include <USB.h>
#include <usbhub.h>
#include <PS3BT.h>    // https://github.com/felis/USB_Host_Shield_2.0
#include <avr/wdt.h>
#include <SPP.h>

USB Usb; // define USB and PS3BT information for Arduino
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the class to support 2 controllers */
PS3BT *PS3Nav = new PS3BT(&Btd);  // First Controller
PS3BT *PS3Nav2 = new PS3BT(&Btd); // Second Controller

SPP SerialBT(&Btd,"BB8","1234"); // Create a BT Serial device(defaults: "Arduino" and the pin to "0000" if not set)
String PS3MoveNavigatonPrimaryMAC = "00:06:F5:64:60:3E"; //If using multiple controlers, designate a primary
//String PS3MoveNavigatonPrimaryMAC = "00:06:F5:0E:6D:67"; //Use this for storage of a 2nd primary controller (keep commented)
boolean isPS3NavigatonInitialized = false;
boolean isSecondaryPS3NavigatonInitialized = false;

boolean WaitingforReconnect = false;
boolean WaitingforReconnectDome = false;

boolean mainControllerConnected = false;
boolean domeControllerConnected = false;

//Used for PS3 Fault Detection
uint32_t Nav1msgLagTime = 0;
uint32_t lastMsgNav1Time = 0;
uint32_t currentNav1Time = 0;
uint32_t Nav2msgLagTime = 0;
uint32_t lastMsgNav2Time = 0;
uint32_t currentNav2Time = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;
int remoteBatt = 0;
bool firstMessage = true;
unsigned long currentMillis;
unsigned long bodyCalibrationMillis;
long previousMillis;
long previousMillisD;
#endif

#ifdef dobogusinclude // Satisfy the IDE, which needs to see the include statment in the ino too.
  #include <spi4teensy3.h>
  #include <SPI.h>
#endif

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NETWORKID       100  // The same on all nodes that talk to each other
#define REMOTE_ADDRESS  1    // The unique identifier of this node
#define BODY_ADDRESS    2    // The recipient of packets
#define DOME_ADDRESS    3    // The recipient of packets
#define DRIVE_ADDRESS   4

//Match frequency to the hardware version of the radio on your Feather
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   115200

// for Feather 32u4 Radio
#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4  // Pin 7 is IRQ 4!
#define RFM69_RST     4

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);





#ifdef serialSound
  struct SEND_DATA_STRUCTURE{
      int Joy1Y=256; //main drive Primary joystick up/down = Joy1Y
      int Joy1X=256; //S2S tilt/steer Primary joystick left/right = Joy1X
      int Joy2Y=256; //head tilt Secondary joystick up/down = Joy2Y
      int Joy2X=256; //head tilt MK3 only (head spin Mk2) Secondary joystick left/right = Joy2X
      int Joy3X=256; //spin Flywheel Primary joystick R2 + left/right = Joy3X
      int Joy4X=256; //head spin MK3 only Secondary joystick R2 left/right 
      byte ServoMode=1; //Select on Secondary joystick with R1
      byte lBut1; //Primary 1
      byte lBut2; //Primary 2
      byte lBut3;   //Primary 3
#ifdef SHADOW
      byte lBut4;   //Primary 4
      byte lButCross;   //Primary Cross
      byte lButCircle;  //Primary Circle
#endif
      byte Fwd; //Select on right joystick = rJoySelect
      byte Speed;
      byte rBut2; //Secondary 2
      byte rBut3; //Secondary 3
#ifdef SHADOW
      byte rBut1;   //Secondary 1
      byte rBut4;   //Secondary 4
      byte rButCross;   //Secondary Cross
      byte rButCircle;  //Secondary Circle
#endif
      byte motorEnable=1; //toggle on top
      byte CalibID; 
      byte wireless;    
  
  }sendToBody;
#endif

#ifdef SHADOW
struct SEND_DATA_STRUCTURE{
      int Joy1Y=256; //main drive Primary joystick up/down = Joy1Y
      int Joy1X=256; //S2S tilt/steer Primary joystick left/right = Joy1X
      int Joy2Y=256; //head tilt Secondary joystick up/down = Joy2Y
      int Joy2X=256; //head tilt MK3 only (head spin Mk2) Secondary joystick left/right = Joy2X
      int Joy3X=256; //spin Flywheel Primary joystick R2 + left/right = Joy3X
      int Joy4X=256; //head spin MK3 only Secondary joystick R2 left/right 
      byte ServoMode; //Select on Secondary joystick with R1
      byte lBut1=1; //Primary 1
      byte lBut2=1; //Primary 2
      byte lBut3;   //Primary 3
      byte lBut4;   //Primary 4
      byte lButCross;   //Primary Cross
      byte lButCircle;  //Primary Circle
      byte Fwd; //Select on right joystick = rJoySelect
      byte Speed;
      byte rBut2=1; //Secondary 2
      byte rBut3=1; //Secondary 3
      byte rBut1;   //Secondary 1
      byte rBut4;   //Secondary 4
      byte rButCross;   //Secondary Cross
      byte rButCircle;  //Secondary Circle
      byte motorEnable=1; //toggle on top
      byte CalibID; 
      byte wireless;    
}sendToBody;

// =======================================================================================
// Remote to Dome
// =======================================================================================

struct SEND_DATA_STRUCTURE_REMOTE{
      int PSI;
      byte lBut3;
      float bodyBatt;
      
}sendTo;
#endif



byte packet[sizeof(sendToBody)];
typedef struct RECEIVE_DATA_STRUCTURE_DOME{
      
      float bodyBatt;
      float domeBatt;
      
}recDomeData;
recDomeData recFromDome;

typedef struct RECEIVE_DATA_STRUCTURE_BODY{

      int psi=0;
      byte button4 = 1;
      float bodyBatt;
      
}recBodyData;
recBodyData recFromBody;

bool SEND;
byte psiState, quitState;
int musicState, voiceNum, musicNum;
byte readPinState = 1; 
unsigned long soundMillis;
int ch1a; //main drive
int ch2a; //tilt / steer
int ch3a; //head tilt
int ch4a; //head spin
int ch5a; //flywheel
int ch6a; //tbd
int Joy1X, Joy1Xa, Joy1Xb;
int Joy1Y, Joy1Ya, Joy1Yb;
int Joy2X, Joy2Xa, Joy2Xb;
int Joy2Y, Joy2Ya, Joy2Yb;
int Joy3X, Joy3Xa, Joy3Xb;
int Joy4X, Joy4Xa, Joy4Xb;
int Joy1XCenter, Joy1YCenter, Joy2XCenter, Joy2YCenter, Joy3XCenter, Joy4XCenter;
int Joy1XLow=512, Joy1YLow=512, Joy2XLow=512, Joy2YLow=512, Joy3XLow=512, Joy4XLow=512;
int Joy1XHigh=512, Joy1YHigh=512, Joy2XHigh=512, Joy2YHigh=512, Joy3XHigh=512, Joy4XHigh=512;
int joyconfigX1, joyconfigY1, joyconfigX2, joyconfigY2, joyconfigX3, joyconfigX4;
byte enable, bodyConfigStep, ServoMode, lBut1State, rBut1State;
byte rJoySelect, domeSend;
byte JoySelectState, ServoJoySelectState, dome180, DomeDirection;
byte menuConfirm, Display, cursorMove, updateScreen = 1;
byte casenum, lastcasenum = 1;
byte start, wireless, bodyWireless;
byte joyConfStep = 1;
byte But1State, But2State, But1Sound, voiceSend, But2Sound, musicSend;
int joyConfCountdown;
byte waitTime = 5;
String menuItems[] = {"Reverse Dome", "Dome Config", "Body Config", "Joystick Config"};

#ifdef SHADOW
byte playSound, soundState;
#endif

String speedDisplay[] = {"Slow", "Medium", "Fast"};
//int rectime[20];
//byte rectimeloc;
const int eepromSet = 890;
int rBut1 = 1;
float measuredvbat;
unsigned long buttonTimer, menuTimeout, joyConfMillis, lastScreenUpdate, domeConfigTimeout; 
unsigned long bodyConfigMillis, domeConnectivityMillis, lastrecdata = 1000; 
unsigned long lastReading, lastLoopMillis, lastrecDataMillis, lastSend;
unsigned long But1Millis, But2Millis;

byte lJoySelect, lBut1=1, lBut2=1, lBut3, lBut4, lButCross, lButCircle, rBut4, rButCross, rButCircle, Fwd; 
byte Speed, rBut2, rBut3=1, motorEnable=1, lBut1Timer, lBut2Timer, rBut3Timer;
unsigned long lBut1Millis, lBut2Millis, rBut3Millis;

byte startup = 1;
  
void setup() {
#ifndef SHADOW
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.set400kHz(); 
  oled.setFont(Callibri15);
  oled.println(F("==========================="));
  oled.println(F("            Joe's Drive       "));
  oled.println(F("==========================="));
  Serial.begin(115200);

  pinMode (lJoySelectPIN, INPUT_PULLUP); 
  pinMode (rJoySelectPIN, INPUT_PULLUP); 
  pinMode (lBut1PIN, INPUT_PULLUP); 
  pinMode (lBut2PIN, INPUT_PULLUP); 
  pinMode (lBut3PIN, INPUT_PULLUP); 
  pinMode (rBut1PIN, INPUT_PULLUP); 
  pinMode (rBut2PIN, INPUT_PULLUP);
  pinMode (enablePIN, INPUT_PULLUP);
#else
  pinMode(SFX_ACT, INPUT_PULLUP); // read stat of Act on Soundboard
  digitalWrite(reset_Pin, LOW);
  pinMode(reset_Pin, OUTPUT);
  delay(10);
  pinMode(reset_Pin, INPUT);
  delay(1000);

  Serial.begin(115200); 
  Serial2.begin(115200);
  Serial3.begin(9600);
  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  if (Usb.Init() == -1) {
    #ifdef DEBUG
      Serial.print(F("OSC did not start - check compiler settings\n"));
    #endif
    while (1); //halt
  }
  #ifdef DEBUG
    Serial.print(F("\r\nSHADOW Bluetooth Library Started\n"));
  #endif
  
  //Setup for PS3 / PS4 NAV Controllers
  PS3Nav->attachOnInit(onInitPS3); // onInit() is called upon a new connection - you can call the function whatever you like
  PS3Nav2->attachOnInit(onInitPS3Nav2);

//================================== SFX Sound Configuration ====================================
/* =========================================================================
Change the FX Sound board into UAT mode for easier control
* ========================================================================= */
/* =========================================================================
/* https://learn.adafruit.com/adafruit-audio-fx-sound-board
/* https://cdn-learn.adafruit.com/downloads/pdf/adafruit-audio-fx-sound-board.pdf
Connect UG to GND (to start the sound board in Uart mode) 
Connect RX to the data-output pin from the microcontroller into the sound board 
Connect TX to the data-output pin to the microcontroller from the sound board 
Connect RST to another microcontroller pin, when toggled low, 
it will reset the sound board into a known state
If you want to know when sound is being played, 
the ACT pin is LOW when audio is playing - this output also controls the red ACT LED
* ========================================================================= */

if (!sfx.reset()) {
  #ifdef DEBUG
    Serial.println("SFX Module Not found");
  #endif
  while (1);
}
  #ifdef DEBUG
    Serial.println("SFX board found and configured in Uart mode");
  #endif

#endif


  // Hard Reset the RFM module
pinMode(RFM69_RST, OUTPUT);
digitalWrite(RFM69_RST, HIGH);
delay(100);
digitalWrite(RFM69_RST, LOW);
delay(100);

// Initialize radio
radio.initialize(FREQUENCY,REMOTE_ADDRESS,NETWORKID);
if (IS_RFM69HCW) {
  radio.setHighPower();    // Only for RFM69HCW & HW!
}
radio.setPowerLevel(20); // power output ranges from 0 (5dBm) to 31 (20dBm)


  if(EEPROM.readInt(36) == eepromSet){
    Joy1XCenter = EEPROM.readInt(0);
    Joy1YCenter = EEPROM.readInt(2);
    Joy2XCenter = EEPROM.readInt(4);
    Joy2YCenter = EEPROM.readInt(6);
    Joy3XCenter = EEPROM.readInt(8);
    Joy4XCenter = EEPROM.readInt(10);
    Joy1XLow = EEPROM.readInt(12);
    Joy1XHigh = EEPROM.readInt(14);
    Joy1YLow = EEPROM.readInt(16);
    Joy1YHigh = EEPROM.readInt(18);
    Joy2XLow = EEPROM.readInt(20);
    Joy2XHigh = EEPROM.readInt(22);
    Joy2YLow = EEPROM.readInt(24);
    Joy2YHigh = EEPROM.readInt(26);
    Joy3XLow = EEPROM.readInt(28);
    Joy3XHigh = EEPROM.readInt(30);
    Joy4XLow = EEPROM.readInt(32);
    Joy4XHigh = EEPROM.readInt(34);
  }else{
    Joy1XCenter = 512;
    Joy1YCenter = 512;
    Joy2XCenter = 512;
    Joy2YCenter = 512;
    Joy3XCenter = 512;
    Joy4XCenter = 512;
    Joy1XLow = 0;
    Joy1XHigh = 1023;
    Joy1YLow = 0;
    Joy1YHigh = 1023;
    Joy2XLow = 0;
    Joy2XHigh = 1023;
    Joy2YLow = 0;
    Joy2YHigh = 1023;
    Joy3XLow = 0;
    Joy3XHigh = 1023;
    Joy4XLow = 0;
    Joy4XHigh = 1023;
  }
  
  delay(2000);
  #ifndef SHADOW
  oled.clear();
  #endif
}
  
void loop() {
  
  recData();
  
  if(millis() - lastLoopMillis >= 20){
    lastLoopMillis = millis(); 
    #ifndef SHADOW
    readInputs();
    #else
    GetMoveController();
    #endif
    setspeed();
    measureVoltage();
    #ifndef SHADOW
    Screen();
    #else
    sounds();
    #endif
    timeout();
  }
  
  if(millis() - lastSend >= sendDelay || SEND){
    SendData();
  }
}
  
