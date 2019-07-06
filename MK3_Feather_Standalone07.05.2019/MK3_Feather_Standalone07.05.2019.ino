// =======================================================================================
//  Joe's Drive, StandaloneFeather:SHADOW :  Small Handheld Arduino Droid Operating Wand for BB8
// =======================================================================================
//                          Last Revised Date: 07/05/2019
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
//  Modified Low PowerLab: https://travis-ci.org/LowPowerLab/RFM69
//  Radio Head RFM69 library: https://github.com/adafruit/RadioHead/archive/master.zip
//    Find file: RFM69.h
//    Find: #define RF69_CSMA_LIMIT_MS 1000
//    Change to: #define RF69_CSMA_LIMIT_MS 100
//  EasyTransfer: https://github.com/madsci1016/Arduino-EasyTransfer
//
//  Add the Adafruit AVR boards to your IDE by adding
//  https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
//  To the Preferences > Additional Boards Managers URLs line in your settings
// =======================================================================================
// =======================================================================================
// Configuration Options
// =======================================================================================
// Update these as necessary to match your setup
#define SHADOW                // If defined - disables various functions to allow communication to SHADOW controller
#define dataDelay 5
#define sendDelay 45


#include <SPI.h>
#include <RFM69.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <RH_Serial.h>
#include "Arduino.h"
#include <EasyTransfer.h>

#ifndef SHADOW
  EasyTransfer SendRemote;
#endif
  EasyTransfer SendBody;

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NETWORKID       100  // The same on all nodes that talk to each other
#define REMOTE_ADDRESS  76    // The unique identifier of this node
#define BODY_ADDRESS    45    // The recipient of packets
#define DOME_ADDRESS    90    // The recipient of packets
#define DRIVE_ADDRESS   76

//Match frequency to the hardware version of the radio on your Feather
//If using RFM69HCW https://www.adafruit.com/product/3077 then set to RF69_433MHZ otherwise RF69_915MHZ
#define FREQUENCY     RF69_433MHZ
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   115200

// for Feather 32u4 Radio
#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4  // Pin 7 is IRQ 4!
#define RFM69_RST     4




RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

RH_Serial driver(Serial1);
RHReliableDatagram manager(driver, BODY_ADDRESS);


    uint8_t remfrom;
    uint8_t rembuf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t rembuflen = sizeof(rembuf);
    uint8_t bodyfrom;
    
unsigned long lastLoop, lastRec, lastSent;

typedef struct RECEIVE_DATA_STRUCTURE_REMOTE{
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
#ifdef SHADOW
      byte lBut4;   //Primary 4
      byte lButCross;   //Primary Cross
      byte lButCircle;  //Primary Circle
#endif
      byte Fwd; //Select on right joystick = rJoySelect
      byte Speed;
      byte rBut2;   //Secondary 2
      byte rBut3=1; //Secondary 3
#ifdef SHADOW
      byte rBut1;   //Secondary 1
      byte rBut4;   //Secondary 4
      byte rButCross;   //Secondary Cross
      byte rButCircle;  //Secondary Circle
#endif
      byte motorEnable=1; //toggle on top
      byte CalibID;
      byte wireless=1;
      
}recRemoteData; 
recRemoteData recFromRemote;

byte send_buf[sizeof(recFromRemote)];
bool Send;   


typedef struct RECEIVE_DATA_STRUCTURE_DRIVE{
      int PSI;
      byte ledStatus;
      float bodyBatt;
      //float domeBatt;
      
}recBodyData;
recBodyData recFromBody;

uint8_t bodybuf[sizeof(recFromBody)];
uint8_t bodybuflen = sizeof(recFromBody);

typedef struct RECEIVE_DATA_STRUCTURE_DOME{

      float domeBatt;
      
}recDomeData;
recDomeData recFromDome;

uint8_t from;
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t buflen = sizeof(buf);





void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial1.begin(57600);
driver.serial().begin(57600);
  if (!manager.init())
  Serial.println("init failed");

// Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY,BODY_ADDRESS,NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(20); // power output ranges from 0 (5dBm) to 31 (20dBm)
#ifndef SHADOW
SendRemote.begin(details(recFromRemote), &Serial1);
#endif
SendBody.begin(details(recFromBody), &Serial1);

  

}

void loop() {
  if(millis() - lastLoop >= 2){
    lastLoop = millis();
    #ifndef SHADOW
    recRemote();
    #endif
  } 
   
  if(millis() - lastSent >= 80){
    recBody();
  }



}  

  
  
  

