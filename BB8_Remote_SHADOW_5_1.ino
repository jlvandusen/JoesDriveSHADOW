// =======================================================================================
//            SHADOWBB8 :  Small Handheld Arduino Droid Operating Wand for BB8
// =======================================================================================
//                          Last Revised Date: 02/25/2017
//                             Written By: jlvandusen
//                        Inspired by KnightShade
//     learn more at http://jimmyzsbb8.blogspot.com or http://jimmyzsr2.blogspot.com
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it .
//         This program is distributed in the hope that it will be useful,
//         but WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// =======================================================================================
//   Note: You will need a Arduino UNO to run this sketch,
//
//   This is written to be a UNIVERSAL Sketch - supporting multiple controller options
//   for the Joe Latiola BB8 build version 1.. in replacement of his custom controller
//   see: www.facebook.com/joesdrive
//   learn more at http://jimmyzsbb8.blogspot.com or http://jimmyzsr2.blogspot.com
//
//      - Single PS3 Move Navigation
//      - Pair of PS3 Move Navigation controllers (recommended)
//      - Bluetooth connected Phone (Limited Controls)
//      - Disco Droid Support (Audio sound support)
//      - JLV: Please do not use Digital pin 7 as input or output because is used in the comunication with MAX3421E
//      - JLV: Digital: 7 (RST), 50 (MISO), 51 (MOSI), 52 (SCK).
//      - JLV: https://www.arduino.cc/en/Main/ArduinoBoardMegaADK
//
// =======================================================================================

// =======================================================================================
//                          Debug Code settings
// =======================================================================================

// Make sure you set the Primary Controller (if you set the mac address - you will no longer need to pair the controller)

//#define SHADOW_DEBUG_JOY
//#define SHADOW_DEBUG_BT
#define SHADOW_SendData // enabled or disabled sending data over serial0 across to mega.  Should be UNCOMMENTED by default
// =======================================================================================
//                          Libraries
// =======================================================================================
// Playstation 3 move controller support derrived from the USB Shield Library
#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
  #include <spi4teensy3.h>
  #include <SPI.h>
#endif

#include <EEPROMex.h>   // https://github.com/thijse/Arduino-EEPROMEx
#include "Arduino.h"

// define USB and PS3BT information for Arduino
USB Usb;
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the class to support 2 controllers */
PS3BT *PS3Nav = new PS3BT(&Btd);
PS3BT *PS3Nav2 = new PS3BT(&Btd);

String PS3MoveNavigatonPrimaryMAC = "00:06:F5:64:60:3E"; //If using multiple controlers, designate a primary
boolean isPS3NavigatonInitialized = false;
boolean isSecondaryPS3NavigatonInitialized = false;

//Used for PS3 Fault Detection
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;

// Configure the channels for mapping to the body over serial
int ch1; //main drive [Right Move Controller UP\DOWN]
int ch2; //tilt / steer [Right Move Controller LEFT\RIGHT]
int ch3; //head tilt [left Move Controller UP\DOWN]
int ch4; //head spin [left Move Controller LEFT\RIGHT]
int ch1a; //main drive
int ch2a; //tilt / steer
int ch3a; //head tilt
int ch4a; //head spin
int ch1b; //main drive
int ch2b; //tilt / steer
int ch3b; //head tilt
int ch4b; //head spin
int but1; //Select on left joystick
int but2; //left 1
int but3; //left 2
int but4; //left3
int but5; //Select on right joystick
int but6; //right 1
int but6Speed = 1;
int but6State = 0;
int but6SpeedLast = 5;
int but7; //right 2
int but8; //right 3
int motorEnable = 1; // Move Controller CROSS (x)

  
int ch1Center;
int ch2Center;
int ch3Center;
int ch4Center;

int state = 0;
int stateLast = 0;

long previousMillis;
long interval = 40;
long previousMillisScreen;
unsigned long bodyCalibrationMillis;

float bodyBatt = 0.0;
float domeBatt = 0.0;
float remoteBatt = 0.0;
int bodyStatus;
int driveSpeed = 55;
int lastDriveSpeed = 55;
int lastBodyStatus = 0;
int calibrationMarker;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];   

boolean newData = false;

long but8Millis;
int but8State;

unsigned long currentMillis = millis();

void setup() {
  Serial.begin(115200);
  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nSHADOW Bluetooth Library Started"));

  //Setup for PS3
  PS3Nav->attachOnInit(onInitPS3); // onInit() is called upon a new connection - you can call the function whatever you like
  PS3Nav2->attachOnInit(onInitPS3Nav2);

  ch1Center = EEPROM.readInt(0);
  ch2Center = EEPROM.readInt(4);
  ch3Center = EEPROM.readInt(8);
  ch4Center = EEPROM.readInt(12);

  if(ch1Center < 100){
    ch1Center = 127;  // 512
  }
  if(ch2Center < 100){
    ch2Center = 127;
  }
  if(ch3Center < 100){
    ch3Center = 127;
  }
  if(ch4Center < 100){
    ch4Center = 127;
  }
  
}

// =======================================================================================
//                          readUSB Usb.Task() commands
// =======================================================================================
//  The more devices we have connected to the USB or BlueTooth, the more often 
//  Usb.Task need to be called to eliminate latency.

boolean readUSB()
{
  //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  Usb.Task();
  if (PS3Nav->PS3NavigationConnected ) Usb.Task();
  if (PS3Nav2->PS3NavigationConnected ) Usb.Task();
  if (criticalFaultDetect())
  {
    return false; // We have a fault condition that we want to ensure that we do NOT process any controller data!!!
  }

  if (criticalFaultDetectNav2())
  {
    return false; // We have a fault condition that we want to ensure that we do NOT process any controller data!!!
  }
  return true;
}


void loop() {
  Usb.Task();
  GetMoveController();
  #ifdef SHADOW_SendData
    senddata();
  #endif
  
}

void GetMoveController() {
  currentMillis = millis();
  readUSB();
  if (PS3Nav->PS3Connected || PS3Nav->PS3NavigationConnected) {
    if (PS3Nav->getAnalogHat(LeftHatX) > 137 || PS3Nav->getAnalogHat(LeftHatX) < 117 || PS3Nav->getAnalogHat(LeftHatY) > 137 || PS3Nav->getAnalogHat(LeftHatY) < 117 || PS3Nav2->getAnalogHat(LeftHatX) > 137 || PS3Nav2->getAnalogHat(LeftHatX) < 117 || PS3Nav2->getAnalogHat(LeftHatY) > 137 || PS3Nav2->getAnalogHat(LeftHatY) < 117 || PS3Nav->getAnalogHat(RightHatX) > 137 || PS3Nav->getAnalogHat(RightHatX) < 117 || PS3Nav->getAnalogHat(RightHatY) > 137 || PS3Nav->getAnalogHat(RightHatY) < 117) {
        ch1a = (PS3Nav->getAnalogHat(LeftHatY));
        ch2a = (PS3Nav->getAnalogHat(LeftHatX));
        ch1 = map(ch1a, 0, 255, 512, 0);
        ch2 = map(ch2a, 0, 255, 512, 0);
        if (PS3Nav2->PS3Connected || PS3Nav2->PS3NavigationConnected){
          ch3a = (PS3Nav2->getAnalogHat(LeftHatY));
          ch4a = (PS3Nav2->getAnalogHat(LeftHatX));
          ch3 = map(ch3a, 0, 255, 512, 0);
          ch4 = map(ch4a, 0, 255, 512, 0);
          #ifdef SHADOW_DEBUG_JOY
            Serial.print(F("\r\nNav2HatX: "));
            Serial.print(ch4a);
            Serial.print(F("\tNav2HatY: "));
            Serial.print(ch3a);
          #endif
          }
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav1HatX: "));
        Serial.print(ch2a);
        Serial.print(F("\tNav1HatY: "));
        Serial.print(ch1a);
      #endif
    } 
    else {
      ch1 = 256;
      ch2 = 256;
      ch3 = 256;
      ch4 = 256;
    }
//
//    if (ch1b == ch1Center){
//      ch1 = 256;
//    }else if (ch1b > ch1Center){
//      ch1 = map(ch1b, ch1Center, 255, 255, 0);
//    }else if (ch1b < ch1Center){
//      ch1 = map(ch1b, 0, ch1Center, 512, 257);
//    }
//    
//    if (ch2b == ch2Center){
//      ch2 = 256;
//    }else if (ch2b > ch2Center){
//      ch2 = map(ch2b, ch2Center, 1024, 255, 0);
//    }else if (ch2b < ch2Center){
//      ch2 = map(ch2b, 0, ch2Center, 512, 257);
//    }
//    
//    if (ch3b == ch3Center){
//      ch3 = 256;
//    }else if (ch3b > ch3Center){
//      ch3 = map(ch3b, ch3Center, 1024, 255, 0);
//    }else if (ch3b < ch3Center){
//      ch3 = map(ch3b, 0, ch3Center, 512, 257);
//    }
//    
//    if (ch4b == ch4Center){
//      ch4 = 256;
//    }else if (ch4b > ch4Center){
//      ch4 = map(ch4b, ch4Center, 1024, 255, 0);
//    }else if (ch4b < ch4Center){
//      ch4 = map(ch4b, 0, ch4Center, 512, 257);
//    }


// Analog button values can be read and used to set speed of the drive 1,2 or 3.
    if (PS3Nav->getAnalogButton(L2) || PS3Nav->getAnalogButton(R2)) {
      if (PS3Nav->getAnalogButton(L2)>180){
        but6Speed=3;
      }
      if ((PS3Nav->getAnalogButton(L2)>50) && (PS3Nav->getAnalogButton(L2)<180)){
        but6Speed=2;
      }
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nL2: "));
        Serial.print(PS3Nav->getAnalogButton(L2));
        Serial.print(F("\r\nSpeed: "));
        Serial.print(but6Speed);
      #endif
    } else but6Speed=1;

// enable motor controllers via L1 + CIRCLE combinations from either controller
    if (PS3Nav->getAnalogButton(L1) && PS3Nav->getButtonClick(CIRCLE)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nR1 + Circle:"));
        Serial.print(motorEnable);
      #endif
    }
    if (PS3Nav2->getAnalogButton(L1) && PS3Nav2->getButtonClick(CIRCLE)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nL1 + Circle:"));
        Serial.print(motorEnable);
      #endif
    }

// Force disconnect the Move navigation controller using L2 + PS buttons combo.
    if (PS3Nav->getAnalogButton(L2) && PS3Nav->getButtonClick(PS)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nDisconnecting Nav1 SHADOW Controller"));
      #endif
      PS3Nav->disconnect();
      PS3Nav->disconnect();
    }
    if (PS3Nav2->getAnalogButton(L2) && PS3Nav2->getButtonClick(PS)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nDisconnecting Nav2 SHADOW Controller"));
      #endif
      PS3Nav2->disconnect();
      PS3Nav2->disconnect();
    }
    if (PS3Nav->getButtonClick(PS) ) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav1 PS"));
      #endif
    }
    if (PS3Nav2->getButtonClick(PS) ) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav2 PS"));
      #endif
    }
    if (PS3Nav->getButtonClick(CIRCLE)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav1 Circle"));
      #endif
    }
    if (PS3Nav2->getButtonClick(CIRCLE)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav2 Circle"));
      #endif
    }
    if (PS3Nav->getButtonClick(CROSS)) {
      if (motorEnable == 1) motorEnable = 0;
      else motorEnable = 1;
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav1 Cross"));
        Serial.print(motorEnable);
      #endif
    }
    if (PS3Nav2->getButtonClick(CROSS)) {
      if (motorEnable == 1) motorEnable = 0;
      else motorEnable = 1;
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav2 Cross"));
        Serial.print(motorEnable);
      #endif
    }

// Button 1 Assignment UP
    if (PS3Nav->getButtonClick(UP)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav1 UP (BUT1)"));
      #endif
      but1=0;
    } else but1=1;
    
    if (PS3Nav->getButtonClick(RIGHT)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav1 RIGHT (BUT2)"));
      #endif
      but2=0;
    } else but2=1;
    
    if (PS3Nav->getButtonClick(DOWN)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav1 DOWN (BUT3)"));
      #endif
      but3=0;
    } else but3=1;
    
    if (PS3Nav->getButtonClick(LEFT)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav1 LEFT (BUT4)"));
      #endif
      but4=0;
    } else but4=1;

// Button 5 Assignment UP
    if (PS3Nav2->getButtonClick(UP)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav2 UP (BUT5)"));
      #endif
      but5=0;
    } else but5=1;
    
    if (PS3Nav2->getButtonClick(RIGHT)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav2 RIGHT (BUT6)"));
      #endif
      but6=0;
    } else but6=1;
    
    if (PS3Nav2->getButtonClick(DOWN)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav2 DOWN (BUT7)"));
      #endif
      but7=0;
    } else but7=1;
    
    if (PS3Nav2->getButtonClick(LEFT)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nNav2 LEFT (BUT8)"));
      #endif
      but8=0;
    } else but8=1;

    if (PS3Nav->getButtonClick(L1))
    #ifdef SHADOW_DEBUG_JOY
      Serial.print(F("\r\nL1"));
    #endif
    if (PS3Nav->getButtonClick(L3)) {
        #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nRight Joy Push"));
        #endif
    } 

    if (PS3Nav2->getButtonClick(L3)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nLeft Joy Push"));
      #endif
      but3=0;
    } else but3=1;
    if (PS3Nav->getButtonClick(R1))
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nR1"));
      #endif
    if (PS3Nav->getButtonClick(R3))
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nR3"));
      #endif
    if (PS3Nav->getButtonClick(SELECT)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nSelect - "));
      #endif
      PS3Nav->printStatusString();
    }
    if (PS3Nav->getButtonClick(START)) {
      #ifdef SHADOW_DEBUG_JOY
        Serial.print(F("\r\nStart"));
      #endif
    }
  }
}

void senddata() {
  if ((PS3Nav->PS3NavigationConnected &&  !PS3Nav2->PS3NavigationConnected) || (!PS3Nav->PS3NavigationConnected &&  !PS3Nav2->PS3NavigationConnected) || (!PS3Nav->PS3NavigationConnected &&  PS3Nav2->PS3NavigationConnected)){
    return;
  }
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis; 
    Serial.print (F("<"));
    Serial.print (ch1);
    Serial.print (F(","));
    Serial.print (ch2);
    Serial.print (F(","));
    Serial.print (ch3);
    Serial.print (F(","));
    Serial.print (ch4);
    Serial.print (F(","));
    Serial.print (but1);
    Serial.print (F(","));
    Serial.print (but2);
    Serial.print (F(","));
    Serial.print (but3);
    Serial.print (F(","));
    Serial.print (but4);
    Serial.print (F(","));
    Serial.print (but5);
    Serial.print (F(","));
    Serial.print (but6Speed);
    Serial.print (F(","));
    Serial.print (but7);
    Serial.print (F(","));
    Serial.print (but8);
    Serial.print (F(","));
    Serial.print (motorEnable);
    Serial.println (F(">"));
  }
}

void onInitPS3()
{
  String btAddress = getLastConnectedBtMAC();
  PS3Nav->setLedOn(LED1);
  isPS3NavigatonInitialized = true;
  badPS3Data = 0;
  #ifdef SHADOW_DEBUG_BT
    Serial.print("BT Address of Last connected Device when Primary PS3 Connected: ");
    Serial.print(btAddress);
  #endif
  if (btAddress == PS3MoveNavigatonPrimaryMAC)
  {
    #ifdef SHADOW_DEBUG_BT
      Serial.print("\r\nWe have our primary controller connected.\r\n");
    #endif
  }
  else
  {
    #ifdef SHADOW_DEBUG_BT
      Serial.print("\r\nWe have a controller connected, but it is not designated as \"primary\".\r\n");
    #endif
  }
}

void onInitPS3Nav2()
{
  String btAddress = getLastConnectedBtMAC();
  PS3Nav2->setLedOn(LED1);
  isSecondaryPS3NavigatonInitialized = true;
  badPS3Data = 0;
  if (btAddress == PS3MoveNavigatonPrimaryMAC) swapPS3NavControllers();
  #ifdef SHADOW_DEBUG_BT
    Serial.print("\r\nBT Address of Last connected Device when Secondary PS3 Connected: ");
    Serial.print(btAddress);
  #endif
  if (btAddress == PS3MoveNavigatonPrimaryMAC)
  {
    #ifdef SHADOW_DEBUG_BT
      Serial.print("\r\nWe have our primary controller connecting out of order.  Swapping locations\r\n");
    #endif
  }
  else
  {
    #ifdef SHADOW_DEBUG_BT
      Serial.print("\r\nWe have a secondary controller connected.\r\n");
    #endif
  }
}

String getLastConnectedBtMAC()
{
  String btAddress = "";
  for (int8_t i = 5; i > 0; i--)
  {
    if (Btd.disc_bdaddr[i] < 0x10)
    {
      btAddress += "0";
    }
    btAddress += String(Btd.disc_bdaddr[i], HEX);
    btAddress += (":");
  }
  btAddress += String(Btd.disc_bdaddr[0], HEX);
  btAddress.toUpperCase();
  #ifdef SHADOW_DEBUG_BT
    Serial.print("\r\nLast Connected Device MAC: ");
    Serial.print(btAddress);
  #endif

  return btAddress;

}

void swapPS3NavControllers()   //Correct the status for Initialization
{
  PS3BT* temp = PS3Nav;
  PS3Nav = PS3Nav2;
  PS3Nav2 = temp;
  boolean tempStatus = isPS3NavigatonInitialized;
  isPS3NavigatonInitialized = isSecondaryPS3NavigatonInitialized;
  isSecondaryPS3NavigatonInitialized = tempStatus;
  
  //Must relink the correct onInit calls
  PS3Nav->attachOnInit(onInitPS3);
  PS3Nav2->attachOnInit(onInitPS3Nav2);
}


// =======================================================================================
//                      Process PS3 Controller Fault Detection
// =======================================================================================
boolean criticalFaultDetect()
{
  if (PS3Nav->PS3NavigationConnected || PS3Nav->PS3Connected)
  {
    lastMsgTime = PS3Nav->getLastMessageTime();
    currentTime = millis();
    if ( currentTime >= lastMsgTime)
    {
      msgLagTime = currentTime - lastMsgTime;
    } else
    {
      #ifdef SHADOW_DEBUG_BT
        Serial.println("msgLagTime growing increase badPS3Data");
      #endif
      badPS3Data++;
      msgLagTime = 0;
    }

    if (msgLagTime > 300)
    {
      #ifdef SHADOW_DEBUG_BT
        Serial.println("msgLagTime > 300");
      #endif
      return true;
    }
    if ( msgLagTime > 30000 ) {
      #ifdef SHADOW_DEBUG_BT
        Serial.println("msgLagTime > 30000 disconnecting");
      #endif
      PS3Nav->disconnect();
      msgLagTime = 0;
    }
    //Check PS3 Signal Data
    if (!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged)) {
      delay(10); // We don't have good data from the controller. Wait 10ms, Update USB, and try again
      Usb.Task();
      if (!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged)) {
        badPS3Data++;
        #ifdef SHADOW_DEBUG_BT
          Serial.println("We don't have good data from the controller. Wait 10ms, Update USB, and try again");
        #endif
        return true;
      }
    }
    else if (badPS3Data > 0)
    {

      badPS3Data = 0;
    }
    if ( badPS3Data > 10 )
    {
      #ifdef SHADOW_DEBUG_BT
        Serial.println("badPS3Data > 10");
      #endif
      PS3Nav->disconnect();
      badPS3Data = 0;
    }
  } else lastMsgTime = millis();
  return false;
}

// =======================================================================================
// //////////////////////////Process of PS3 Secondary Controller Fault Detection//////////
// =======================================================================================
boolean criticalFaultDetectNav2() {
  if (PS3Nav2->PS3NavigationConnected || PS3Nav2->PS3Connected) {
    lastMsgTime = PS3Nav2->getLastMessageTime();
    currentTime = millis();

    if ( currentTime >= lastMsgTime) {
      msgLagTime = currentTime - lastMsgTime;
    }
    else {
      #ifdef SHADOW_DEBUG_BT
        Serial.println("msgLagTime growing increase badPS3Data");
      #endif
      badPS3Data++;
      msgLagTime = 0;
    }

    if ( msgLagTime > 10000 ){
      #ifdef SHADOW_DEBUG_BT
        Serial.println("msgLagTime > 10000");
      #endif
      PS3Nav2->disconnect();
      badPS3Data = 0;
      msgLagTime = 0;
      return true;
    }

    //Check PS3 Signal Data
    if (!PS3Nav2->getStatus(Plugged) && !PS3Nav2->getStatus(Unplugged))
    {
      #ifdef SHADOW_DEBUG_BT
        Serial.println("We don't have good data from the controller.");
        Serial.println("Wait 15ms, Update USB, and try again");
      #endif
      //  We don't have good data from the controller.
      //  Wait 15ms, Update USB, and try again
      delay(15);
      Usb.Task();
      if (!PS3Nav2->getStatus(Plugged) && !PS3Nav2->getStatus(Unplugged))
      {
        badPS3Data++;
        #ifdef SHADOW_DEBUG_BT
          Serial.println("BadPS3Data Increased");
        #endif
        return true;
      }
    }
    else if (badPS3Data > 0)
    {
      badPS3Data = 0;
    }

    if ( badPS3Data > 10 )
    {
#ifdef SHADOW_DEBUG

#endif
      PS3Nav2->disconnect();
      badPS3Data = 0;
      return true;
    }
  } else lastMsgTime = millis();
  return false;
}
// =======================================================================================
//                        Set Joystick Centers (calibration)
// =======================================================================================

void setJoystickCenter(){
    ch1Center = ch1a;
    ch2Center = ch2a;
    ch3Center = ch3a;
    ch4Center = ch4a;
    EEPROM.writeInt(0,ch1Center);
    EEPROM.writeInt(4,ch2Center);
    EEPROM.writeInt(8,ch3Center);
    EEPROM.writeInt(12,ch4Center);
    delay(1000);
}

