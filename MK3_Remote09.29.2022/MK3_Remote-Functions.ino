// =======================================================================================
//                          readUSB Usb.Task() commands
// =======================================================================================
//  The more devices we have connected to the USB or BlueTooth, the more often 
//  Usb.Task need to be called to eliminate latency.

boolean readUSB() {
  //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  Usb.Task();
  if (PS3Nav->PS3NavigationConnected ) Usb.Task();
  if (PS3Nav2->PS3NavigationConnected ) Usb.Task();
  if (criticalFaultDetect())
  {
    //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
    return false;
  }

  if (criticalFaultDetectNav2())
  {
    //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
    return false;
  }
  return true;
}


//==================  Get Nav Controller Data from Bluetooth  ====================================
// The Function checks for any values > 140 or < 114 which allows zeroing before reading 
// anything of 20 (joystick dead zone)
// Second Nav Controller (lefthand)
// Flywheel = ch5
// DomeSpinServo = ch4servo
// DomeSpin = ch4
// Joystickdome = ch3
// Primary Nav Controller (righthand)
// Swing S2S = ch2
// Joystickdrive = ch1
//================================================================================================

void GetMoveController() {
  currentMillis = millis();
  readUSB();
  if (PS3Nav->PS3NavigationConnected && PS3Nav2->PS3NavigationConnected) {
    if (PS3Nav->getAnalogHat(LeftHatX) > 140 || PS3Nav->getAnalogHat(LeftHatX) < 114 || PS3Nav->getAnalogHat(LeftHatY) > 140 || PS3Nav->getAnalogHat(LeftHatY) < 114 || PS3Nav2->getAnalogHat(LeftHatX) > 140 || PS3Nav2->getAnalogHat(LeftHatX) < 114 || PS3Nav2->getAnalogHat(LeftHatY) > 140 || PS3Nav2->getAnalogHat(LeftHatY) < 114) {
        ch1a = (PS3Nav->getAnalogHat(LeftHatY));
        if (PS3Nav->getAnalogHat(LeftHatX) && !PS3Nav->getAnalogButton(L1)) {  // If NAV2 L1 is not pressed S2S
          ch2a = (PS3Nav->getAnalogHat(LeftHatX));
          ch5a = 127;
        }
        else if (PS3Nav->getAnalogButton(L1)){ // else If NAV2 L1 is pressed Flywheel
          ch5a = (PS3Nav->getAnalogHat(LeftHatX));
          ch2a = 127;
          if(Fwd == 0) {
          #ifndef reverseFlywheel
            Joy3X = map(ch5a, 0, 255, 512, 0);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #else
            Joy3X = map(ch5a, 0, 255, 0, 512);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #endif
          } else {
          #ifndef reverseFlywheel
            Joy3X = map(ch5a, 0, 255, 0, 512);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #else
            Joy3X = map(ch5a, 0, 255, 512, 0);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #endif
          }
        }
        if(Fwd == 0) {
          #ifndef reverseDrive
            Joy1Y = map(ch1a, 0, 255, 512, 0);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #else
            Joy1Y = map(ch1a, 0, 255, 0, 512);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #endif
          #ifndef reverseS2S
            Joy1X = map(ch2a, 0, 255, 512, 0);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #else
            Joy1X = map(ch2a, 0, 255, 0, 512);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #endif
        } else {
          #ifndef reverseDrive
            Joy1Y = map(ch1a, 0, 255, 0, 512);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #else
            Joy1Y = map(ch1a, 0, 255, 512, 0);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #endif
          #ifndef reverseS2S
            Joy1X = map(ch2a, 0, 255, 0, 512);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #else
            Joy1X = map(ch2a, 0, 255, 512, 0);    // May need to reverse this (0,512) depending upon which way the POTs is set with Negative up or down. S2S
          #endif
        }
        if (PS3Nav2->PS3NavigationConnected){
          ch3a = (PS3Nav2->getAnalogHat(LeftHatY));
          ch4a = (PS3Nav2->getAnalogHat(LeftHatX));
          if(Fwd == 0) {
            #ifndef reverseDomeTilt
              Joy2Y = map(ch3a, 0, 255, 0,512);   // May need to reverse this (512,0) depending upon which way the POTs is set with Negative up or down. DomeTilt
            #else
              Joy2Y = map(ch3a, 0, 255, 512,0);   // May need to reverse this (512,0) depending upon which way the POTs is set with Negative up or down. DomeTilt
            #endif
          } else {
            #ifndef reverseDomeTilt
              Joy2Y = map(ch3a, 0, 255, 512,0);   // May need to reverse this (512,0) depending upon which way the POTs is set with Negative up or down. DomeTilt
            #else
              Joy2Y = map(ch3a, 0, 255, 0,512);   // May need to reverse this (512,0) depending upon which way the POTs is set with Negative up or down. DomeTilt
            #endif
          }
          #ifndef reverseDomeSpin
            Joy2X = map(ch4a, 0, 255, 512, 0);  // May need to reverse (0,512) depending upon which way you connect the power on motors Dome spin
          #else
            Joy2X = map(ch4a, 0, 255, 0, 512);  // May need to reverse (0,512) depending upon which way you connect the power on motors Dome spin
          #endif

          #ifdef DEBUG_NAV
            Serial.print(F("\r\nNav2HatX: "));
            Serial.print(ch4a);
            Serial.print(F("\tNav2HatY: "));
            Serial.print(ch3a);
          #endif
        }
        else {
          Joy2Y = 256;  // send center values
          Joy2X = 256;  // send center values
        }
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav1HatX: "));
        Serial.print(ch2a);
        Serial.print(F("\tNav1HatY: "));
        Serial.print(ch1a);
      #endif
    } 
    else {
      Joy1Y = 256;  // send center values
      Joy1X = 256;  // send center values
      Joy2Y = 256;  // send center values
      Joy2X = 256;  // send center values
      Joy3X = 256;  // send center values
    }

// FUTURE: Battery checks for the MOVE controllers to report in power levels
//  if (PS3Nav->getStatus(Full)) remoteBatt = 100;
//  else if (PS3Nav->getStatus(High)) remoteBatt = 70;
//  else if (PS3Nav->getStatus(Low)) remoteBatt = 40;
//  else if (PS3Nav->getStatus(Dying)) remoteBatt = 10;
  
  }
  else {
    Joy1Y=256;
    Joy1X=256;
    Joy2Y=256;
    Joy2X=256;
    Joy3X=256;
    Joy4X=256;
    ServoMode=1;
    lBut1=1;
    lBut2=1;
    lBut3=1;
    lBut4=1;
    lButCross=1;
    lButCircle=1;
    rBut1=1;
    rBut2=1;
    rBut3=1;
    rBut4=1;
    rButCross=1;
    rButCircle=1;
    motorEnable=1;
  }
}

void onInitPS3() {
  String btAddress = getLastConnectedBtMAC();
  PS3Nav->setLedOn(LED1);
  isPS3NavigatonInitialized = true;
  badPS3Data = 0;
  #ifdef DEBUG
    Serial.print("\r\nBT Address of Last connected Device when Primary PS3 Connected: ");
    Serial.print(btAddress);
  #endif
  if (btAddress == PS3MoveNavigatonPrimaryMAC) {
      #ifdef DEBUG
        Serial.print("\r\nWe have our primary controller connected.\r\n");
        bool PS3status;
        if (PS3Nav->getStatus(Full)) remoteBatt = 100;
        else if (PS3Nav->getStatus(High)) remoteBatt = 70;
        else if (PS3Nav->getStatus(Low)) remoteBatt = 40;
        else if (PS3Nav->getStatus(Dying)) remoteBatt = 10;
        Serial.println(" ");
        Serial.print("Battery for \"primary\" is at ");
        Serial.print(remoteBatt);
        Serial.println("%");
      #endif
    } else {
      #ifdef DEBUG
      Serial.print("\r\nWe have a controller connected, but it is not designated as \"primary\".\r\n");
      Serial.print("\r\nWe have our primary controller connected.\r\n");
        bool PS3status;
        if (PS3Nav->getStatus(Full)) remoteBatt = 100;
        else if (PS3Nav->getStatus(High)) remoteBatt = 70;
        else if (PS3Nav->getStatus(Low)) remoteBatt = 40;
        else if (PS3Nav->getStatus(Dying)) remoteBatt = 10;
        Serial.println(" ");
        Serial.print("Battery for Non-\"primary\" is at ");
        Serial.print(remoteBatt);
        Serial.println("%");
      #endif
    }
}

void onInitPS3Nav2() {
  String btAddress = getLastConnectedBtMAC();
  PS3Nav2->setLedOn(LED1);
  isSecondaryPS3NavigatonInitialized = true;
  badPS3Data = 0;
  #ifdef DEBUG
    Serial.print("\r\nBT Address of Last connected Device when Secondary PS3 Connected: ");
    Serial.print(btAddress);
  #endif
  if (btAddress == PS3MoveNavigatonPrimaryMAC) {
    #ifdef DEBUG
      Serial.print("\r\nWe have our primary controller connecting out of order.  Swapping locations\r\n");
      bool PS3status;
      if (PS3Nav->getStatus(Full)) remoteBatt = 100;
      else if (PS3Nav->getStatus(High)) remoteBatt = 70;
      else if (PS3Nav->getStatus(Low)) remoteBatt = 40;
      else if (PS3Nav->getStatus(Dying)) remoteBatt = 10;
      Serial.println(" ");
      Serial.print("Battery for \"primary\" is at ");
      Serial.print(remoteBatt);
      Serial.println("%");
    #endif
    swapPS3NavControllers();
  } else {
    #ifdef DEBUG
      Serial.print("\r\nWe have a secondary controller connected.\r\n");
      bool PS3status;
      if (PS3Nav2->getStatus(Full)) remoteBatt = 100;
      else if (PS3Nav2->getStatus(High)) remoteBatt = 70;
      else if (PS3Nav2->getStatus(Low)) remoteBatt = 40;
      else if (PS3Nav2->getStatus(Dying)) remoteBatt = 10;
      Serial.println(" ");
      Serial.print("Battery for \"secondary\" is at ");
      Serial.print(remoteBatt);
      Serial.println("%");
    #endif
  }
}

void GetLocalAddress() {
   String localAddress = "";
  for (int8_t i = 5; i > 0; i--)
  {
    if (Btd.my_bdaddr[i] < 0x10)
    {
      localAddress += "0";
    }
    localAddress += String(Btd.my_bdaddr[i], HEX);
    localAddress += (":");
  }
  localAddress += String(Btd.my_bdaddr[0], HEX);
  localAddress.toUpperCase();
  if (localAddress !="00:00:00:00:00:0"){
    Serial.print(F("My Current Bluetooth Dongle Address is: "));
    Serial.println(localAddress);
  }
}

String getLastConnectedBtMAC() {
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
  Serial.print(F("Last Connected Address is: "));
  Serial.println(btAddress);
  return btAddress;
}

void swapPS3NavControllers() {
  PS3BT* temp = PS3Nav;
  #ifdef DEBUG
    Serial.println("Swapping Nav1 with Nav2: ");
  #endif
  PS3Nav = PS3Nav2;
  PS3Nav2 = temp;
  #ifdef DEBUG
    Serial.println("Now Initializing: ");
  #endif
  //Correct the status for Initialization
  boolean tempStatus = isPS3NavigatonInitialized;
  isPS3NavigatonInitialized = isSecondaryPS3NavigatonInitialized;
  isSecondaryPS3NavigatonInitialized = tempStatus;
  //Must relink the correct onInit calls
  PS3Nav->attachOnInit(onInitPS3);
  PS3Nav2->attachOnInit(onInitPS3Nav2);
  #ifdef DEBUG
    Serial.println("Re-attaching ");
  #endif
}

// =======================================================================================
//                      Process PS3 Controller Fault Detection
// =======================================================================================
boolean criticalFaultDetect(){
  if (PS3Nav->PS3NavigationConnected || PS3Nav->PS3Connected)
  {
    lastMsgNav1Time = PS3Nav->getLastMessageTime();
    currentNav1Time = millis();
    if ( currentNav1Time >= lastMsgNav1Time)
    {
      Nav1msgLagTime = currentNav1Time - lastMsgNav1Time;
    } else
    {
      #ifdef DEBUG
        Serial.println("NAV1: msgLagTime growing increase badPS3Data");
      #endif
      badPS3Data++;
      Nav1msgLagTime = 0;
    }

    if (Nav1msgLagTime > 300)
    {
      #ifdef DEBUG
        Serial.println("NAV1: msgLagTime > 300");
      #endif
      return true;
    }
    if ( Nav1msgLagTime > 30000 ) {
      #ifdef DEBUG
        Serial.println("NAV1: msgLagTime > 30000 disconnecting");
      #endif
      PS3Nav->disconnect();
      Nav1msgLagTime = 0;
    }
    //Check PS3 Signal Data
    if (!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged)) {
      delay(10); // We don't have good data from the controller. Wait 10ms, Update USB, and try again
      Usb.Task();
      if (!PS3Nav->getStatus(Plugged) && !PS3Nav->getStatus(Unplugged)) {
        badPS3Data++;
        #ifdef DEBUG
          Serial.println("NAV1: We don't have good data from the controller. Wait 10ms, Update USB, and try again");
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
      #ifdef DEBUG
        Serial.println("NAV1: badPS3Data > 10 Disconnecting");
      #endif
      PS3Nav->disconnect();
      badPS3Data = 0;
    }
  } else lastMsgNav1Time = millis();
  return false;
}

// =======================================================================================
// //////////////////////////Process of PS3 Secondary Controller Fault Detection//////////
// =======================================================================================
boolean criticalFaultDetectNav2() {
  if (PS3Nav2->PS3NavigationConnected || PS3Nav2->PS3Connected) {
    lastMsgNav2Time = PS3Nav2->getLastMessageTime();
    currentNav2Time = millis();

    if ( currentNav2Time >= lastMsgNav2Time) Nav2msgLagTime = currentNav2Time - lastMsgNav2Time;
    else {
      #ifdef DEBUG
        Serial.println("NAV2: msgLagTime growing increase badPS3Data");
      #endif
      badPS3Data++;
      Nav2msgLagTime = 0;
    }

    if ( Nav2msgLagTime > 30000 ){
      #ifdef DEBUG
        Serial.println("NAV2: Nav2msgLagTime > 30000");
      #endif
//      PS3Nav2->disconnect();
      badPS3Data = 0;
      Nav2msgLagTime = 0;
      return true;
    }

    //Check PS3 Signal Data
    if (!PS3Nav2->getStatus(Plugged) && !PS3Nav2->getStatus(Unplugged)) {
      #ifdef DEBUG
        Serial.println("NAV2: We don't have good data from the controller.");
        Serial.println("Wait 15ms, Update USB, and try again");
      #endif
      //  We don't have good data from the controller.
      //  Wait 15ms, Update USB, and try again
      delay(15);
      Usb.Task();
      if (!PS3Nav2->getStatus(Plugged) && !PS3Nav2->getStatus(Unplugged)) {
        badPS3Data++;
        #ifdef DEBUG
          Serial.println("NAV2: BadPS3Data Increased");
        #endif
        return true;
      }
    }
    else if (badPS3Data > 0) badPS3Data = 0;

    if ( badPS3Data > 10 ) {
      #ifdef DEBUG
          Serial.println("NAV2: BadPS3Data > 10 disconnecting");
      #endif
      PS3Nav2->disconnect();
      badPS3Data = 0;
      return true;
    }
  } else lastMsgNav2Time = millis();
  return false;
}

 
void setspeed(){ 
  
  if(Display <2){
    if(rBut1 == 0 && rBut1State == 0){
      rBut1State = 1;
    }else if(rBut1 == 1 && rBut1State == 1){
      rBut1State = 0;
      if(Speed == 0){
        Speed = 1;
      }else if(Speed == 1){
        Speed = 2;
      }else if(Speed == 2){
        Speed = 0;
      }
      updateScreen = 1;
    }   
  }
}
  
void measureVoltage(){

  if((millis() - lastReading >= 10000 && analogRead(battPin) > 1) || startup == 1){
    lastReading = millis();
    measuredvbat = analogRead(battPin);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
      if(startup == 1){
        startup = 0;
      }
  }
}
  
void readInputs(){
#ifdef serialSound

  lBut1 = digitalRead(lBut1PIN);
  lBut2 = digitalRead(lBut2PIN);
  rBut3 = digitalRead(lBut3PIN);
  
  if(analogRead(rBut3PIN) > 1){
    rBut3 = 1;
  }else{
    rBut3 = 0;
  }
  
        
  if((lBut1 == 0 || rBut3 == 0) && But1State == 0){
    But1State = 1;
    But1Millis = millis();
    
  }else if(lBut1 == 1 && rBut3 == 1 && But1State == 1){
    But1State = 0;
    But1Sound ++;
    
      if(But1Sound > 6){
        But1Sound -= 6;
        
      }
  }
  if(But1Sound > 0 && millis() - But1Millis >= 800){
    voiceSend = 0;
    But1Sound = 0;
    sendToBody.lBut1 = But1Sound;
    
  }else if(But1Sound > 0 && millis() - But1Millis >= 400){
    sendToBody.lBut1 = But1Sound;
  }  
  
    
  if(lBut2 == 0 && But2State == 0){
    But2State = 1;
    But2Millis = millis();
  }else if(lBut2 == 1 && But2State == 1){
    But2State = 0;
    But2Sound ++;
    
      if(But2Sound > 6){
        But2Sound -= 6;
        
      }
  }
  
  if(But2Sound > 0 && millis() - But2Millis >= 800){
    musicSend = 0;
    But2Sound = 0;
    sendToBody.lBut2 = But2Sound;
    
  }else if(But2Sound > 0 && millis() - But2Millis >= 400){
    sendToBody.lBut2 = But2Sound;
  }
  
  if(sendToBody.rBut3 != 0){
    sendToBody.rBut3 = 0;
  }

 #else
 #ifndef SHADOW
  lBut1Timer = digitalRead(lBut1PIN);
  lBut2Timer = digitalRead(lBut2PIN);
  
  
  if(analogRead(rBut3PIN) > 1){
    rBut3Timer = 1;
  }else{
    rBut3Timer = 0;
  }


  if(lBut1Timer == 0 && lBut1 == 1){
    lBut1 = 0;
    lBut1Millis = millis();
  }else if(lBut1Timer == 1 && lBut1 == 0){
    if(millis() - lBut1Millis >= 300){
      lBut1 = 1;
    }
  }

  if(lBut2Timer == 0 && lBut2 == 1){
    lBut2 = 0;
    lBut2Millis = millis();
  }else if(lBut2Timer == 1 && lBut2 == 0){
    if(millis() - lBut2Millis >= 300){
      lBut2 = 1;
    }
  }

  if(rBut3Timer == 0 && rBut3 == 1){
    rBut3 = 0;
    rBut3Millis = millis();
  }else if(rBut3Timer == 1 && rBut3 == 0){
    if(millis() - rBut3Millis >= 300){
      rBut3 = 1;
    }
  }

#else
  lBut1 = digitalRead(lBut1PIN);
  lBut2 = digitalRead(lBut2PIN);
  rBut3 = digitalRead(lBut3PIN);
  
  if(analogRead(rBut3PIN) > 1){
    rBut3 = 1;
  }else{
    rBut3 = 0;
  }
  
        
  if((lBut1 == 0 || rBut3 == 0) && But1State == 0){
    But1State = 1;
    But1Millis = millis();
    
  }else if(lBut1 == 1 && rBut3 == 1 && But1State == 1){
    But1State = 0;
    But1Sound ++;
    
      if(But1Sound > 6){
        But1Sound -= 6;
        
      }
  }
  if(But1Sound > 0 && millis() - But1Millis >= 800){
    voiceSend = 0;
    But1Sound = 0;
    sendToBody.lBut1 = But1Sound;
    
  }else if(But1Sound > 0 && millis() - But1Millis >= 400){
    sendToBody.lBut1 = But1Sound;
  }  
  
    
  if(lBut2 == 0 && But2State == 0){
    But2State = 1;
    But2Millis = millis();
  }else if(lBut2 == 1 && But2State == 1){
    But2State = 0;
    But2Sound ++;
    
      if(But2Sound > 6){
        But2Sound -= 6;
        
      }
  }
  
  if(But2Sound > 0 && millis() - But2Millis >= 800){
    musicSend = 0;
    But2Sound = 0;
    sendToBody.lBut2 = But2Sound;
    
  }else if(But2Sound > 0 && millis() - But2Millis >= 400){
    sendToBody.lBut2 = But2Sound;
  }
  
  if(sendToBody.rBut3 != 0){
    sendToBody.rBut3 = 0;
  }
#endif
#endif

lBut3 = digitalRead(lBut3PIN);
rBut1 = digitalRead(rBut1PIN);
rBut2 = digitalRead(rBut2PIN);

Joy1Xa = analogRead(Joy1XPIN);
Joy1Ya = analogRead(Joy1YPIN);
Joy2Xa = analogRead(Joy2XPIN);
Joy2Ya = analogRead(Joy2YPIN);
Joy3Xa = analogRead(Joy3XPIN);
Joy4Xa = analogRead(Joy4XPIN);
lJoySelect = digitalRead(lJoySelectPIN);
rJoySelect = digitalRead(rJoySelectPIN);
motorEnable = digitalRead(enablePIN);
centerChannels();
setServoMode();


}
  
void centerChannels(){
     
  if (Joy1Xa == Joy1XCenter){
    Joy1Xb = 256;
  }else if (Joy1Xa > Joy1XCenter){
    Joy1Xb = map(Joy1Xa, Joy1XCenter, Joy1XHigh, 255, 0);
  }else if (Joy1Xa < Joy1XCenter){
    Joy1Xb = map(Joy1Xa, Joy1XLow, Joy1XCenter, 512, 257);
  }
  
  if (Joy1Ya == Joy1YCenter){
    Joy1Yb = 256;
  }else if (Joy1Ya > Joy1YCenter){
    Joy1Yb = map(Joy1Ya, Joy1YCenter, Joy1YHigh, 255, 0);
  }else if (Joy1Ya < Joy1YCenter){
    Joy1Yb = map(Joy1Ya, Joy1YLow, Joy1YCenter, 512, 257);
  }

  if (Joy2Xa == Joy2XCenter){
    Joy2Xb = 256;
  }else if (Joy2Xa > Joy2XCenter){
    Joy2Xb = map(Joy2Xa, Joy2XCenter, Joy2XHigh, 255, 0);
  }else if (Joy2Xa < Joy2XCenter){
    Joy2Xb = map(Joy2Xa, Joy2XLow, Joy2XCenter, 512, 257);
  }

  if (Joy2Ya == Joy2YCenter){
    Joy2Yb = 256;
  }else if (Joy2Ya > Joy2YCenter){
    Joy2Yb = map(Joy2Ya, Joy2YCenter, Joy2YHigh, 255, 0);
  }else if (Joy2Ya < Joy2YCenter){
    Joy2Yb = map(Joy2Ya, Joy2YLow, Joy2YCenter, 512, 257);
  }

  if (Joy3Xa == Joy3XCenter){
    Joy3Xb = 256;
  }else if (Joy3Xa > Joy3XCenter){
    Joy3Xb = map(Joy3Xa, Joy3XCenter, Joy3XHigh, 255, 0);
  }else if (Joy3Xa < Joy3XCenter){
    Joy3Xb = map(Joy3Xa, Joy3XLow, Joy3XCenter, 512, 257);
  }

  if (Joy4Xa == Joy4XCenter){
    Joy4Xb = 256;
  }else if (Joy4Xa > Joy4XCenter){
    Joy4Xb = map(Joy4Xa, Joy4XCenter, Joy4XHigh, 255, 0);
  }else if (Joy4Xa < Joy4XCenter){
    Joy4Xb = map(Joy4Xa, Joy4XLow, Joy4XCenter, 512, 257);
  }

  Joy1X = constrain(Joy1Xb,0,512);
  Joy1Y = constrain(Joy1Yb,0,512);
  Joy2X = constrain(Joy2Xb,0,512);
  Joy2Y = constrain(Joy2Yb,0,512);
  Joy3X = constrain(Joy3Xb,0,512);
  Joy4X = constrain(Joy4Xb,0,512);

  if(Display < 2){
    reverseControls();
  }
}
  
  
void reverseControls(){
  
  if(Display == 0){
    if(rJoySelect == 0 && JoySelectState == 0){
      JoySelectState = 1;
    }else if (rJoySelect == 1 && JoySelectState != 0){
      JoySelectState = 0;
        if(Fwd == 0){
          Fwd = 1;
          updateScreen = 1;
        }else if(Fwd == 1){
          Fwd = 0;
          updateScreen = 1;
        }
    }
  }  
}
  
void setDomeDirection(){

  switch (dome180) {
    case 0:
     if (Fwd == 0){
      DomeDirection = 0;
     }else{
      DomeDirection = 1;
     }
    break;
  
    case 1:
     if (Fwd == 0){
      DomeDirection = 1;
     }else{
      DomeDirection = 0;
     }
    break;
  }
}
  
void SendData(){
  if(millis() > 2000){
    if(millis() - lastSend >= sendDelay){

      #ifndef serialSound
        sendToBody.lBut1 = lBut1; //left 1
        sendToBody.lBut2 = lBut2; //left 2
        sendToBody.rBut3 = rBut3; //right 3
      #endif

      
      sendToBody.Joy1Y = Joy1Y; //main drive
      sendToBody.Joy1X = Joy1X; //tilt / steer
      sendToBody.Joy2Y = Joy2Y; //head tilt
      sendToBody.Joy2X = Joy2X; //head spin
      sendToBody.Joy3X = Joy3X; //spin Flywheel
      sendToBody.Joy4X = Joy4X;
      sendToBody.ServoMode = ServoMode; //Select on left joystick
      sendToBody.lBut3 = lBut3; //left3
      sendToBody.Fwd = Fwd; //Select on right joystick = rJoySelect
      sendToBody.Speed = Speed;
      sendToBody.rBut2 = rBut2; //right 2
      sendToBody.motorEnable = motorEnable; //toggle on top
      sendToBody.wireless = bodyWireless;
      memcpy(packet, &sendToBody, sizeof(sendToBody)); //Copy data from "sendToBody" array to "send_buf" byte array 
      radio.send(BODY_ADDRESS, packet, sizeof(packet)); //target node Id, message as string or byte array, message length
      delay(5); 
      lastSend = millis();
      #ifdef DEBUG
        debugRoutine(); 
      #endif
    }
  }
}
 
void recData(){
  if(millis() - lastrecDataMillis >= recDelay){
    if (radio.receiveDone()) {
      if(radio.SENDERID == uint8_t(DOME_ADDRESS)){ //********** This needs to be fixed for new lib
        if(radio.DATALEN != sizeof(recFromDome)){
          Serial.print("Invalid DOME payload received, not matching Payload struct! Should Be:");
          Serial.print(sizeof(recFromDome)); Serial.print(" received:"); Serial.println(radio.DATALEN);
        }else{
            recFromDome = *(recDomeData*)radio.DATA;
            //Serial.println(millis() - lastrecdata);
            lastrecdata = millis();
            lastrecDataMillis = millis();
            if(bodyWireless == 0){
              bodyWireless = 1;
            }
          }
        SEND = true;
      }else if(radio.SENDERID == uint8_t(BODY_ADDRESS)){ //********** This needs to be fixed for new lib
        if(radio.DATALEN != sizeof(recFromBody)){
          Serial.print("Invalid BODY payload received, not matching Payload struct! Should Be:");
          Serial.print(sizeof(recFromBody)); Serial.print(" received:"); Serial.println(radio.DATALEN);
        }else{
            recFromBody = *(recBodyData*)radio.DATA; 
            recFromDome.bodyBatt = recFromBody.bodyBatt;               
            Serial.println("rec body");
            lastrecdata = millis();
            lastrecDataMillis = millis();
            if(recFromDome.domeBatt != 99.99){
              recFromDome.domeBatt = 99.99;
            }
            if(bodyWireless == 1){
              bodyWireless = 0;
            }
          }
        SEND = true;
      }
    }
  }
}
  
void Screen(){

if(Display < 2){
    infoScreen();
    menuTimer();
}else if(Display == 2){
  if(ServoMode != 0){
    ServoMode = 0;
  }
  menuScreen();
  MenuTimeout();
  enable = 1;
  Joy1Xa = Joy1XCenter;
  Joy1Ya = Joy1YCenter;
  Joy2Xa = Joy2XCenter;
  Joy2Ya = Joy1YCenter;
  Joy3Xa = Joy3XCenter;
  Joy4Xa = Joy4XCenter;
  lJoySelect = 1;
  rJoySelect = 1;
}else if(Display == 3){
  domeReverseScreen();
}else if(Display == 4){
  domeConfig();     
}else if(Display == 5){
  bodyConfig();
}else if(Display == 6){
  joystickConfig();
  enable = 1;
  Joy1Xa = Joy1XCenter;
  Joy1Ya = Joy1YCenter;
  Joy2Xa = Joy2XCenter;
  Joy2Ya = Joy1YCenter;
  Joy3Xa = Joy3XCenter;
  Joy4Xa = Joy4XCenter;
  lJoySelect = 1;
  rJoySelect = 1;
}
}
  
void menuTimer(){
  
  if(rBut2 == 0 && rBut3 == 0 && Display == 0){
    buttonTimer = millis();
    Display = 1;
  }else if(rBut2 == 0 && rBut3 == 0 && Display == 1 && millis() - buttonTimer >= 1500){
    #ifndef SHADOW
    oled.clear();
    #endif
    Display = 2;
    menuTimeout = millis();
  }else if(rBut2 == 1 || rBut3 == 1 && Display == 1){
    Display = 0; 
  }
}
  
void MenuTimeout(){
  
  if(millis() - menuTimeout >= 15000){
    resetMenu();
  }
}
  
void menuScreen(){
  #ifndef SHADOW
  menuCursor();
  if(casenum != lastcasenum){
    lastcasenum = casenum;
    switch (casenum){
      case 0:
        oled.setFont(Callibri15);
        oled.setCursor(0,0);
        oled.print(">");
        oled.setCursor(15,0);
        oled.println(menuItems[0]);
        oled.print("   ");
        oled.setCursor(15,20);
        oled.println(menuItems[1]);
        oled.print("   ");
        oled.setCursor(15,40);
        oled.println(menuItems[2]);
        oled.print("   ");
        oled.setCursor(15,60);
        oled.println(menuItems[3]);
      break;
      case 1:
        oled.setFont(Callibri15);
        oled.setCursor(0,0);
        oled.print("   ");
        oled.setCursor(15,0);
        oled.println(menuItems[0]);
        oled.print(">");
        oled.setCursor(15,20);
        oled.println(menuItems[1]);
        oled.print("   ");
        oled.setCursor(15,40);
        oled.println(menuItems[2]);
        oled.print("   ");
        oled.setCursor(15,60);
        oled.println(menuItems[3]);
      break;
    
      case 2:
        oled.setFont(Callibri15);
        oled.setCursor(0,0);
        oled.print("   ");
        oled.setCursor(15,0);
        oled.println(menuItems[0]);
        oled.print("   ");
        oled.setCursor(15,20);
        oled.println(menuItems[1]);
        oled.print(">");
        oled.setCursor(15,40);
        oled.println(menuItems[2]);
        oled.print("   ");
        oled.setCursor(15,60);
        oled.println(menuItems[3]);
      break;
    
      case 3:
        oled.setFont(Callibri15);
        oled.setCursor(0,0);
        oled.print("   ");
        oled.setCursor(15,0);
        oled.println(menuItems[0]);
        oled.print("   ");
        oled.setCursor(15,20);
        oled.println(menuItems[1]);
        oled.print("   ");
        oled.setCursor(15,40);
        oled.println(menuItems[2]);
        oled.print(">");
        oled.setCursor(15,60);
        oled.println(menuItems[3]);
      break;
    }
  }
  
    if(digitalRead(lJoySelectPIN) == 0 ||digitalRead(rJoySelectPIN) == 0){
      switch(casenum){
        case 0:
          Display = 3;
          subMenuDome180();
            //Do This to reverse the dome
        break;
        
        case 1:
          JoySelectState = 1;
          Display = 4;
          #ifndef SHADOW
          oled.clear();
          oled.println("1. Face dome forward");
          oled.println("2. Press 'select'");
          #endif
          domeConfigTimeout = millis();
            //Do This for Dome Config
        break;
    
        case 2:
          Display = 5;
          #ifndef SHADOW
          oled.clear();
          #endif
            //Do This for Body Config
        break;
        
        case 3:
          Display = 6;
        break;
      }
    }
  #endif
}
   
void menuCursor(){

  if((analogRead(Joy1YPIN) < 200 && cursorMove == 0) || (analogRead(Joy2YPIN) < 200 && cursorMove == 0)){
    menuTimeout = millis();
    casenum++;
    cursorMove = 1;
    if(casenum > 3){
      casenum = 0;
    }
  }else if((analogRead(Joy1YPIN) > 823 || analogRead(Joy2YPIN) > 823) && cursorMove == 0){
    menuTimeout = millis();
    casenum--;
    cursorMove = 1;
    if(casenum < 0){
      casenum = 3;
    }
  }else if(analogRead(Joy1YPIN) < 612 && analogRead(Joy1YPIN) > 412 && analogRead(Joy2YPIN) < 612 && analogRead(Joy2YPIN) > 412){
    cursorMove = 0;
  }
}
  
void infoScreen(){
  #ifndef SHADOW
  if(millis() - lastScreenUpdate >= 10000 || updateScreen == 1){
    updateScreen = 0;
    oled.setCursor(0,0);
    oled.setFont(Callibri15);
    oled.print(speedDisplay[Speed]);oled.print(F("        "));
    oled.setCursor(100,0);
    if(Fwd == 0 || Fwd == 2){
      oled.println(F("Fwd     "));
    }else{
      oled.println(F("Rev     "));
    }
    oled.print(F("Body: ")); 
      if(wireless == 1 && recFromDome.bodyBatt != 99.99){
        oled.print(recFromDome.bodyBatt); oled.println(F("v                 "));
      }else{
        oled.println(F("Disconnected              ")); 
      }
    oled.print(F("Dome: ")); 
      if(wireless == 1 && recFromDome.domeBatt != 99.99){
        oled.print(recFromDome.domeBatt); oled.println("v                 ");
      }else{
        oled.println(F("Disconnected              ")); 
      }
    oled.print("Remote: "); oled.print(measuredvbat); oled.println(F("v               "));
    lastScreenUpdate = millis();
  }
  #endif
}

void subMenuDome180(){
  
  if(dome180 == 0){
    dome180 = 2;
   }else{
    dome180 = 0;
   }
  menuTimeout = millis();
  #ifndef SHADOW
  oled.clear();
  oled.print(F("Dome Reversed!"));
  #endif
}
  
void domeConfig(){
  
  if(JoySelectState == 1 && sendToBody.CalibID == 0){
    if(lJoySelect == 1 && rJoySelect == 1){
      JoySelectState = 0;
    }
  }
  if((lJoySelect == 0 || rJoySelect == 0) && JoySelectState != 1){
    JoySelectState = 1;
    sendToBody.CalibID = 1;
    #ifndef SHADOW
    oled.clear();
    oled.print(F(" Dome Configured! "));
    #endif
    domeConfigTimeout = millis() - 8500 ;
    
  }

  if(millis() - domeConfigTimeout >= 10000){
    JoySelectState = 0;
    resetMenu();
  }
  
}
  
void bodyConfig(){
  
  if(bodyConfigStep == 0){
    #ifndef SHADOW
    oled.print(F(" Body Configuration "));
    #endif
    bodyConfigStep = 1;
    bodyConfigMillis = millis();
    sendToBody.CalibID = 2;
  }else if(bodyConfigStep == 1 && millis() - bodyConfigMillis >= 1500){
    #ifndef SHADOW
    oled.setCursor(0,0);
    oled.println(F(" Adjust dome tilt        "));
    oled.println(F(" until the dome is        "));
    oled.println(F(" centered front to back.        "));
    oled.println(F(" Then press 'Select'        "));
    #endif
    bodyConfigStep = 2;
    
  }else if(bodyConfigStep == 2){
    if((digitalRead(lJoySelectPIN) == 0 || digitalRead(rJoySelectPIN) == 0) && JoySelectState == 0){
      JoySelectState = 1;
    }else if((digitalRead(lJoySelectPIN) == 1 && digitalRead(rJoySelectPIN) == 1) && JoySelectState == 1){
      JoySelectState = 0;
      sendToBody.CalibID = 3;
      bodyConfigStep = 3;
      bodyConfigMillis = millis();
      #ifndef SHADOW
      oled.clear();
      oled.println(F(" Dome tilt offset "));
      oled.println(F(" and pitch offset "));
      oled.println(F(" saved to eeprom "));
      #endif
    }
  }else if(bodyConfigStep == 3 && millis() - bodyConfigMillis >= 2500){
    #ifndef SHADOW
    oled.clear();
    oled.println(F(" Adjust Side to Side "));
    oled.print(F(" until ")); oled.print(F(droidName)); oled.println(F(" is straight."));
    oled.println(F(" Then press 'Select'        "));
    #endif
    bodyConfigStep = 4;
  }else if(bodyConfigStep == 4){
    if((digitalRead(lJoySelectPIN) == 0 || digitalRead(rJoySelectPIN) == 0) && JoySelectState == 0){
      JoySelectState = 1;
    }else if((digitalRead(lJoySelectPIN) == 1 && digitalRead(rJoySelectPIN) == 1) && JoySelectState == 1){
      JoySelectState = 0;
      sendToBody.CalibID = 4;
      bodyConfigStep = 5;
      bodyConfigMillis = millis();
      #ifndef SHADOW
      oled.clear();
      oled.println(F(" Body Calibration ")); oled.println(F(" Complete! "));
      #endif
    }
  }else if(bodyConfigStep == 5 && millis() - bodyConfigMillis >= 2500){
    resetMenu();
  }
}
  
void joystickConfig(){
  
  if(joyConfStep == 1){     
    joyConfMillis = millis();
    joyConfStep = 2;
    #ifndef SHADOW
    oled.clear();
    #endif
  }else if(joyConfStep == 2 && millis() - joyConfMillis >= joyConfCountdown && waitTime > 0 ){;
    #ifndef SHADOW
    oled.setCursor(0,0);
    oled.println(F("Release Joysticks"));
    oled.print(F("Wait: ")); oled.print(waitTime);
    #endif
    joyConfCountdown += 1000;
    waitTime--;
  }else if(joyConfStep == 2 && waitTime == 0 && millis() - joyConfMillis >= joyConfCountdown){
    joyConfStep = 3;
    joyConfCountdown += 2000;
    #ifndef SHADOW
    oled.setCursor(0,0);
    oled.println(F("Storing Center values"));
    oled.print(F("                    "));
    #endif
    setJoystickCenter();
    waitTime = 15;
  }else if(joyConfStep == 3 && millis() - joyConfMillis >= joyConfCountdown && waitTime > 0){
    #ifndef SHADOW
    oled.setCursor(0,0);
    oled.println(F("Continuously rotate      ")); oled.println(F("all joysticks"));
    oled.print(waitTime);oled.print("                  ");
    #endif
    joyConfCountdown += 1000;
    waitTime--;
  }else if(joyConfStep == 3 && waitTime != 0){
    readJoystickHighAndLow();
  }else if(joyConfStep == 3 && waitTime == 0){
    joyConfStep = 4;
    joyConfCountdown += 2500;
    #ifndef SHADOW
    oled.setCursor(0,0);
    oled.println(F("Calibration Complete!              "));
    oled.println(F("                                   "));
    oled.println(F("                                   "));
    #endif
    setJoystickHighAndLow();
  }else if(joyConfStep == 4 && millis() - joyConfMillis >= joyConfCountdown){
    resetMenu();
  }
 
}
  
void resetMenu(){
  #ifndef SHADOW
  oled.clear();
  #endif
  joyConfStep = 1;
  casenum = 0;
  lastcasenum = 1;
  menuConfirm = 0;
  joyConfCountdown = 0;
  waitTime = 5;
  Display = 0;
  sendToBody.CalibID = 0;
  bodyConfigStep = 0;
  
}
  
void readJoystickHighAndLow(){

  if(start == 0){
    start = 1;
    Joy1XLow = 512;
    Joy1XHigh = 512;
    Joy1YLow = 512;
    Joy1YHigh = 512;
    Joy2XLow = 512;
    Joy2XHigh = 512;
    Joy2YLow = 512;
    Joy2YHigh = 512;
    Joy3XLow = 512;
    Joy3XHigh = 512;
    Joy4XLow = 512;
    Joy4XHigh = 512;
  }else if(start == 1){

    constrain(joyconfigX1 = analogRead(Joy1XPIN),0,1023);
    constrain(joyconfigY1 = analogRead(Joy1YPIN),0,1023);
    constrain(joyconfigX2 = analogRead(Joy2XPIN),0,1023);
    constrain(joyconfigY2 = analogRead(Joy2YPIN),0,1023);
    constrain(joyconfigX3 = analogRead(Joy3XPIN),0,1023);
    constrain(joyconfigX4 = analogRead(Joy4XPIN),0,1023);

    if (joyconfigX1 < Joy1XLow){
      Joy1XLow = joyconfigX1;
    }
    if (joyconfigY1 < Joy1YLow){
      Joy1YLow = joyconfigY1;
    }
    if (joyconfigX2 < Joy2XLow){
      Joy2XLow = joyconfigX2;
    }
    if (joyconfigY2 < Joy2YLow){
      Joy2YLow = joyconfigY2;
    }
    if (joyconfigX3 < Joy3XLow){
      Joy3XLow = joyconfigX3;
    }
    if (joyconfigX4 < Joy4XLow){
      Joy4XLow = joyconfigX4;
    }
    if (joyconfigX1 > Joy1XHigh){
      Joy1XHigh = joyconfigX1;
    }
    if (joyconfigY1 > Joy1YHigh){
      Joy1YHigh = joyconfigY1;
    }
    if (joyconfigX2 > Joy2XHigh){
      Joy2XHigh = joyconfigX2;
    }
    if (joyconfigY2 > Joy2YHigh){
      Joy2YHigh = joyconfigY2;
    }
    if (joyconfigX3 > Joy3XHigh){
      Joy3XHigh = joyconfigX3;
    }
    if (joyconfigX4 > Joy4XHigh){
      Joy4XHigh = joyconfigX4;
    }
  }
}
  
 void setJoystickCenter(){
  
  start = 0;
  EEPROM.writeInt(0,analogRead(Joy1XPIN));
  EEPROM.writeInt(2,analogRead(Joy1YPIN));
  EEPROM.writeInt(4,analogRead(Joy2XPIN));
  EEPROM.writeInt(6,analogRead(Joy2YPIN));
  EEPROM.writeInt(8,analogRead(Joy3XPIN));
  EEPROM.writeInt(10,analogRead(Joy4XPIN));
  delay(1000);
}
  
void setJoystickHighAndLow(){

  EEPROM.writeInt(12,Joy1XLow);
  EEPROM.writeInt(14,Joy1XHigh);
  EEPROM.writeInt(16,Joy1YLow);
  EEPROM.writeInt(18,Joy1YHigh);
  EEPROM.writeInt(20,Joy2XLow);
  EEPROM.writeInt(22,Joy2XHigh);
  EEPROM.writeInt(24,Joy2YLow);
  EEPROM.writeInt(26,Joy2YHigh);
  EEPROM.writeInt(28,Joy3XLow);
  EEPROM.writeInt(30,Joy3XHigh);
  EEPROM.writeInt(32,Joy4XLow);
  EEPROM.writeInt(34,Joy4XHigh);
  EEPROM.writeInt(36,eepromSet);
  delay(1000);

}
  
void domeReverseScreen(){
  
  if(millis() - menuTimeout >= 1500){
   resetMenu();
  }
}
  
void timeout(){
  
  if(millis() - lastrecdata >= 2500){
    if(wireless == 1){
      wireless = 0;
      updateScreen = 1;
    }
  }else if(millis() - lastrecdata <= 1000 && wireless == 0){
    wireless = 1;
    updateScreen = 1;
  }

}
  
void setServoMode(){
  
  if(Display == 0){
    if(lJoySelect == 0 && ServoJoySelectState == 0){
      ServoJoySelectState = 1;
    }else if(lJoySelect == 1 && ServoJoySelectState == 1){
      ServoJoySelectState = 0;
      if(ServoMode == 0){
        ServoMode = 1;
      }else{
        ServoMode = 0;
      }
    }
  }
}


/*====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ================================================================================== Sounds and PSI ================================================================================== 
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 */
  
#ifdef SHADOW
void sounds(){
  readPinState = digitalRead(SFX_ACT);
  if((lBut1 == 1 || rBut3 == 1 )&& soundState == 0){   // Press lBut1 or rBut3 and it plays a random speach track. 
    soundState = 1;
    psiState = 1;
    Serial3.print(F("#")); Serial3.println(voiceNum);
    soundMillis = millis(); 
    
  }else if(lBut1 == 0 && rBut3 == 0 && soundState == 1){ // Let go of lBut1 and rBut3 set's it back to "0". 
    soundState = 0;
    
    voiceNum =  random(0,numberOfVoice);
    
  }else if(lBut2 == 1 && soundState == 0){  // Press lBut2 to sequentially play through the "music" tracks
    soundState = 2;
    Serial3.print(F("#")); Serial3.println(numberOfVoice + musicNum);
  }else if(lBut2 == 0 && soundState == 2){   // Let go of lBut2 to set music state back to 0
    soundState = 0;
    musicNum++;
    if(musicNum == numberOfMusic){
      musicNum = 0;
    }
  }else if(lBut1 > 1 || rBut3 > 1 && soundState == 0){   // if lBut1 or Rbut3 is "multi-pressed" it will play the "quickVoice" associated with that number of presses
    psiState = 1;
    soundMillis = millis(); 
    //byte Switch = lBut1; 
    switch (lBut1) {
      case 2:
        Serial3.print(F("#")); Serial3.println(quickVoice1);
      break;

      case 3:
        Serial3.print(F("#")); Serial3.println(quickVoice2);
      break;

      case 4:
        Serial3.print(F("#")); Serial3.println(quickVoice3);
      break;

      case 5:
        Serial3.print(F("#")); Serial3.println(quickVoice4);
      break;

      case 6:
        Serial3.print(F("#")); Serial3.println(quickVoice5);
      break;
    }
  }else if(lBut2 > 1 && soundState == 0){      // if lBut2 is "multi-pressed" it will play the "quickMusic" associated with that number of presses
    
    switch (lBut2) {
      case 2:
        Serial3.print(F("#")); Serial3.println(quickMusic1);
      break;

      case 3:
        Serial3.print(F("#")); Serial3.println(quickMusic2);
      break;

      case 4:
        Serial3.print(F("#")); Serial3.println(quickMusic3);
      break;

      case 5:
        Serial3.print(F("#")); Serial3.println(quickMusic4);
      break;

      case 6:
        Serial3.print(F("#")); Serial3.println(quickMusic5);
      break;
    }
  }

  if(rBut2  == 0 && quitState == 0){
    quitState = 1;
   // rBut2Millis = millis();

  }else if(rBut2 == 1 && quitState == 1){
    quitState = 0;
    if(digitalRead(SFX_ACT) == LOW){
      Serial3.println("q");
      psiState = 0;
      sendTo.PSI = 0;
    }
  }
  psiVal();
}
#endif

void psiVal(){
  
#ifndef disablePSIflash
  if(readPinState == 1 && psiState != 0 && (millis() - soundMillis > 600)){
    sendTo.PSI = 0;
    psiState = 0;
  }else if(psiState == 1){
    sendTo.PSI = constrain(analogRead(fadePin),0,255);
   // Serial.println(analogRead(fadePin));
  }
#endif

#ifdef debugPSI
  Serial.print(F(" readPinState: ")); Serial.print(readPinState);
  Serial.print(F(" psiState: "));Serial.print(psiState);
  Serial.print(F(" fadePin: "));Serial.print(analogRead(fadePin));
  Serial.print(F(" PSI: ")); Serial.println(sendTo.PSI);
#endif
 sendTo.lBut3 = lBut3;
 
}

void debugRoutine(){
      Serial.print(sendToBody.Joy1Y); Serial.print(", ");
      Serial.print(sendToBody.Joy1X); Serial.print(", ");
      Serial.print(sendToBody.Joy2Y); Serial.print(", ");
      Serial.print(sendToBody.Joy2X); Serial.print(", ");
      Serial.print(sendToBody.Joy3X); Serial.print(", ");
      Serial.print(sendToBody.Joy4X); Serial.print(", ");
      Serial.print(sendToBody.ServoMode); Serial.print(", ");
      Serial.print(sendToBody.lBut1); Serial.print(", ");
      Serial.print(sendToBody.lBut2); Serial.print(", ");
      Serial.print(sendToBody.lBut3); Serial.print(", ");
      Serial.print(sendToBody.Fwd); Serial.print(", ");
      Serial.print(sendToBody.Speed); Serial.print(", ");
      Serial.print(sendToBody.rBut2); Serial.print(", ");
      Serial.print(sendToBody.rBut3); Serial.print(", ");
      Serial.print(sendToBody.motorEnable); Serial.print(", ");
      Serial.print(sendToBody.CalibID); Serial.print(", ");
      Serial.println(sendToBody.wireless);
}
