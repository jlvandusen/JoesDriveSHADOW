// NOTE: you can also use: getButtonPress, getAnalogButton, getButtonClick
// Calling BB8_SoundControl(XX) with a number value - you can review which file on z_SndControls.ino will play selected snd byte
// if you wish to call randoms... those are stored in 99-105

void getMoveButtons(){
  if (PS3Nav2->PS3NavigationConnected) {
    if (PS3Nav2->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNAV2 L2:"));
      #endif
    }
    if (PS3Nav2->getAnalogButton(L2) && PS3Nav2->getButtonClick(PS)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nDisconnecting Nav2 SHADOW Controller"));
      #endif
      #ifdef SFX_SERIAL
        BB8_SoundControl(21); // Serial.print("Played PT02NEXT0OGG\n");
      #endif
      #ifdef DEBUG_SFX
        Serial.print("Played PT02NEXT0OGG\n");
      #endif
      #ifdef DEBUG
        bool PS3status;
        if (PS3Nav2->getStatus(Full)) remoteBatt = 100;
        else if (PS3Nav2->getStatus(High)) remoteBatt = 70;
        else if (PS3Nav2->getStatus(Low)) remoteBatt = 40;
        else if (PS3Nav2->getStatus(Dying)) remoteBatt = 10;
        Serial.println(" ");
        Serial.print("Battery for Nav2 is at ");
        Serial.print(remoteBatt);
        Serial.println("%");
      #endif
      PS3Nav2->disconnect();
    }
  }


// =======================================================================================
//                      L2 Speed Control
// =======================================================================================


  if (PS3Nav->PS3NavigationConnected) {
    if (PS3Nav->getAnalogButton(L2)) { // Analog button values can be read and used to set speed of the drive 1,2 or 3.
      if (PS3Nav->getAnalogButton(L2)>180){
        Speed=3;
      }
      if ((PS3Nav->getAnalogButton(L2)>50) && (PS3Nav->getAnalogButton(L2)<180)){
        Speed=2;
      }
      #ifdef DEBUG
        Serial.print(F("\r\nL2: "));
        Serial.print(PS3Nav->getAnalogButton(L2));
        Serial.print(F("\r\nSpeed: "));
        Serial.print(Speed);
      #endif
    } else Speed=1;

// =======================================================================================
//                      L2 + PS Disconnect the controller(s)
// =======================================================================================

    // Force disconnect the Move navigation controller using L2 + PS buttons combo.
    if (PS3Nav->getAnalogButton(L2) && PS3Nav->getButtonClick(PS)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nDisconnecting Nav1 SHADOW Controller"));
      #endif
      #ifdef SFX_SERIAL
      BB8_SoundControl(21); // Serial.print("Played PT02NEXT0OGG\n");
      #endif
      #ifdef DEBUG_SFX
        Serial.print("Played PT02NEXT0OGG\n");
      #endif
      #ifdef DEBUG
        bool PS3status;
        if (PS3Nav->getStatus(Full)) remoteBatt = 100;
        else if (PS3Nav->getStatus(High)) remoteBatt = 70;
        else if (PS3Nav->getStatus(Low)) remoteBatt = 40;
        else if (PS3Nav->getStatus(Dying)) remoteBatt = 10;
        Serial.println(" ");
        Serial.print("Battery for Nav1 is at ");
        Serial.print(remoteBatt);
        Serial.println("%");
      #endif
      PS3Nav->disconnect();

    }
  
  }

  if (PS3Nav->PS3NavigationConnected && PS3Nav2->PS3NavigationConnected) {
  
    if (PS3Nav->getButtonPress(PS) ) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav1 PS"));
      #endif
    }
    if (PS3Nav2->getButtonPress(PS) ) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav2 PS"));
      #endif
    }
    
    if (PS3Nav2->getButtonClick(CIRCLE) && PS3Nav2->getAnalogButton(L2)) {
      if (ServoMode == 1) ServoMode = 0;
      else ServoMode = 1;
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav2 Circle & L2: "));
        Serial.println(ServoMode);
      #endif
      #ifdef SFX_SERIAL
        BB8_SoundControl(1);
      #endif
    }
    
    if (PS3Nav->getButtonClick(CROSS)) {
      if (motorEnable == 1) motorEnable = 0;
      else motorEnable = 1;
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav1 Cross (motorEnable):"));
        Serial.println(motorEnable);
      #endif
    }
    
    if (PS3Nav2->getButtonClick(CROSS)) {
      if (motorEnable == 1) motorEnable = 0;
      else motorEnable = 1;
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav2 Cross (motorEnable):"));
        Serial.println(motorEnable);
      #endif
    }
    
    // Button 1 Assignment UP 
    if (PS3Nav->getButtonPress(UP) && !PS3Nav2->getAnalogButton(L1)&& !PS3Nav2->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav1 UP (BUT1)"));
      #endif
      lBut1=0;
    } else lBut1=1;
    
    if (PS3Nav->getButtonClick(RIGHT) && !PS3Nav2->getAnalogButton(L1)&& !PS3Nav2->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav1 RIGHT (lBut2)"));
      #endif
      lBut2=0;
      #ifdef SFX_SERIAL
        BB8_SoundControl(44);
      #endif
      #ifdef DEBUG_SFX
        Serial.print("Played PT04NEXT3OGG\n");
      #endif
    } else lBut2=1;
    
    if (PS3Nav->getButtonClick(DOWN) && !PS3Nav2->getAnalogButton(L1)&& !PS3Nav2->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav1 DOWN (lBut3)"));
      #endif
      lBut3=0;
      #ifdef SFX_SERIAL
        Serial1.print("q\n");
        Serial1.print("PT03NEXT3OGG\n");
      #endif
      #ifdef DEBUG_SFX
        Serial.print("q\n");
        Serial.print("PT03NEXT3OGG\n");
      #endif
    } else lBut3=1;
    
    if (PS3Nav->getButtonClick(LEFT) && !PS3Nav2->getAnalogButton(L1)&& !PS3Nav2->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav1 LEFT (lBut4)"));
      #endif
      lBut4=0;
      #ifdef SFX_SERIAL
      Serial1.print("q\n");
      Serial1.print("PT03NEXT3OGG\n");
      #endif
    } else lBut4=1;
    
    if (PS3Nav2->getButtonPress(UP) && !PS3Nav->getAnalogButton(L1)&& !PS3Nav->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav2 UP (rBut1) and Volume UP"));
      #endif
      rBut1=0;
    } else rBut1=1;
    
    if (PS3Nav2->getButtonClick(RIGHT) && !PS3Nav->getAnalogButton(L1)&& !PS3Nav->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav2 RIGHT (rBut2)"));
      #endif
      rBut2=0;
      #ifdef SFX_SERIAL
        BB8_SoundControl(1);
      #endif
      #ifdef DEBUG_SFX
      Serial.print("Played Random Sound Location 1\n");
      #endif
    } else rBut2=1;
    
    if (PS3Nav2->getButtonClick(DOWN) && !PS3Nav->getAnalogButton(L1)&& !PS3Nav->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav2 DOWN (BUT7)"));
      #endif
      rBut3=0;
      #ifdef SFX_SERIAL
        BB8_SoundControl(1);
      #endif
      #ifdef DEBUG_SFX
        Serial.println(F("Played Random Sound location 1"));
      #endif
    } else rBut3=1;

    
    if (PS3Nav2->getButtonPress(LEFT) && !PS3Nav->getAnalogButton(L1)&& !PS3Nav->getAnalogButton(L2)) {
      #ifdef DEBUG_NAV
        Serial.print(F("\r\nNav2 LEFT (BUT8)"));
      #endif
      rBut4=0;
      #ifdef SFX_SERIAL
        BB8_SoundControl(28); // Serial1.print("PT02NEXT7OGG\n");
      #endif
    } else rBut4=1;

    
    if (PS3Nav2->getButtonPress(L1) && PS3Nav->getButtonClick(UP)) {
      #ifdef DEBUG_SFX
        Serial.println(F("\r\nPlayed PT05NEXT1OGG\n"));
      #endif
      #ifdef SFX_SERIAL
        BB8_SoundControl(52); // Serial1.print("PT05NEXT1OGG\n");
      #endif
    }
    if (PS3Nav2->getButtonPress(L1) && PS3Nav->getButtonClick(RIGHT)) {
      #ifdef DEBUG_SFX
        Serial.println(F("\r\nPlayed PT05NEXT2OGG\n"));
      #endif
      #ifdef SFX_SERIAL
        BB8_SoundControl(53); // Serial1.print("PT05NEXT2OGG\n");
      #endif
    }
    if (PS3Nav2->getButtonPress(L1) && PS3Nav->getButtonPress(DOWN)) {
      #ifdef DEBUG_SFX
        Serial.print(F("\r\nNav2 L1 + Nav DOWN"));
      #endif
      #ifdef SFX_SERIAL
        BB8_SoundControl(54); // Serial1.print("PT05NEXT2OGG\n");
      #endif
    }
    if (PS3Nav2->getButtonPress(L1) && PS3Nav->getButtonPress(LEFT)) {
      #ifdef DEBUG_SFX
        Serial.print(F("\r\nNav2 L1 + Nav LEFT"));
      #endif
      #ifdef SFX_SERIAL
        BB8_SoundControl(55); // Serial1.print("PT05NEXT2OGG\n");
      #endif
    }
  }
}
