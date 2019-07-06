// =======================================================================================
// Read MPU Status Updates via i2c (Legacy)
// =======================================================================================

void readMPU() {
  #ifdef ShowCurrentOffsets
    Serial.print(" Set PitchOffset = ");
    Serial.print(pitch);
    Serial.print(" Set RollOffset = ");
    Serial.print(roll);  
    Serial.print(" Set YawOffset = ");
    Serial.println(yaw);
  #endif
  
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;
  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);
  pitch = kalPitch;
  roll = kalRoll;
  yawbody = yawbody + (gyr.ZAxis/M_PI);
  yaw = map(yawbody,8000,-8000,180,-180);

}

// =======================================================================================
// If you need to filter your POTs values
// =======================================================================================

void FilterPOTS2S() {
  int sensorValuePOTS2S;
  int sensorValuePOTDLT;
  int sensorValuePOTDSM;
  
  sensorValuePOTS2S = analogRead(S2SpotPin);
  #ifndef MK3
  sensorValuePOTDLT = analogRead(domeTiltPotPin);
  DTL_EMA_S = (EMA_a*sensorValuePOTDLT) + ((1-EMA_a)*DTL_EMA_S);
  #endif
  sensorValuePOTDSM = analogRead(domeSpinPot);
  S2S_EMA_S = (EMA_a*sensorValuePOTS2S) + ((1-EMA_a)*S2S_EMA_S);
  DSM_EMA_S = (EMA_a*sensorValuePOTDSM) + ((1-EMA_a)*DSM_EMA_S);
  Serial.print("S2S Pot: ");
  Serial.print(sensorValuePOTS2S);
  Serial.print(" Filtered: ");
  Serial.println(S2S_EMA_S);
  Serial.print("DomeTilt Pot: ");
  Serial.print(sensorValuePOTDLT);
  Serial.print(" Filtered: ");
  Serial.println(DTL_EMA_S);
  Serial.print("DomeSpin Pot: ");
  Serial.print(sensorValuePOTDSM);
  Serial.print(" Filtered: ");
  Serial.println(DSM_EMA_S);
  delay(50);
}
  
void receiveRemote(){
  if(millis() - lastReceivedMillis >= recDelay){
    //if (manager.recvfrom(buf, &buflen, &from) && buflen == sizeof(recFromRemote)) {
    //  memcpy(&recFromRemote, buf, sizeof(recFromRemote));
    if(Serial1.available() > 0){ //== recFromRemote){
      RecRemote.receiveData();
      lastReceivedMillis=millis();
      SendData = true;       
    }
    remoteTimeout();
  }
}
  
void remoteTimeout(){
  if(millis() - lastReceivedMillis >= 600){
    ControllerStatus = 1;
  }else{
    if(ControllerStatus != 0){
      ControllerStatus = 0;
    }
  }
}
  
void receiveIMUData(){  
  if(Serial2.available() > 0){
    RecIMU.receiveData();
    IMUMillis = millis();
  }
  IMUtimeout();
}
  
void IMUtimeout(){
  if(millis() - IMUMillis >= 500){
    IMUStatus = 1;
  }else{
    if(IMUStatus != 0){
      IMUStatus = 0;
    }
  }
}
  
  
void sendDriveData(){
  if(SendData){  
    SendBody.sendData();
    delay(5);
    SendData = !SendData;
  }
}



void movement() {
  debugRoutines();
  checkCalibID();
  setDriveSpeed();
  domeMovement();

  if (recFromRemote.motorEnable == 0 && ControllerStatus == 0 && IMUStatus == 0){
    autoDisableMotors();
    sideTilt();
    mainDrive();
    flywheelSpin();
    #ifdef MK2
      domeTiltMK2();
    #endif
    #ifdef MK3
      domeTiltMK3();
    #endif
  }else{
    digitalWrite(enablePin, LOW);
    digitalWrite(S2SenablePin, LOW);
    digitalWrite(enablePinDome, LOW);
    autoDisableState = 0;
    autoDisableDoubleCheck = 0;
    turnOffAllTheThings();
  }
} 
  

/*
 **************************************************************************************** Dome *****************************************************************************************
 */
  
  void domeMovement(){
  
    if(recFromRemote.lJoySelect == 0 || recFromRemote.motorEnable == 1){
       domeSpin();
    } else if (recFromRemote.lJoySelect == 1 && ControllerStatus == 0){
      domeSpinServo();
    } 
  }
  
/*
 ************************************************************************************ Main Drive ***********************************************************************************
 */

  
void mainDrive() {
  
#ifdef reverseDrive
  if(recFromRemote.Fwd == 0 || recFromRemote.Fwd == 2){
    joystickDrive = map(recFromRemote.Joy1Y, 0,512, driveSpeed, (driveSpeed * -1));  //Read joystick - change -55/55 to adjust for speed. 
  }else{
    joystickDrive = map(recFromRemote.Joy1Y, 0,512,(driveSpeed * -1), driveSpeed);  //Read joystick - change -55/55 to adjust for speed. 
  }

#else
  if(recFromRemote.Fwd == 0 || recFromRemote.Fwd == 2){
    joystickDrive = map(recFromRemote.Joy1Y, 0,512,(driveSpeed * -1), driveSpeed);  //Read joystick - change -55/55 to adjust for speed. 
  }else{
    joystickDrive = map(recFromRemote.Joy1Y, 0,512, driveSpeed, (driveSpeed * -1));  //Read joystick - change -55/55 to adjust for speed. 
  }
#endif

      
  if ((joystickDrive > driveAccel) && (driveAccel >= 0)){         
    driveAccel ++;
    Setpoint3 = speedArray[constrain(abs(driveAccel),0, 110)];
  } 
  else if ((joystickDrive < driveAccel) && (driveAccel >= 0)) {
    driveAccel --;
    Setpoint3 = speedArray[constrain(abs(driveAccel),0, 110)];
  }
  else if ((joystickDrive > driveAccel) && (driveAccel <= 0)){
    driveAccel ++;
    Setpoint3 = (speedArray[constrain(abs(driveAccel),0, 110)] * -1);
  } 
  else if ((joystickDrive < driveAccel) && (driveAccel <= 0)) {
    driveAccel --;
    Setpoint3 = (speedArray[constrain(abs(driveAccel),0, 110)] * -1);
  }
    

  Setpoint3 = constrain(Setpoint3, -55, 55);
  #ifndef SHADOW       
  #ifdef reversePitch
    pitch = recIMUData.pitch * -1;
  #else
    pitch = recIMUData.pitch;
  #endif
  
  #ifdef reverseRoll
      roll = recIMUData.roll *-1;
  #else 
      roll = recIMUData.roll;
  #endif
  Input3 = (pitch + pitchOffset) + (Joy2YEaseMap /= 2);// - domeOffset; 
  #else

  #ifdef reversePitch
    pitch = pitch * -1;
  #else
    pitch = pitch;
  #endif
  
  #ifdef reverseRoll
    roll = roll *-1;
  #else 
    roll = roll;
  #endif
  
  #endif

  Input3 = (pitch + pitchOffset) + (Joy2YEaseMap /= 2);// - domeOffset;   
  PID3.Compute();
  if(recFromRemote.CalibID != 2){
    if (Output3 >= 2){    //make BB8 roll
      Output3a = abs(Output3);
      analogWrite(drivePWM1, Output3a);   
      analogWrite(drivePWM2, 0);
    }else if (Output3 < -2){ 
      Output3a = abs(Output3);
      analogWrite(drivePWM2, Output3a);  
      analogWrite(drivePWM1, 0);
    }
  }else{
    analogWrite(drivePWM2, 0);  
    analogWrite(drivePWM1, 0);
  }
}
  
  
/*
 ************************************************************************************ side tilt ************************************************************************************
 */

  void sideTilt() {
  
  #ifdef reverseS2S
    if(recFromRemote.Fwd == 0 || recFromRemote.Fwd == 2){
        joystickS2S = map(constrain(recFromRemote.Joy1X, 0 , 512), 0,512,maxS2STilt,-maxS2STilt); //- is  left, + is  right
    }else{
        joystickS2S = map(constrain(recFromRemote.Joy1X, 0 , 512), 0,512,-maxS2STilt,maxS2STilt); //- is  left, + is  right
    }
  #else
    if(recFromRemote.Fwd == 0 || recFromRemote.Fwd == 2){
        joystickS2S = map(constrain(recFromRemote.Joy1X, 0 , 512), 0,512,-maxS2STilt,maxS2STilt); //- is  left, + is  right
    }else{
        joystickS2S = map(constrain(recFromRemote.Joy1X, 0 , 512), 0,512,maxS2STilt,-maxS2STilt); //- is  left, + is  right
    }
  #endif
  
           // Setpoint will increase/decrease by S2SEase each time the code runs until it matches the joystick. This slows the side to side movement.  
  
    if ((Setpoint2 > -S2SEase) && (Setpoint2 < S2SEase) && (joystickS2S == 0)){
      Setpoint2 = 0;
    }else if ((joystickS2S > Setpoint2) && (joystickS2S != Setpoint2)){
      Setpoint2+=S2SEase;  
    }else if ((joystickS2S < Setpoint2) && (joystickS2S != Setpoint2)){
      Setpoint2-=S2SEase;
    }
  
  #ifdef reverseS2SPot
    S2Spot = map(analogRead(S2SpotPin), 0, 1024, -135,135);
  #else
    S2Spot = map(analogRead(S2SpotPin), 0, 1024, 135,-135);
  #endif
  
           
    Input2 = roll + rollOffset; 
    Setpoint2 = constrain(Setpoint2, -maxS2STilt,maxS2STilt);
    PID2.Compute();  //PID2 is used to control the 'servo' control of the side to side movement. 
    
    Input1  = S2Spot + potOffsetS2S;
    Setpoint1 = map(constrain(Output2, -maxS2STilt,maxS2STilt), -maxS2STilt,maxS2STilt, maxS2STilt,-maxS2STilt);
    PID1.Compute();   //PID1 is for side to side stabilization
    
    if ((Output1 <= -1) && (Input1 > -maxS2STilt)){
      Output1a = abs(Output1);
      analogWrite(s2sPWM2, Output1a);   
      analogWrite(s2sPWM1, 0);
    }else if ((Output1 >= 1) && (Input1 < maxS2STilt)){ 
      Output1a = abs(Output1);
      analogWrite(s2sPWM1, Output1a);  
      analogWrite(s2sPWM2, 0);
    }else{
      analogWrite(s2sPWM2, 0);  
      analogWrite(s2sPWM1, 0);
    }

  }


/*
 *************************************************************************************** Dome Tilt *************************************************************************************
 */
  
#ifdef MK2
    
void domeTiltMK2(){   

#ifdef reverseDomeTilt
  #define revDome1 MaxDomeTiltAngle
  #define revDome2 -MaxDomeTiltAngle
#else
  #define revDome1 -MaxDomeTiltAngle
  #define revDome2 MaxDomeTiltAngle
#endif


  //speedDomeTilt offsets the dome based on the main drive to tilt it in the direction of movement. 
  if (Setpoint3 < 3 && Setpoint3 > -3) {
    speedDomeTilt = 0;
  } else {
    speedDomeTilt = Output3 / 20;
  }

#ifdef reverseDomeTiltPot
  domeTiltPot = (map(analogRead(domeTiltPotPin), 0, 1024, -135, 135) + domeTiltPotOffset);
#else
  domeTiltPot = (map(analogRead(domeTiltPotPin), 0, 1024, 135, -135) + domeTiltPotOffset);
#endif
 

#ifdef TiltDomeForwardWhenDriving
  if(recFromRemote.Fwd == 0 || recFromRemote.Fwd == 2){
    joystickDome = constrain(map(recFromRemote.Joy2Y, 0,512,revDome1,revDome2), -MaxDomeTiltAngle, MaxDomeTiltAngle) - speedDomeTilt;   // Reading the stick for angle -40 to 40
  }else{
    joystickDome = constrain(map(recFromRemote.Joy2Y, 0,512,revDome2,revDome1),-MaxDomeTiltAngle , MaxDomeTiltAngle);   // Reading the stick for angle -40 to 40
  }
#else
  if(recFromRemote.Fwd == 0 || recFromRemote.Fwd == 2){  
    joystickDome = constrain(map(recFromRemote.Joy2Y, 0,512,revDome2,revDome1),-MaxDomeTiltAngle , MaxDomeTiltAngle);   // Reading the stick for angle -40 to 40
  }else{
    joystickDome = constrain(map(recFromRemote.Joy2Y, 0,512,revDome1,revDome2), -MaxDomeTiltAngle, MaxDomeTiltAngle) - speedDomeTilt;   // Reading the stick for angle -40 to 40
  }
#endif        
         
  Input4  = domeTiltPot + (pitch + pitchOffset);
  
  if ((Setpoint4 > -1) && (Setpoint4 < 1) && (joystickDome == 0)){
    Setpoint4 = 0;
  }else if ((joystickDome > Setpoint4) && (joystickDome != Setpoint4)){
    Setpoint4 += easeDomeTilt;
  }else if ((joystickDome < Setpoint4) && (joystickDome != Setpoint4)){
    Setpoint4 -= easeDomeTilt;
  }
  
  Setpoint4 = constrain(Setpoint4, -MaxDomeTiltAngle,MaxDomeTiltAngle);
  
  PID4.Compute();
  
  if (Output4 < -0 && domeTiltPot > -25){
    Output4a = abs(Output4);
    analogWrite(domeTiltPWM2, Output4a);    
    analogWrite(domeTiltPWM1, 0);
  }else if (Output4 >= 0 && domeTiltPot < 25){ 
    Output4a = abs(Output4);
    analogWrite(domeTiltPWM1, Output4a);  
    analogWrite(domeTiltPWM2, 0);
  }else{
    analogWrite(domeTiltPWM2, 0);  
    analogWrite(domeTiltPWM1, 0);
  }

}
#endif


#ifdef MK3
void domeTiltMK3(){ 
  if(recFromRemote.Fwd == 0 || recFromRemote.Fwd == 2){
    Joy2XDirection = map(recFromRemote.Joy2X, 0, 512, -MaxDomeTiltX, MaxDomeTiltX);
    Joy2YDirection = map(recFromRemote.Joy2Y, 0, 512, -MaxDomeTiltY, MaxDomeTiltY);
  }else{
    Joy2YDirection = map(recFromRemote.Joy2Y, 0, 512, MaxDomeTiltY, -MaxDomeTiltY);
    Joy2XDirection = map(recFromRemote.Joy2X, 0, 512, MaxDomeTiltX, -MaxDomeTiltX);
  }

  if(Joy2YDirection <= 1.7 && Joy2YDirection >= -1.7){  
     Joy2YDirection = 0;   
  }

  if(Joy2XDirection <= 1.7 && Joy2XDirection >= -1.7){  
     Joy2XDirection = 0;   
  }
    
  if(Setpoint3 >= 2 || Setpoint3 <= -2){
    Joy2YPitch = Joy2YDirection + pitch;
  }else{
    Joy2YPitch = Joy2YDirection - pitchOffset;
  }
  
  if((Joy2XDirection + DomeXEase) > Joy2XEase && (Joy2XDirection - DomeXEase) < Joy2XEase){
    Joy2XEase = Joy2XDirection;
  }else if(Joy2XEase > Joy2XDirection){
    Joy2XEase -= DomeXEase;
  }else if(Joy2XEase < Joy2XDirection){
    Joy2XEase += DomeXEase;
  }
  if((Joy2YPitch + DomeYEase) > Joy2YEase && (Joy2YPitch - DomeYEase) < Joy2YEase){
    Joy2YEase = Joy2YPitch;
  }else if(Joy2YEase > Joy2YPitch){
    Joy2YEase -= DomeYEase;
  }else if(Joy2YEase < Joy2YPitch){
    Joy2YEase += DomeYEase;
  }
  
  Joy2XEaseMap = Joy2XEase;
  Joy2YEaseMap = Joy2YEase;

  if(Joy2YEaseMap< 0){
    Joy2Ya = map(Joy2YEaseMap, -20, 0, 70, 0);
    Joy2XLowOffset = map(Joy2Ya, 1, 70, -15, -50);
    Joy2XHighOffset = map(Joy2Ya, 1, 70, 30, 20);
  }else if(Joy2YEaseMap> 0){
    Joy2Ya = map(Joy2YEaseMap,0, 24, 0, -80); 
    Joy2XLowOffset = map(Joy2Ya, -1, -80, -15, 10);
    Joy2XHighOffset = map(Joy2Ya, -1, -80, 30, 90);
  }else{
    Joy2Ya = 0;
  }
  
  if(Joy2XEaseMap> 0){
    Joy2XLowOffsetA = map(Joy2XEaseMap, 0, 18, 0, Joy2XLowOffset);
    Joy2XHighOffsetA = map(Joy2XEaseMap, 0, 18, 0, Joy2XHighOffset);
    ServoLeft = Joy2Ya + Joy2XHighOffsetA;
    ServoRight = Joy2Ya + Joy2XLowOffsetA;
   
  }else if(Joy2XEaseMap< 0){
    Joy2XLowOffsetA = map(Joy2XEaseMap, -18, 0, Joy2XLowOffset, 0);
    Joy2XHighOffsetA = map(Joy2XEaseMap, -18, 0, Joy2XHighOffset, 0);
    ServoRight = Joy2Ya + Joy2XHighOffsetA;
    ServoLeft = Joy2Ya + Joy2XLowOffsetA;
  }else{
    Joy2XHighOffsetA = 0;
    Joy2XLowOffsetA = 0; 
    ServoRight = Joy2Ya;
    ServoLeft = Joy2Ya;
  }
 
  leftServo.write(constrain(map(ServoLeft, -90, 90, 0, 180),0, 180) +5, domeSpeed, false); 
  rightServo.write(constrain(map(ServoRight,-90, 90, 180, 0), 0, 180), domeSpeed, false);
  
}
#endif

  
  void domeSpin() {
  
  
  #ifdef reverseDomeSpin
    #ifdef MK2
      domeRotation = map(recFromRemote.Joy2X, 0,512,-255,255);
    #else
      domeRotation = map(recFromRemote.Joy4X, 0,512,-255,255);
    #endif
  #else
    #ifdef MK2
      domeRotation = map(recFromRemote.Joy2X, 0,512,255,-255);
    #else
      domeRotation = map(recFromRemote.Joy4X, 0,512,255,-255);
    #endif
  #endif
         
  
      if (domeRotation < 25 && domeRotation > -25 && currentDomeSpeed > -easeDomeUp && currentDomeSpeed < easeDomeUp) {
        domeRotation = 0;
        currentDomeSpeed = 0;
      }
      
      if ((domeRotation > currentDomeSpeed) && (currentDomeSpeed >= 0)){
        currentDomeSpeed += easeDomeUp ;
      }else if ((domeRotation < currentDomeSpeed) && (currentDomeSpeed >= 0)){
        currentDomeSpeed -= easeDomeDown ;
      }else if ((domeRotation > currentDomeSpeed) && (currentDomeSpeed <= 0)){
        currentDomeSpeed += easeDomeUp ;
      }else if ((domeRotation < currentDomeSpeed) && (currentDomeSpeed <= 0)){
        currentDomeSpeed -= easeDomeDown ;
      } 
        
     
      if ((currentDomeSpeed<=-20) && (ControllerStatus == 0 && IMUStatus == 0)){
        currentDomeSpeed = constrain(currentDomeSpeed,-255,255);
        analogWrite(domeSpinPWM2, 0);
        analogWrite(domeSpinPWM1, abs(currentDomeSpeed));
      }else if ((currentDomeSpeed>=20) && (ControllerStatus == 0 && IMUStatus == 0)){
        currentDomeSpeed = constrain(currentDomeSpeed,-255,255);
        analogWrite(domeSpinPWM1, 0);
        analogWrite(domeSpinPWM2, abs(currentDomeSpeed));
      }else {
        analogWrite(domeSpinPWM1, 0);
        analogWrite(domeSpinPWM2, 0);
      }   
    }
  
  void domeSpinServo() {
      
  #ifndef reverseDomeSpinServo
      #ifdef MK2
        ch4Servo = map(recFromRemote.Joy2X, 0, 512, domeServoModeAngle, -domeServoModeAngle);
      #else
        ch4Servo = map(recFromRemote.Joy4X, 0, 512, domeServoModeAngle, -domeServoModeAngle);
      #endif
  #else
      #ifdef MK2
        ch4Servo = map(recFromRemote.Joy2X, 0, 512, -domeServoModeAngle, domeServoModeAngle);
      #else
        ch4Servo = map(recFromRemote.Joy4X, 0, 512, -domeServoModeAngle, domeServoModeAngle);
      #endif
  #endif
  
  #ifdef reverseDomeSpinPot
      if(recFromRemote.Fwd == 1 || recFromRemote.Fwd == 2){
        Input5 = ((map(analogRead(domeSpinPot),0, 1023, -180, 180) + domeSpinOffset)-180);
      }else {
        Input5 = map(analogRead(domeSpinPot),0, 1023, -180, 180) + domeSpinOffset;
      }
  #else
      if(recFromRemote.Fwd == 1 || recFromRemote.Fwd == 2){
        Input5 = ((map(analogRead(domeSpinPot),0, 1023, 180, -180) + domeSpinOffset)-180);
      }else {
        Input5 = map(analogRead(domeSpinPot),0, 1023, 180, -180) + domeSpinOffset;
      }
  #endif
  
      if (Input5 < -180){
        Input5 += 360;
      } else if (Input5 > 180){
        Input5 -= 360;
      } else {
        Input5 = Input5;
      }
  
  
    if ((Setpoint5 > -5) && (Setpoint5 < 5) && (ch4Servo == 0)){
      Setpoint5 = 0;
    }else if ((ch4Servo > (Setpoint5 + 2)) && (ch4Servo != Setpoint5)){
      Setpoint5+=5;  
    }else if ((ch4Servo < (Setpoint5 -2)) && (ch4Servo != Setpoint5)){
      Setpoint5-=5;
    }
    constrain(Setpoint5, -domeServoModeAngle, domeServoModeAngle);
      
    PID5.Compute();
  
  
          
    if (Output5 < -4){
      Output5a = constrain(abs(Output5),0, 255);
      analogWrite(domeSpinPWM1, Output5a);     
      analogWrite(domeSpinPWM2, 0);
    }else if (Output5 > 4){ 
      Output5a = constrain(abs(Output5), 0, 255);
      analogWrite(domeSpinPWM2, Output5a);  
      analogWrite(domeSpinPWM1, 0);
    }else{
      analogWrite(domeSpinPWM2, 0);  
      analogWrite(domeSpinPWM1, 0);
    }
      
  }
  
  
  
 /*
 *********************************************************************************** Flywheel Spin **********************************************************************************
 */
  
  void flywheelSpin() {
  
  #ifdef reverseFlywheel
    ch5PWM = constrain(map(recFromRemote.Joy3X, 0,512,255,-255),-255,255);
  
  #else
    ch5PWM = constrain(map(recFromRemote.Joy3X, 0,512,-255,255),-255,255);
  
  #endif
            
    if(ch5PWM > -1 && ch5PWM < 35){
      ch5PWM = 0;
    } else if(ch5PWM < 0 && ch5PWM > -35){
      ch5PWM = 0;
    } else if(ch5PWM > 35){
      map(ch5PWM, 35, 255, 0, 255);
    } else if(ch5PWM < -35){
      map(ch5PWM, -35, -255, 0, -255);
    }
  
    constrain(ch5PWM, -255, 255);
      
      
    if(ch5PWM >= 200 && flywheelRotation <= -100){
      flywheelRotation = 255;
    }else if(ch5PWM <= -200 && flywheelRotation >= 100){
      flywheelRotation = -255;
    }else if(ch5PWM < -220 && (flywheelRotation > -30 && flywheelRotation < 30)){
      flywheelRotation = -255;
    }else if(ch5PWM > 220 && (flywheelRotation > -30 && flywheelRotation < 30)){
      flywheelRotation = 255;
    }else if(ch5PWM > flywheelRotation) {
                flywheelRotation+=flywheelEase;           
    } else if (ch5PWM < flywheelRotation) {
      flywheelRotation-=flywheelEase; 
    }        
    
    constrain(flywheelRotation, -255, 255);
    
    if ((flywheelRotation < -10) && (ControllerStatus == 0 && IMUStatus == 0) && (recFromRemote.motorEnable == 0)){
      analogWrite(flywheelSpinPWM1, 0);
      analogWrite(flywheelSpinPWM2, abs(flywheelRotation));
    }
    else if ((flywheelRotation > 10) && (ControllerStatus == 0 && IMUStatus == 0) && (recFromRemote.motorEnable == 0)){
      analogWrite(flywheelSpinPWM2, 0);
      analogWrite(flywheelSpinPWM1, abs(flywheelRotation));
    }
    else {
      analogWrite(flywheelSpinPWM1, 0);
      analogWrite(flywheelSpinPWM2, 0);
    }  
  
  }


/*
 ********************************************************************************** Auto Disable Motors ********************************************************************************
 */


  void autoDisableMotors(){
  
    if(recFromRemote.CalibID < 2){
      
      if((joystickDrive > -2 && joystickDrive < 2) && (joystickS2S > -2 && joystickS2S < 2) && (joystickDome > -2 && joystickDome < 2) && (flywheelRotation < 25 && flywheelRotation > -25) && (recFromRemote.Joy2X < 276 && recFromRemote.Joy2X > 236) && (autoDisableState == 0)){
  
        autoDisableMotorsMillis = millis();
        autoDisableState = 1;
          
      } else if(joystickDrive < -2 || joystickDrive > 2 || joystickS2S < -2 || joystickS2S > 2 || joystickDome < -2 || joystickDome > 2 || flywheelRotation > 30 || flywheelRotation < -30 || recFromRemote.Joy2X > 276 || recFromRemote.Joy2X < 236 || (lastDirection != recFromRemote.lJoySelect)){
  
        autoDisableState = 0;     
        digitalWrite(enablePin, HIGH); 
        digitalWrite(S2SenablePin, HIGH);
        digitalWrite(enablePinDome, HIGH);
        autoDisableDoubleCheck = 0; 
        autoDisable = 0;
        if(recFromRemote.lJoySelect != lastDirection){
          
          lastDirection = recFromRemote.lJoySelect;
        }
      
      }
              
      if(autoDisableState == 1 && (millis() - autoDisableMotorsMillis >= 3000) && Output1a < 25 && Output3a < 8){
        digitalWrite(enablePin, LOW);
        digitalWrite(S2SenablePin, LOW);
        digitalWrite(enablePinDome, LOW);
        
        autoDisable = 1;
          
      }else if(Output1a > 50 || Output3a > 20){
        autoDisableState = 0;
        digitalWrite(enablePin, HIGH);
        digitalWrite(S2SenablePin, HIGH);
        digitalWrite(enablePinDome, HIGH);
        autoDisableDoubleCheck = 0;  
        autoDisable = 0;    
      }else if((Output1a > 25 || Output3a > 8) && autoDisableDoubleCheck == 0){
        autoDisableDoubleCheckMillis = millis();
        autoDisableDoubleCheck = 1;
         
      } else if((autoDisableDoubleCheck == 1) && (millis() - autoDisableDoubleCheckMillis >= 100)){
        if(Output1a > 30 || Output3a > 8){ 
          autoDisableState = 0;
          digitalWrite(enablePin, HIGH);
          digitalWrite(S2SenablePin, HIGH);
          digitalWrite(enablePinDome, HIGH);
          autoDisableDoubleCheck = 0;
          autoDisable = 0;
        }else{
          autoDisableDoubleCheck = 0;
        }
      } 
      
    }else if(recFromRemote.CalibID == 2){ //drive and dome tilt
      digitalWrite(enablePin, HIGH);
      digitalWrite(S2SenablePin, LOW);
      digitalWrite(enablePinDome, LOW);
    
    }else if(recFromRemote.CalibID == 3){//S2S
      digitalWrite(enablePin, LOW);
      digitalWrite(S2SenablePin, HIGH);
      digitalWrite(enablePinDome, LOW);  
    }else if(recFromRemote.CalibID == 4){
      digitalWrite(enablePin, LOW);
      digitalWrite(S2SenablePin, LOW);
      digitalWrite(enablePinDome, LOW);   
    }
  }
  

/*
 ************************************************************************************ Set Drive Speed ****************************************************************************
 */

  void setDriveSpeed(){  
      
    if(lastSpeed != recFromRemote.Speed){
      lastSpeed = recFromRemote.Speed;
      if(recFromRemote.Speed == 0){
        driveSpeed = 55;
      }else if(recFromRemote.Speed == 1){
        driveSpeed = 75;
      }else if(recFromRemote.Speed == 2){
        driveSpeed = 110;
      }
    }
  }
  

/*
 **************************************************************************************** Read Voltage *******************************************************************************
 */

  void readVin() {
  
    if(millis() - lastBatteryUpdate >= 15000){
      lastBatteryUpdate = millis();
      sendTo.bodyBatt = ((analogRead(battMonitor) * outputVoltage) / 1024.0) / (R2/(R1+R2));
    }
  }
  

/*
 **************************************************************************************** Read Calibration ID *******************************************************************************
 */

  
  void   checkCalibID(){
   
    if (recFromRemote.CalibID == 1 && lastCalibID != 1){
      lastCalibID = 1;
      setDomeSpinOffset();
    }else if(recFromRemote.CalibID == 3 && lastCalibID !=3){
      lastCalibID = 3;
      setPitchOffset();
      setDomeTiltOffset();
    }else if(recFromRemote.CalibID == 4 && lastCalibID != 4){
      lastCalibID = 4;
      setS2SOffset();
    }else if(recFromRemote.CalibID == 0 && lastCalibID != 0){
      lastCalibID = 0;   
    }
  
   
  }  
  
/*====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ================================================================================ Disable Droid ===================================================================================== 
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 */

 
  void turnOffAllTheThings(){
    //disables all PIDS and movement. This is to avoid any sudden jerks when re-enabling motors. 
    
      joystickS2S = 0;
      Input2 = 0;
      Setpoint2 = 0; 
      Output2 = 0;
      Input1 = 0;
      Setpoint1 = 0;
      Output1 = 0;
      joystickDrive = 0;
      driveAccel = 0;
      Input3 = 0;
      Setpoint3 = 0;
      Output3 = 0;
      joystickDome = 0;
      Input4 = 0;
      Setpoint4 = 0;
      Output4 = 0;
      flywheelRotation = 0;
      ch5PWM = 0;
    if(ControllerStatus == 1){
      analogWrite(domeSpinPWM2, 0);  
      analogWrite(domeSpinPWM1, 0);
    }
  }

 
  
    
/*=====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ================================================================================= Set Offsets ====================================================================================== 
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 */ 
  
void setDomeSpinOffset() {

  if(recFromRemote.Fwd == 0 || recFromRemote.Fwd == 2){
    domeSpinOffset = 0 - map(analogRead(domeSpinPot),0, 1023, 180, -180);
  }else{
    domeSpinOffset = 180 - map(analogRead(domeSpinPot),0, 1023, 180, -180);
  }

  EEPROM.writeInt(12,domeSpinOffset); 


}
  
  
void setPitchOffset(){

  if(recIMUData.pitch < 0){
    pitchOffset = abs(recIMUData.pitch);
  }else{
        pitchOffset = recIMUData.pitch * -1;
  }
  EEPROM.writeFloat(0,pitchOffset);

}
  
void setDomeTiltOffset(){
#ifdef MK2
#ifdef reverseDomeTiltPot
  if(map(analogRead(domeTiltPotPin), 0, 1024, -135, 135) < 0){
    domeTiltPotOffset = abs(map(analogRead(domeTiltPotPin), 0, 1024, 135, -135));
  }else{
    domeTiltPotOffset = map(analogRead(domeTiltPotPin), 0, 1024, -135, 135) * -1;
  }
#else
  if(map(analogRead(domeTiltPotPin), 0, 1024, 135, -135) < 0){
    domeTiltPotOffset = abs(map(analogRead(domeTiltPotPin), 0, 1024, 135, -135));
  }else{
    domeTiltPotOffset = map(analogRead(domeTiltPotPin), 0, 1024, 135, -135) * -1;
  }
#endif
  EEPROM.writeInt(10,domeTiltPotOffset);
#endif
}
  
void setS2SOffset(){
  if(recIMUData.roll < 0){
    rollOffset = abs(recIMUData.roll);
  }else{
      rollOffset = recIMUData.roll * -1;
  }

  if(S2Spot < 0){
    potOffsetS2S = abs(S2Spot);
  }else{
    potOffsetS2S = S2Spot * -1;
  }
  EEPROM.writeFloat(4,rollOffset);
  EEPROM.writeInt(8,potOffsetS2S);
}



  
 /*====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ================================================================================== Sounds and PSI ================================================================================== 
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 */
  
  #ifdef SerialSound
  void sounds(){
    readPinState = digitalRead(ACTpin);
    
    if((recFromRemote.lBut1 == 1 || recFromRemote.rBut3 == 1 )&& soundState == 0){   // Press lBut1 or rBut3 and it plays a random speach track. 
      soundState = 1;
      psiState = 1;
      Serial3.print(F("#")); Serial3.println(voiceNum);
      soundMillis = millis(); 
      
    }else if(recFromRemote.lBut1 == 0 && recFromRemote.rBut3 == 0 && soundState == 1){ // Let go of lBut1 and rBut3 set's it back to "0". 
      soundState = 0;
      
      voiceNum =  random(0,numberOfVoice);
      
    }else if(recFromRemote.lBut2 == 1 && soundState == 0){  // Press lBut2 to sequentially play through the "music" tracks
      soundState = 2;
      Serial3.print(F("#")); Serial3.println(numberOfVoice + musicNum);
    }else if(recFromRemote.lBut2 == 0 && soundState == 2){   // Let go of lBut2 to set music state back to 0
      soundState = 0;
      musicNum++;
      if(musicNum == numberOfMusic){
        musicNum = 0;
      }
    }else if(recFromRemote.lBut1 > 1 || recFromRemote.rBut3 > 1 && soundState == 0){   // if lBut1 or Rbut3 is "multi-pressed" it will play the "quickVoice" associated with that number of presses
      psiState = 1;
      soundMillis = millis(); 
      //byte Switch = recFromRemote.lBut1; 
      switch (recFromRemote.lBut1) {
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
    }else if(recFromRemote.lBut2 > 1 && soundState == 0){      // if lBut2 is "multi-pressed" it will play the "quickMusic" associated with that number of presses
      
      switch (recFromRemote.lBut2 ) {
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
  
    if(recFromRemote.rBut2  == 0 && quitState == 0){
      quitState = 1;
     // rBut2Millis = millis();

    }else if(recFromRemote.rBut2 == 1 && quitState == 1){
      quitState = 0;
      if(digitalRead(ACTpin) == LOW){
        Serial3.println("q");
        psiState = 0;
        sendTo.PSI = 0;
      }
    }
    
    
    psiVal();
  
  
  
  }
  
#endif
#ifndef SHADOW
void sounds(){

  readPinState = digitalRead(ACTpin);

  if(ControllerStatus == 0 ){
      
    if ((recFromRemote.lBut1 == 0 || recFromRemote.rBut3 == 0) && recFromRemote.rBut2 == 1  && playSound == 0 && readPinState == 1){             
        playSound = 1;       
    }else if (recFromRemote.lBut2 == 0 && readPinState == 1){  
        digitalWrite(soundpin6, LOW);
    }else if (recFromRemote.lBut2 == 1) {
        digitalWrite(soundpin6, HIGH);
    }     
      
    if(playSound == 1){
        randSoundPin = random(0, 5);
        digitalWrite((soundPins[randSoundPin]), LOW);
        soundMillis = millis();
        playSound = 2;
        psiState = 1; 
    }else if(playSound == 2 && (millis() - soundMillis > 200)){
        digitalWrite((soundPins[randSoundPin]), HIGH);
        playSound = 0;
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
   sendTo.lBut3 = recFromRemote.lBut3;
   
  }
  
  
 /*====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ===================================================================================== Debug ======================================================================================== 
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 * ====================================================================================================================================================================================
 */
 
void debugRoutines(){
      
#ifdef debugDrive
  Serial.print(F(" joystickDrive: "));
  Serial.print(joystickDrive);
  Serial.print(F(" accel: "));
  Serial.print(driveAccel);
  Serial.print(F(" SetDrive: "));
  Serial.print(Setpoint3);
  Serial.print(F(" InDrive: "));
  Serial.print(Input3);
  Serial.print(F(" OutDrive: "));
  Serial.println(Output3);
#endif

#ifdef debugS2S
  Serial.print(F(" joystickS2S: "));
  Serial.print(joystickS2S);
  Serial.print(F(" Roll: "));
  Serial.print(roll);
  Serial.print(F(" RollOffset: "));
  Serial.print(rollOffset);
  Serial.print(F(" S2SPot: "));
  Serial.print(S2Spot);
  Serial.print(F(" potOffsetS2S: "));
  Serial.print(potOffsetS2S);
  Serial.print(F(" In2: "));
  Serial.print(Input2);
  Serial.print(F(" Set2: "));
  Serial.print(Setpoint2);
  Serial.print(F(" Out2/Set1: "));
  Serial.print(Output2);
  Serial.print(F(" In1: "));
  Serial.print(Input1);
  Serial.print(F(" Out1: "));
  Serial.println(Output1);
  
#endif

#ifdef debugDomeTilt
  #ifdef MK3
  Serial.print(F(" Joy2YDirection: ")); 
  Serial.print(Joy2YDirection);
  Serial.print(F(" pitchOffset: ")); 
  Serial.print(pitchOffset);
  Serial.print(F(" Setpoint3: ")); 
  Serial.print(Setpoint3);
  Serial.print(F(" Joy2YPitch: ")); 
  Serial.print(Joy2YPitch);
  Serial.print(F("DomeYEase: ")); 
  Serial.println(DomeYEase); 
  #else
  
  Serial.print(F(" joystickDome: "));
  Serial.print(joystickDome);
  Serial.print(F(" In4:"));
  Serial.print(Input4);
  Serial.print(F(" Actual domeTiltPot:"));
  Serial.print(analogRead(domeTiltPotPin));
  Serial.print(F(" Mapped domeTiltPot:"));
  Serial.print(domeTiltPot);
  Serial.print(F(" domeTiltPotOffset:"));
  Serial.print(domeTiltPotOffset);
  Serial.print(F(" pitch:"));
  Serial.print(pitch);
  Serial.print(F(" pitchOffset:"));
  Serial.print(pitchOffset);
  Serial.print(F(" Set4 :"));
  Serial.print(Setpoint4);
  Serial.print(F(" Out4 :"));
  Serial.println(Output4);

  #endif

#endif

#ifdef debugdomeRotation

  Serial.print(F(" domeRotation: "));
  Serial.print(domeRotation);
  Serial.print(F(" currentDomeSpeed: "));
  Serial.print(currentDomeSpeed);
  Serial.print(F(" ch4Servo: "));
  Serial.print(ch4Servo);
  Serial.print(F(" In5: "));
  Serial.print(Input5);
  Serial.print(F(" Set5: "));
  Serial.print(Setpoint5);
  Serial.print(F(" Out5: "));
  Serial.print(Output5);
  Serial.print(F(" domeServo: "));
  Serial.print(domeServo);
  Serial.print(F(" domeSpinOffset: "));
  Serial.print(domeSpinOffset);
  Serial.print(F(" pot: "));
  Serial.print(analogRead(domeSpinPot));
  Serial.print(F(" lJoySelect: "));
  Serial.print(recFromRemote.lJoySelect);
  Serial.print(F(" motorEnable: "));
  Serial.println(recFromRemote.motorEnable);

#endif



#ifdef printbodyBatt

  Serial.print(F(" Vin: "));
  Serial.print(bodyBatt);

#endif


#ifdef printYPR

  Serial.print(F(" Roll: "));
  Serial.print(roll);
  Serial.print(F("  Pitch: "));
  Serial.println(pitch);

 #endif

 #ifdef printDome
 
  Serial.print(F(" Dome Batt: "));
  Serial.print(recFromDome.domeBatt);
  Serial.print (F(" PSI: "));
  Serial.print (PSI);

 #endif


 #ifdef printRemote

  Serial.print (F("  Remote: "));
  Serial.print (recFromRemote.Joy1Y);
  Serial.print (" , ");
  Serial.print (recFromRemote.Joy1X);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.Joy2Y);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.Joy2X);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.Joy3X);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.Joy4X);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.lJoySelect);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.lBut1);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.lBut2);  
  Serial.print (F(" , "));
  Serial.print (recFromRemote.lBut3);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.Fwd);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.Speed);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.rBut2);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.rBut3);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.motorEnable);
  Serial.print (F(" , "));
  Serial.print (recFromRemote.CalibID);
  Serial.print ('\n');

#endif

#ifdef printOffsets

  Serial.print(F(" pitchOffset: "));
  Serial.print(pitchOffset);
  Serial.print(F(" rollOffset: "));
  Serial.print(rollOffset);
  Serial.print(F(" potOffsetS2S: "));
  Serial.print(potOffsetS2S);
  Serial.print(F("domeTiltPotOffset: "));
  Serial.println(domeTiltPotOffset);
  
#endif



#ifdef printOutputs

  Serial.print(F(" Out1: ")); Serial.print(Output1a);
  Serial.print(F(" Out2: ")); Serial.print(Output2a);
  Serial.print(F(" Out3: ")); Serial.print(Output3a);
  Serial.print(F(" Out4: ")); Serial.println(Output4a);

#endif

#ifdef printSoundPins

  Serial.print(F(" Pin1: ")); Serial.print(digitalRead(soundpin1));
  Serial.print(F(" Pin2: ")); Serial.print(digitalRead(soundpin2));
  Serial.print(F(" Pin3: ")); Serial.print(digitalRead(soundpin3));
  Serial.print(F(" Pin4: ")); Serial.print(digitalRead(soundpin4));
  Serial.print(F(" Pin5: ")); Serial.print(digitalRead(soundpin5));
  Serial.print(F(" Pin6: ")); Serial.print(digitalRead(soundpin6));
  Serial.print(F(" readPinState: ")); Serial.print(digitalRead(ACTpin));
  Serial.print(F(" randSoundPin: ")); Serial.println(randSoundPin);

#endif

#ifdef debugFlywheelSpin

  Serial.print(F(" ch5: "));
  Serial.print(recFromRemote.Joy3X);
  Serial.print(F(" ch5PWM: "));
  Serial.print(ch5PWM);
  Serial.print(F(" flywheelRotation: "));
  Serial.println(flywheelRotation);

#endif

}
  
    
  
  



 
