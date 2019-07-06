  // ===================================================================================================================================================================================================== 
  //                         Joe's Drive  - Dome "MK3"  - Updated 03/21/2019
  //
  //             ***         You are free to use, and share this code as long as it is not sold. There is no warranty, guarantee, or other tomfoolery. 
  //                         This entire project was masterminded by an average Joe, your mileage may vary. 
  // ===================================================================================================================================================================================================== 
  //                            Written by Joe Latiola - https://www.facebook.com/groups/JoesDrive/
  //                            Shadow alternate Controls Written by James VanDusen
  //                            You will need libraries: AdafruitNeopixel: https://github.com/adafruit/Adafruit_NeoPixel
  //                                                     Modified Low PowerLab: https://travis-ci.org/LowPowerLab/RFM69
  //                                                     Github RFM69: https://github.com/LowPowerLab/RFM69
  //
  //
  // ===================================================================================================================================================================================================== 
  // =====================================================================================================================================================================================================
  
      
  #define psiPIN      10
  #define sLogicPIN   5
  #define lLogicPIN   6 
  #define hpPIN       3
  #define eyePIN      11
  #define battPin     A9
  #define dataDelay   0
  #define recDelay    10
  #define interval    40
  #define interval2   2 
  #define sendDelay   50 

  //#define useNeoPixelPSI   // Uncomment if you are using a neopixel for the PSI, leave commented if using a standard led.
  
  #include <Adafruit_NeoPixel.h>
  
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

  

  #define FREQUENCY     RF69_915MHZ  // THIS NEEDS TO MATCH YOUR FEATHER


  
  #define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
  
  //*********************************************************************************************
  #define RFM69_CS      8
  #define RFM69_IRQ     7
  #define RFM69_IRQN    4  // Pin 7 is IRQ 4!
  #define RFM69_RST     4
  
  RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);


  
  struct SEND_DATA_STRUCTURE{
        float bodyBatt;
        float domeBatt;
    
  }sendFromDome;
  byte packet[sizeof(sendFromDome)];
  float domeBatt1;
    
  typedef struct RECEIVE_DATA_STRUCTURE{

        int psi=0;
        byte button4 = 1;
        float bodyBatt;
        
  }recBodyData;
  recBodyData recFromBody;
  
  byte i, a, b, Set = 1, s;

  
  #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
  bool blinkState = false;
    
  
  Adafruit_NeoPixel sLOGIC = Adafruit_NeoPixel(3, sLogicPIN, NEO_GRB + NEO_KHZ800); 
  Adafruit_NeoPixel lLOGIC = Adafruit_NeoPixel(6, lLogicPIN, NEO_GRB + NEO_KHZ800); 
  Adafruit_NeoPixel HP = Adafruit_NeoPixel(1, hpPIN, NEO_RGB + NEO_KHZ800); 
  Adafruit_NeoPixel EYE = Adafruit_NeoPixel(1, eyePIN, NEO_GRB + NEO_KHZ800); 

  #ifdef useNeoPixelPSI
      Adafruit_NeoPixel PSI = Adafruit_NeoPixel(1, psiPIN, NEO_GRB + NEO_KHZ800); 
  #endif

  unsigned long randomMillis, previousMillis, previousMillis2, lastSendRecMillis;
  unsigned long lastHPCycleMillis, randomMillisSingle, but4StateMillis, lastBattUpdate;
  unsigned long lastBodyReceive, lastFlash;
  int psiVal;
  
  const byte numChars = 32;
  int LEDState = 1;
  int but4State;
  int flashtime;
  int holoPulseState = 2;
  int bpulse = 80;

  int hpCycleState, hpRed, hpGreen, hpBlue;

  int rearFadeState, rearFadeRed, rearFadeBlue, rearFadeGreen;
  
  //     ==========================================================================
  
    
  void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);


    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    sLOGIC.begin();
    lLOGIC.begin();
    HP.begin();
    EYE.begin();
    #ifdef useNeoPixelPSI
      PSI.begin();
    #endif
  
      // Hard Reset the RFM module
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, HIGH);
    delay(100);
    digitalWrite(RFM69_RST, LOW);
    delay(100);
  
    // Initialize radio
    radio.initialize(FREQUENCY,DOME_ADDRESS,NETWORKID);
    if (IS_RFM69HCW) {
      radio.setHighPower();    // Only for RFM69HCW & HW!
    }
    radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

   
    eyeLED();
     
    }
    
  void loop () {
    
    sendAndReceive();
    
    if(millis() - previousMillis > interval) {
      previousMillis = millis();
      
      if (LEDState == 1){
        doubleLogic();
        Holo();
        rearLogic();
      }else if (LEDState == 2){
        doubleLogicFade();
        holoPulse();
        rearLogicFade();
        if(hpCycleState != 0){
          hpCycleState = 0;
        }
      }else if (LEDState == 3){
        doubleLogicRandom();
        hpCycle();
        rearLogicRandom();
      }
  
      if (recFromBody.button4 == 0 && but4State < 2){
        LED_State();
      } else if(recFromBody.button4 == 1 && but4State != 0){
        but4State = 0;
      }
    }
  }  
