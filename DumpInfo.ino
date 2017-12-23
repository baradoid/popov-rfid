/*
   --------------------------------------------------------------------------------------------------------------------
   Example sketch/program showing how to read data from a PICC to serial.
   --------------------------------------------------------------------------------------------------------------------
   This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid

   Example sketch/program showing how to read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
   Reader on the Arduino SPI interface.

   When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
   then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
   you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
   will show the ID/UID, type and any data blocks it can read. Note: you may see "Timeout in communication" messages
   when removing the PICC from reading distance too early.

   If your reader supports it, this sketch/program will read all the PICCs presented (that is: multiple tag reading).
   So if you stack two or more PICCs on top of each other and present them to the reader, it will first output all
   details of the first and then the next PICC. Note that this may take some time as all data blocks are dumped, so
   keep the PICCs at reading distance until complete.

   @license Released into the public domain.

   Typical pin layout used:
   -----------------------------------------------------------------------------------------
               MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
               Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
   Signal      Pin          Pin           Pin       Pin        Pin              Pin
   -----------------------------------------------------------------------------------------
   RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
   SPI SS      SDA(SS)      10            53        D10        10               10
   SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
   SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
   SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
*/

#include <SPI.h>
#include <MFRC522.h>
#include "TimerOne.h"

#define pinLed0 14

#define LED_ENA        0 
#define LED_BLINK_SLOW 1
#define LED_BLINK_FAST 2

#define DEV_NUM 8
//int ledState[devNum];
byte ssPinNum[] =   {10, 9,  8,  7,  6, 5, 4, 3};
bool bDevEna[] = {false, false, false, false, false, false, false, false};

enum ELedState {ena, blink_slow, blink_fast, disable};
ELedState ledState[DEV_NUM];
unsigned long lastEnaTime[DEV_NUM];
bool bLedEna[DEV_NUM];

//byte ledPinNump[] = {13, 14, 16, 17, 18, 19, 2};

long cardFirstTimeChecked[DEV_NUM];


#define RST_PIN         UINT8_MAX          // Configurable, see typical pin layout above
//#define SS_PIN          10         // Configurable, see typical pin layout above

#define RCK_PIN 15
MFRC522 mfrc522;  // Create MFRC522 instance
//MFRC522 mfrc522_1(10, UINT8_MAX);  // Create MFRC522 instance

void setup() {
  Serial.begin(115200);   // Initialize serial communications with the PC
  while (!Serial);    // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  Serial.println(F("Power up"));
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);


  pinMode(RCK_PIN, OUTPUT);
  digitalWrite(RCK_PIN, LOW);
  
  SPI.begin();      // Init SPI bus

  for (int i = 0; i < DEV_NUM; i++) {
    ledState[i] = disable;
    bLedEna[i] = true;
//    pinMode(ledPinNump[i], OUTPUT);  //Led0
//    digitalWrite(ledPinNump[i], HIGH);
    cardFirstTimeChecked[i] = -1;
  }

  //Timer1.initialize(25000);         // initialize timer1, and set a 1/2 second period
  //Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  //digitalWrite(9, LOW);
  //mfrc522_1.PCD_Init();   // Init MFRC522
  //mfrc522_1.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details
  //digitalWrite(9, HIGH);
  //Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));

  //Timer1.initialize(500000);         // initialize timer1, and set a 1/2 second period
  //Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  //Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  //delay(1000);
}


bool checkDev(byte ssNum)
{
  bool ret = false;
  mfrc522.PCD_Init(ssNum, UINT8_MAX);   // Init MFRC522
  //mfrc522_0.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details
  byte v = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
  //Serial.print(ssNum);
  //Serial.print(":");
  if (v != 0x92) {
    /*Serial.print(millis());
      Serial.print(" ");
      Serial.print(ssNum);
      Serial.print(": ");
      Serial.print(v);
      Serial.print(" ");
      Serial.println(F("WARNING: Communication failure"));*/
    ret = false;
  }
  else {
    ret = true;
  }
  return ret;
}

int ledInd=0;
void loop() {
  for (int i = 0; i < DEV_NUM; i++) {
    bool bOn = checkDev(ssPinNum[i]);
    if (bOn == true) {
      if (bDevEna[i] == false) {
        bDevEna[i] = true;
        Serial.print(i, HEX);
        Serial.println(":on");
      }
            
      if (mfrc522.PICC_IsNewCardPresent()) {
        if (mfrc522.PICC_ReadCardSerial()) {
          //Serial.print(millis());
          //Serial.print(" ");
          Serial.print(i, HEX);
          Serial.print(":");
          printUid(&(mfrc522.uid));
          
          if(cardFirstTimeChecked[i] == -1){
            cardFirstTimeChecked[i] = millis();            
            ledState[i] = blink_slow;
          }
          else if((millis() - cardFirstTimeChecked[i])>2000){
            ledState[i] = blink_fast;            
          }
        }
      }
      else {
        ledState[i] = ena;
        cardFirstTimeChecked[i] = -1;
      }
    }
    else {
      ledState[i] = disable;
      if (bDevEna[i] == true) {
        bDevEna[i] = false;
        Serial.print(i, HEX);
        Serial.println(":off");
      }
    }
    processLeds();
  }
}

void printUid(MFRC522::Uid *uid)
{
  //Serial.print(F("Card UID:"));
  for (byte i = 0; i < uid->size; i++) {
    if (uid->uidByte[i] < 0x10)
      Serial.print(F("0"));
    //Serial.print(F(" 0"));
    //else
    //  Serial.print(F(" "));
    Serial.print(uid->uidByte[i], HEX);
  }
  Serial.println();
}

void processLeds()
{     
  int leds = 0;
  for(int i=0; i<DEV_NUM; i++){       
    switch(ledState[i]){
      case disable: bLedEna[i] = false; break;
      case ena:     bLedEna[i] = true;  break;
      case blink_slow:
        if( (millis() - lastEnaTime[i]) > 500 ){
          lastEnaTime[i] = millis(); 
          if(bLedEna[i]) bLedEna[i] = false;
          else bLedEna[i] = true;        
        }
      break;
      case blink_fast:
        if( (millis() - lastEnaTime[i]) > 100 ){
          lastEnaTime[i] = millis(); 
          if(bLedEna[i]) bLedEna[i] = false;
          else bLedEna[i] = true;        
        }
        break;
    }
    //digitalWrite(ledPinNump[i], bLedEna[i]);
     if(bLedEna[i]==true)
        leds |= (1<<i);
  }

  double L = pow(2, ledInd++); // вычисляем активный светодиод
  //leds = round(L); // округляем число до целого
//  leds = 0x0;
//  Serial.print("leds=");
//  Serial.print(leds, HEX);
//  Serial.println();
//  leds <<=1;
//  if(leds>256)
//    leds = 1;
  //leds = (1<<iNum);
  SPI.transfer(leds);
  digitalWrite(RCK_PIN, HIGH);
  delay(1);
  digitalWrite(RCK_PIN, LOW);
}


