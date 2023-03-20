//IDE HARDWARE PACKAGE:
//ATTinyCore by Spence Konde v 1.5.2


// attiny core 841 (no bootloader) or attiny core 441 (no bootloader)  
// 8mhz, slave only
// "<" less than 4.5v
// counterclockwise pin mapping
//slave only mode

#define APPLECLONE //if using a clone remote
//#define DEBUG //Serial output @9600

/*
   Light Weight NEC Infrared decoder using interrupt instead of timer.
   04_06_2019 v0_05 namespace model
*/
#include "IrNec.h"
#include <Wire.h>

// Set to the RECV_PIN number of an external interrupt pin and connect the IR receiver to it.
// Uno/Nano etc. : pin2 or pin3
// ATTINY841 : pin1 (PB1)
// ESP8266 : Any GPIO pin apart from GPIO16
// see attachInterrupt() for other examples and more details

//ATTinyCore is working fine

//A1 RTC; AND A2 MIC 

const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

#define RECV_PIN        1  //only one INT0 pin on ATtiny85 
#define STATE_LED_PIN   3 //4 on attiny/13 on nano
#define AIN1_PIN        A1 //pa1 RTC BATTERY
//#define AIN2_PIN        A0 //pa0
#define AIN3_PIN        A2 //pa2

unsigned long key_value = 0;



int myData[4]; //4x 2byte > 8byte

int val1 = 0;
//int val2 = 0;
int smooth_val1 = 0;
int smooth_val2 = 0;
int sound = 0;

unsigned long previousMillis = 0;        // will store last time LED was updated
byte probecount = 0;

int IRcode = 0;
int BTrem = 0;
int state;
byte gotvolts = 0;

#define updatetime 100 //100ms per probe
#define getvoltstime 60000 //1min per probe



void setup() {

  pinMode(STATE_LED_PIN, OUTPUT);
  pinMode(AIN1_PIN, INPUT); //RTC
  //pinMode(AIN2_PIN, INPUT);
  pinMode(AIN3_PIN, INPUT); //MIC

  digitalWrite(STATE_LED_PIN, HIGH); //Build-in LED
  delay(1000); //wait for MASTER boot

  digitalWrite(STATE_LED_PIN, LOW); //Build-in LED

#ifdef DEBUG
  Serial.begin(9600) ;
  Serial.println(F("IR v0_01")) ;
#endif
  nsIrNec::begin(RECV_PIN) ;  // MUST BE EXTERNAL INTERRUPT PIN

  Wire.begin(0x08);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // fucntion to run when asking for data
  Wire.flush();
}

void loop() {


  //////// READING ADC VOLTAGE



if (gotvolts == 1) {

  unsigned long currentMillis = millis();
  if (abs(currentMillis - previousMillis) > getvoltstime) {

  gotvolts = 0;
  previousMillis = currentMillis;
  }

 }

 else {
  
  

  unsigned long currentMillis = millis();
  if (abs(currentMillis - previousMillis) > updatetime) {

    if (probecount < 5) { //100ms steps

      int aRead1 = analogRead(AIN1_PIN);
      val1 = val1 + aRead1;
      //int aRead2 = analogRead(AIN2_PIN);
      //val2 = val2 + aRead2;
      probecount++;
       digitalWrite(STATE_LED_PIN, HIGH); //visualising probing

    }
    else if (probecount == 5) { //500ms update rate
      smooth_val1 = val1 / 5;
      val1 = 0;
      gotvolts = 1;
      //smooth_val2 = val2 / 5;
      //val2 = 0;
      
    
      probecount = 0; //reset probes
      
    }
    previousMillis = currentMillis;
  }


}


  ///////// READING SOUND LEVELS



  unsigned long startMillis = millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;


  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(AIN3_PIN);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  //double volts = (peakToPeak * 5.0) / 1024;  // convert to volts
  //  sound = volts * 100; //covert float to integer
  sound = peakToPeak; //covert float to integer
//sound = analogRead(AIN3_PIN)


  ///////// READING IR


  nsIrNec::loop() ;   // check for new data

  if ( nsIrNec::dataOut != 0 ) {

    IRcode = nsIrNec::dataOut;

    if (nsIrNec::dataOut == 0XFFFFFFFF) {
      nsIrNec::dataOut = key_value;
    }

    //----
#ifndef APPLECLONE

    else if (nsIrNec::dataOut == 0x77E1400C) {
#ifdef DEBUG
      Serial.println("menu");
#endif

      //1
      BTrem = 1;
    }

    else if (nsIrNec::dataOut == 0x77E17A0C) {
#ifdef DEBUG
      Serial.println("play");
#endif

      //2
      BTrem = 2;
    }

    else if (nsIrNec::dataOut == 0x77E1BA0C) {
#ifdef DEBUG
      Serial.println("center");
#endif

      //3
      BTrem = 3;

    }

    else if (nsIrNec::dataOut == 0x77E1B00C) {
#ifdef DEBUG
      Serial.println("down");
#endif

      //4
      BTrem = 4;
    }

    else if (nsIrNec::dataOut == 0x77E1D00C) {
#ifdef DEBUG
      Serial.println("up");
#endif

      //5
      BTrem = 5;
    }

    else if (nsIrNec::dataOut == 0x77E1E00C) {
#ifdef DEBUG
      Serial.println("right");
#endif

      //6
      BTrem = 6;
    }

    else if (nsIrNec::dataOut == 0x77E1100C) {
#ifdef DEBUG
      Serial.println("left");
#endif

      //7
      BTrem = 7;
    }

    digitalWrite(STATE_LED_PIN, HIGH);
    delay(5);
    //delay(50);//for testing

    key_value = nsIrNec::dataOut;
    nsIrNec::dataOut = 0 ; //clear

  }

#endif

  //------
#ifdef APPLECLONE

  else if (nsIrNec::dataOut == 0x77E1C040) {
#ifdef DEBUG
    Serial.println("menu");
#endif

    //1
    BTrem = 1;
  }

  else if (nsIrNec::dataOut == 0x77E1FA40)  {

#ifdef DEBUG
    Serial.println("play");
#endif

    //2
    BTrem = 2;
  }

  else if (nsIrNec::dataOut == 0x77E13A40) {
#ifdef DEBUG
    Serial.println("center");
#endif

    //3
    BTrem = 3;

  }

  else if (nsIrNec::dataOut == 0x77E13040) {
#ifdef DEBUG
    Serial.println("down");
#endif

    //4
    BTrem = 4;
  }

  else if (nsIrNec::dataOut == 0x77E15040) {
#ifdef DEBUG
    Serial.println("up");
#endif

    //5
    BTrem = 5;
  }

  else if (nsIrNec::dataOut == 0x77E16040) {
#ifdef DEBUG
    Serial.println("right");
#endif

    //6
    BTrem = 6;
  }

  else if (nsIrNec::dataOut == 0x77E19040) {
#ifdef DEBUG
    Serial.println("left");
#endif

    //7
    BTrem = 7;
  }

  digitalWrite(STATE_LED_PIN, HIGH);
  delay(5);
  //delay(50);//for testing

  key_value = nsIrNec::dataOut;
  nsIrNec::dataOut = 0 ; //clear

}

#endif




else {
  //8: no action
  digitalWrite(STATE_LED_PIN, LOW);

}


}

void requestEvent()
{

  myData [0] = smooth_val1; //vbat with voltage divider
  myData [1] = sound;
  myData [2] = smooth_val2;
  myData [3] = BTrem;

  Wire.write((byte *) myData, sizeof myData);

  BTrem = 0; //reset the IR button value after sending
  IRcode = 0; //reset the IR button value after sending
}
