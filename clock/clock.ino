//Code by Albert Kravcov
//==================================================================================================================
// TODOS
// - WIFI Module Sync (done)
// - EEPROM support() & menus for Animation() + Color()
// - RTC battery alarm()
// - Alarm setting support()
// - Microphone support()
// - Radio support()



//Controll & Function notes
//UP  >         Toggle colors (00-09) (EEPROM). Mic function adds additional color preset (10)?
//UP long >     Set Clock Animation OFF/1min/10min (EEPROM) "CA:00/01/02"
//Center >      Alarm ON/OFF? (EEPROM) Alarm Indication??? "AL:00/01"
//Center long > Set the time (+Alarm? EEPROM?)
//Down >        Toggle pages (three pages with dht, two without)
//Down long >   Set Pages scrolling ON/OFF (EEPROM) "PS:00/01"

//Needed digits
//An: >ClockAnimation
//Al: >Alarm
//PS: >PageScroll


//>> More charakters needed "A" "n" "P" "S" "l"


//Returned Button codes
// 1 = Up
// 2 = Center
// 3 = Down
// 4 = Up LONG
// 5 = Center LONG
// 6 = Down LONG




//OPTIONAL USER CONFIG START
//==================================================================================================================
#define DEBUG        //SERIAL OUTPUT

#define LightSensor   //An external photo-resistor can automatically adjust brightness of the clock
//#define DHTsensor     //The DHT sensor will show temperature and humidity data > CONFIGURE SENSOR BELLOW!
#define WIFI          //ESP8266 S01 Module can sync time over WIFI
#define OLED          //OLED SCEEN < WORK IN PROGRESS

//#define RAD           //TODO: RADIO MODULE - Requres OLED & IRCONTROL option
//#define AudioSensor   //TODO: uncomment if you are using an microphone, adds additional animation mode controlled by sound
//==================================================================================================================

#ifdef DHTsensor
#include <DHT.h>            //"Adafruit DHT" + "Ardafruit Unified Sensor" libraries required;
#define DHTTYPE DHT22       // DHT 22   (AM2302), AM2321
#endif
//#define Temp_F              //Teperature will be converted from C to F
#ifdef WIFI
int UTCoffset = +1;         //UTC Time offset e.g. ("1" or "-1") - Used only for WiFi-Sync
#endif
//==================================================================================================================

float tempoffset       =   0.0;  //-1 Temperature adjustment (positive or negative value) no DHT Sensor required
float humidityoffset   =   0.0;   //+1 Humidity adjustment (positive or negative value) only with DHT sensor
//==================================================================================================================
//USER CONFIG END
//==================================================================================================================

#ifdef RAD
#include <radio.h>
#include <TEA5767.h>
#define FIX_BAND RADIO_BAND_FM
#define FIX_STATION 8930
TEA5767 radio;    // Create an instance of Class for Si4703 Chip
#endif

#ifdef OLED
#include <U8g2lib.h>
#endif
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h> //Adafruit NeoPixel
#include "RTClib.h"
RTC_DS3231 rtc;

#ifdef OLED

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_96X16_ER_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // EastRising 0.69" OLED
//U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED

#endif

#ifdef WIFI
//#include <SoftwareSerial.h> //testing only
#include <ICSC.h>
//SoftwareSerial mySerial(11, 10); // RX, TX > UNCOMMENT IF YOU HARDWARE HAS A SECOND SERIAL

//ICSC icsc(mySerial, 'B');  //Use softserial (breaks the LEDs on Nano)
ICSC icsc(Serial, 'B');  //MAIN UART
//ICSC icsc(Serial1, 'B');  //Use port 1 on a Leonardo or ATmega644p

byte newhour = 0;
unsigned long wifiPacket = 0; //packets
unsigned long oldwifiPacket = 0; //packets
byte Packetdelta = 10; //
uint8_t wifiHour = 0; //hours
uint8_t wifiMinute = 0; //minutes
uint8_t wifiSecond = 0; //seconds
uint8_t wifiWeekDay = 0; //weekday
uint8_t wifiDD = 0; //day
uint8_t wifiMM = 0; //month
uint16_t wifiYYYY = 0; //year
uint8_t wifiState = 0; //module status


struct dataStruct {
  unsigned long Packet;
  uint8_t hh; //HH hours
  uint8_t mm; //MM minutes
  uint8_t ss; //SS seconds
  uint8_t WeekDay; //day of the week
  uint8_t DD; //DD day
  uint8_t MM; //MM month
  uint16_t YY; //YYYY year
  uint8_t ModuleStatus; //Status (normaly 2)
  uint8_t cond; //xx 5
  uint8_t temp_max; //xx 30
  uint8_t temp_min; //xx 10
};

#endif

//#define serial //Debug data over Serial


#define LEDPIN          3   //was 3
#define IRPIN           2   //was 3
#define bt_up           4   //Button up
#define bt_set          5   //Button set
#define bt_dwn          6   //Button down
#define DHTPIN          7   //AUX2 pad (DHT22 out) 
#define buzzer          9   //Speaker
#define vbat            A0  //RTC Battery Monitoring //can be left
#define lightsens       A1  //Photo Resistor 
//#define mic             A2  //AUX1 pad - Microphone //dont have to be analog
#define colorADDR       1   // EEPROM Adress
#define animationADDR   2   // EEPROM Adress
#define alarmStateADDR  3   // EEPROM Adress
#define alarmHourADDR   4   // EEPROM Adress
#define alarmMinuteADDR 5   // EEPROM Adress

#define NUMPIXELS       60
#define longpresstime   500 // in ms

byte wifimodule = 0;
byte popup = 0;
byte animationsetting = 1;  //0 off; 1 every minute; 2 every 10 minutes
byte page = 0;
byte menu = 0;
byte menustep = 0;
byte pressedbut = 0;
byte i_butt = 0;
byte number_hour1 = 0;
byte number_hour2 = 0;
byte number_min1 = 0;
byte number_min2 = 0;
byte animateflag = 0;
byte digitbuffer = 0;
byte dot = 0;
byte newhours;
byte newminutes;
int lightvalue;
byte tempsamplecount = 0;
float buffertemp = 0;
float bufferhum = 0;
float newtemp = 0;
float newhum = 0;
int16_t digittemp = 0;
int16_t digithum = 0;
char daysOfTheWeek[7][12] = {"SO", "MO", "DI", "MI", "DO", "FR", "SA"};


byte brightness = 0;
byte old_brightness = 0;
byte volume = 0;
byte old_volume = 0;
byte colorset = 0;
byte old_colorset = 5;


unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long previousAniMillis = 0;  // Animation Timer
unsigned long previousUpdateMillis = 0;  // Sensor Update Timer
unsigned long previousTimeoutMillis = 0;  // Timeout Timer


const long interval = 1000; // interval at which to blink (milliseconds)
const long animinterval = 100;  // interval at which to blink (milliseconds)

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);
#ifdef DHTsensor
DHT dht(DHTPIN, DHTTYPE);
#endif




void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  //  pinMode(vbat, INPUT);           //RTC Battery voltage
  pinMode(buzzer, OUTPUT);        //SPEAKER
  pinMode(bt_up, INPUT_PULLUP);   //BUTTON
  pinMode(bt_set, INPUT_PULLUP);  //BUTTON
  pinMode(bt_dwn, INPUT_PULLUP);  //BUTTON

  pixels.begin(); // This initializes the NeoPixel library.
  wipe(); // wipes the LED buffers


#ifdef LightSensor
  pinMode(lightsens, INPUT);      //PHOTO SENSOR
#endif
#ifdef DHTsensor
  pinMode(DHTPIN, INPUT);         //DHT SENSOR
  dht.begin();
#endif
#ifdef AudioSensor
  pinMode(mic, INPUT);            //AUDIO SENSOR
#endif


#ifdef WIFI
  //mySerial.begin(115200);
  //Serial1.begin(115200); Use port 1 on a Leonardo or ATmega644p
  
  icsc.begin();
  icsc.registerCommand('U', &wifiupdate);
#endif

#ifdef OLED
  u8g2.begin();
#endif

  //RTC.begin(); //only for non AVR Boards
  RTC_DS3231 rtc;
  //setSyncProvider(RTC.get);   // the function to get the time from the RTC

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  

  //Get data from EEPROM
  colorset = EEPROM.read(colorADDR);
  if (colorset > 9) {
    colorset = 0;
  }
#ifdef OLED
  u8g2.begin();
#endif
  tone(buzzer, 500, 50);

#ifdef RAD
  radio.init();
  radio.setBandFrequency(FIX_BAND, FIX_STATION); // hr3 nearby Frankfurt in Germany
  radio.setVolume(volume);
  radio.setMono(false);
#endif



}





//BUTTON READING
//==================================================================================================================
byte buttoncheck()
{
  int i_butt = 0;
  byte buttonz = 0;
  if (digitalRead(bt_up) != 1) {
    while (digitalRead(bt_up) != 1) {
      delay(2);
      i_butt++;
    }
    buttonz = 1; //Up pressed

    if (i_butt > (longpresstime)) {
      buttonz = 4; //Button up pressed long
      delay(2);
    }
  }

  else if (digitalRead(bt_set) != 1) {
    while (digitalRead(bt_set) != 1) {
      delay(2);
      i_butt++;
    }
    buttonz = 2; //Center pressed

    if (i_butt > (longpresstime)) {
      buttonz = 5; //Button center pressed long
      delay(2);
    }

  }
  else if (digitalRead(bt_dwn) != 1) {
    while (digitalRead(bt_dwn) != 1) {
      delay(2);
      i_butt++;
    }
    buttonz = 3; //Down pressed

    if (i_butt > (longpresstime)) {
      buttonz = 6; //Button down pressed long
      delay(2);
    }
  }

  pressedbut = buttonz;
  return buttonz;

}





//MAIN LOOP
//==================================================================================================================
void loop() {

     buttoncheck();
     

     if (colorset == 0) {
     color_rainbowcycle();
     }
     else if (colorset == 1) { 
     color_rainbow(); 
     }
     else if (colorset == 2) { 
     color_cyber(); 
     }
     else if (colorset == 3) { 
     color_white(); 
     }
     else if (colorset == 4) { 
     color_pink(); 
     }
     else if (colorset == 5) { 
     color_velvet();  
     }
     else if (colorset == 6) { 
     color_red();
     }
     else if (colorset == 7) { 
     color_green();  
     }
     else if (colorset == 8) { 
     color_blue(); 
     }
     else if (colorset == 9) { 
     color_cyan();
     }
     
    


#ifdef WIFI
    icsc.process();
    checktimeout();
#endif


    if (pressedbut == 1) {
      if (colorset < 9) {
        colorset = colorset + 1;
        Serial.println(colorset);
      }
      else {
        //colorset = 0;
        Serial.println(colorset);
      }
    }
    if (pressedbut == 3) {
#ifdef DHTsensor //add humidity page
      if (page < 2) {
        page = page + 1;

        //reset temp and humidity samples
        //buffertemp = 0;
        //bufferhum = 0;
        //tempsamplecount = 0;

        Serial.println(page);
      }
#endif

#ifndef DHTsensor
      if (page < 1) {
        page = page + 1;

        //reset temp samples
        //buffertemp = 0;
        //tempsamplecount = 0; //reset temp and humidity samples

        Serial.println(page);
      }
#endif

      else {
        page = 0;
        Serial.println(page);
      }
    }
    if (pressedbut == 5) {
      //get current time for the setting menu
      DateTime now = rtc.now();
      newhours = now.hour();
      newminutes = now.minute();
      menu = 1;
      settime();
    }

    if (page == 0) {
      // check if animation is triggered
      
      if (animateflag == 0) {
        updateNumber();
      }
      else {
        animate();
      }
    }

    else if (page == 1) {
      showtemp();

    }
    else if (page == 2) {
      showhumidity();
    }

  
  OLEDdraw();
  PopUphandler();  
  getTempHum();

  //delay(100);
}





//LED CODE
//==================================================================================================================


void color_rainbow() { // modified from Adafruit example to make it a state machine
  static uint16_t j=0;
    for(int i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel((i+j) & 255));
    }
    
    mapPixels();
    setbrightness();    
    
    pixels.show();
    
     j++;
  if(j >= 256) j=0;   
}

void color_rainbowcycle() { 
  static uint16_t j=0;
    for(int i=0; i< pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
    }
    mapPixels();
    setbrightness();    
    
    pixels.show();
    
  j++;
  if(j >= 256*5) j=0;
}
void color_cyber() { 
  for (int i = 0; i < pixels.numPixels(); i++) {

        uint32_t mint = pixels.Color(0, 250, 180); //CYBER
        uint32_t green = pixels.Color(50, 250, 50);
        uint32_t yellow = pixels.Color(250, 200, 0);
        uint32_t pink = pixels.Color(255, 0, 255);
        uint32_t purple = pixels.Color(100, 0, 255);

        pixels.fill(pink, 0, 7);
        pixels.fill(purple, 7, 14);
        pixels.fill(yellow, 14, 21);
        pixels.fill(green, 21, 28);
        pixels.fill(mint, 28, 30);
      }
      mapPixels();
      setbrightness();    
     
      pixels.show();
      
}
void color_red() { 
  for (int i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 255, 0, 0); //red
      }
      mapPixels();
      setbrightness();    
    pixels.show();
}
void color_green() { 
  for (int i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 0, 255, 0); //green
      }
      mapPixels();
      setbrightness();    
    pixels.show();
}
void color_blue() { 
  for (int i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 0, 0, 255); //blue
      }
      mapPixels();
      setbrightness();    
    pixels.show();
}
void color_white() { 
  for (int i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 255, 255, 255); //white
      }
      mapPixels();
      setbrightness();    
    pixels.show();
}
void color_pink() { 
  for (int i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 255, 0, 255); //pink
      }
      mapPixels();
      setbrightness();    
    pixels.show();
}
void color_velvet() { 
  for (int i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 255, 255, 0); //velvet
      }
      mapPixels();
      setbrightness();    
    pixels.show();
}
void color_cyan() { 
  for (int i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 0, 255, 255); //cyan
      }
      mapPixels();
      setbrightness();    
    pixels.show();
}


void wipe(){ // clear all LEDs
     for(int i=0;i<pixels.numPixels();i++){
       pixels.setPixelColor(i, pixels.Color(0,0,0)); 
       
       }
}


//DIGIT MAPPINGS
//==================================================================================================================
void mapPixels() {
  //1st digit definitions number_min2 (counting from right)
  if (number_min2 == 0) {
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 1) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 2) {
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 3) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 4) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 5) {
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 6) {
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 7) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 8) {
  }
  else if (number_min2 == 9) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 10) { //all off code
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 11) { //"c" code
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
  }

  else if (number_min2 == 12) { //"C" code
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
  }

  else if (number_min2 == 13) { //"o up" code
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 14) { //"o down" code
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
  }
  else if (number_min2 == 15) { //"dash" code
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
  }



  //2nd digit definitions number_min1 (counting from right)
  if (number_min1 == 0) {
    pixels.setPixelColor(3 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 1) {
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 2) {
    pixels.setPixelColor(2 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 3) {
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 4) {
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 5) {
    pixels.setPixelColor(6 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 6) {
    pixels.setPixelColor(6 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 7) {
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 8) {
  }
  else if (number_min1 == 9) {
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 13) { //"o up" code
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 14) { //"o up" code
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 15) { //"dash" code
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 7, pixels.Color(0, 0, 0));
  }

  //3rd digit definitions number_hour2 (counting from right)
  if (number_hour2 == 0) {
    pixels.setPixelColor(3 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 1) {
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 2) {
    pixels.setPixelColor(2 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 3) {
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 4) {
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 5) {
    pixels.setPixelColor(6 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 6) {
    pixels.setPixelColor(6 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 7) {
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 8) {
  }
  else if (number_hour2 == 9) {
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 15) { //"dash" code
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 14, pixels.Color(0, 0, 0));
  }

  //4th digit definitions number_hour1 (counting from right)
  if (number_hour1 == 0) {
    pixels.setPixelColor(3 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 1) {
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 2) {
    pixels.setPixelColor(2 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 3) {
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 4) {
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 5) {
    pixels.setPixelColor(6 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 6) {
    pixels.setPixelColor(6 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 7) {
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 8) {
  }
  else if (number_hour1 == 9) {
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 10) { //all off code
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 21, pixels.Color(0, 0, 0));
  }

  else if (number_hour1 == 11) { //minus code
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 15) { //"dash" code
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 21, pixels.Color(0, 0, 0));
  }

  //dots
  if (page == 0) {
    if (dot == 0) {
      pixels.setPixelColor(28 , pixels.Color(0, 0, 0));
      pixels.setPixelColor(29 , pixels.Color(0, 0, 0));
    }
    else {}
  }
  else if (page == 1) {

#ifdef DHTsensor
    pixels.setPixelColor(28 , pixels.Color(0, 0, 0));
    pixels.setPixelColor(29 , pixels.Color(0, 0, 0));
#endif

#ifndef DHTsensor
    pixels.setPixelColor(29 , pixels.Color(0, 0, 0));

#endif

  }
  else if (page == 2) {
    pixels.setPixelColor(28 , pixels.Color(0, 0, 0));
    pixels.setPixelColor(29 , pixels.Color(0, 0, 0));
  }

}





//BRIGHTNESS CONTROL
//==================================================================================================================
void setbrightness() {

#ifdef LightSensor
  lightvalue = analogRead(lightsens); //Photo-Resistor

  // Serial.print("Photosensor: ");
  // Serial.println(lightvalue);

  if (lightvalue < 40) { //value for dark
    lightvalue = 40;
  }
  else if (lightvalue > 1000) { //value for bright
    lightvalue = 1000;
  }

  lightvalue = map(lightvalue, 40, 1000, 50, 255);
  pixels.setBrightness(lightvalue); //regulates the brightness of the whole strip
  #ifdef OLED
  u8g2.setContrast(lightvalue); //set brightness of the OLED
  #endif
#endif
#ifndef LightSensor
  pixels.setBrightness(255); //max brightness if "LightSensor" not defined
#endif
}





//CLOCK FUNCTIONS getting time from RTC
//==================================================================================================================
void updateNumber() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //tmElements_t tm;
    //RTC.read(tm);
    //Serial.print("RTC READ:");
    //Serial.print(tm.Hour, DEC);
    //Serial.print(':');
    //Serial.print(tm.Minute,DEC);
    //Serial.print(':');
    //Serial.println(tm.Second,DEC);


    DateTime now = rtc.now();
    
    
    number_hour2 = (now.hour() % 10); //last digit of the hours
    

    if (now.hour() < 10) {
      number_hour1 = 0;
    }
    else {
      number_hour1 = (now.hour() / 10U) % 10; //first digit of the hours
    }

    number_min2 = (now.minute() % 10); //last digit of the seconds


    if (now.minute() < 10) {
      number_min1 = 0;
    }
    else {
      number_min1 = (now.minute() / 10U) % 10; //first digit of the seconds
    }


    //Serial.print("SYSTEM TIME: ");
    //Serial.print(hour());
    //Serial.print(":");
    //Serial.print(minute());
    //Serial.print(":");
    //Serial.println(second());


    if (animationsetting == 1) {
      //animation every minute
      if (number_min2 != digitbuffer) {
        animateflag = 1;
        //number_min1 = 0;
        number_min2 = 0;
        //number_hour1 = 0;
        //number_hour2 = 0;
      }
      else {
        animateflag = 0;
      }
    }

    else if (animationsetting == 2) {
      //animation every ten minutes
      if (number_min1 != digitbuffer) {
        animateflag = 1;
        number_min1 = 0;
        //number_min2 = 0;
        //number_hour1 = 0;
        //number_hour2 = 0;
      }
      else {
        animateflag = 0;
      }
    }
    else {    }


    //Dot animation
    if (dot == 0) {
      dot = 1;
    }
    else {
      dot = 0;
    }

  }

}



void getTempHum() {

  unsigned long currentUpdateMillis = millis();

  if (currentUpdateMillis - previousUpdateMillis >= 1000) { //was1000
    previousUpdateMillis = currentUpdateMillis;

    if (tempsamplecount < 5) {

#ifndef DHTsensor
      int16_t rtcTemp = rtc.getTemperature();
      float c = rtcTemp;
      //float c = rtcTemp / 4.; //convert to celsius

#ifdef Temp_F
      c = c * 9.0 / 5.0 + 32.0; //convert to fahrenheit
#endif

#endif

#ifdef DHTsensor
      float c = dht.readTemperature(); //temp in celsius
      float h = dht.readHumidity();

#ifdef Temp_F
      c = c * 9.0 / 5.0 + 32.0; //convert to fahrenheit
#endif

      if (isnan(c)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        //return;
      }

      //Serial.print(F("Sensor Humidity: "));
      //Serial.println(h);
#endif

      //Serial.print(F("Sensor Temperature: "));
      //Serial.println(c);


      //apply user temperature offset
      c = c + tempoffset;
      buffertemp += c;

#ifdef DHTsensor
      //apply user humidity offset
      h = h + humidityoffset;
      bufferhum += h;
#endif

      tempsamplecount++;
      //Serial.println(tempsamplecount);
    }

    //get the avarege of the five measurements

    else if (tempsamplecount == 5) {
      buffertemp = buffertemp / 5;
      Serial.print("AVG Temp:");
      Serial.println(buffertemp);
      newtemp = buffertemp;
      digittemp = newtemp * 10;

      buffertemp = 0; //reset buffer


#ifdef DHTsensor
      bufferhum = bufferhum / 5;
      Serial.print("AVG Hum:");
      Serial.println(bufferhum);
      newhum = bufferhum;
      digithum = newhum * 10;

      bufferhum = 0; //reset buffer
#endif

      tempsamplecount = 0; //reset the sample counter


    }




  }
  
return;

}





//TEMPERATURE READOUT (RTC & DHT)
//==================================================================================================================
void showtemp() {


  //draw a "dash" while waiting for data first time
  if (digittemp == 0) {
    number_hour1 = 15; //dash
    number_hour2 = 15; //dash
    number_min1 = 13; //code for c
    number_min2 = 11; //code for "°"

    //Serial.println("No avg. data yet, waiting…");
  }
  else {
    number_hour1 = (digittemp / 100U) % 10; //first digit of temp
    number_hour2 = (digittemp / 10U) % 10; //second digit of temp
    number_min1 = 13; //code for c
    number_min2 = 11; //code for "°"
  }

}





//HUMIDITY READOUT (DHT required)
//==================================================================================================================
void showhumidity() {

  //draw a "dash" while waiting for data first time
  if (newhum == 0) {
    number_hour1 = 15; //dash
    number_hour2 = 15; //dash
    number_min1 = 14; //code for "°"  %
    number_min2 = 14; //code for "°"  %

    Serial.println("No avg. data yet, waiting…");
  }
  else {
    number_hour1 = (digithum / 100U) % 10; //first digit of temp
    number_hour2 = (digithum / 10U) % 10; //second digit of temp
    number_min1 = 14; //code for "°"  %
    number_min2 = 14; //code for "°"  %
  }
}







//SET TIME FUNCTION
//==================================================================================================================
void settime() {

  Serial.println("entered menu");

  while (menu == 1) {
    buttoncheck();

    uint32_t off = pixels.Color(0, 0, 0);
    uint32_t white = pixels.Color(255, 255, 255);

    pixels.fill(white, 0, 29);

    //display digits
    number_hour2 = (newhours % 10); //last digit of the seconds
    if (newhours < 10) {
      number_hour1 = 0;
    }
    else {
      number_hour1 = (newhours / 10U) % 10; //first digit of the seconds
    }

    number_min2 = (newminutes % 10); //last digit of the seconds
    if (newminutes < 10) {
      number_min1 = 0;
    }
    else {
      number_min1 = (newminutes / 10U) % 10; //first digit of the seconds
    }

    //time set routine:

    if (menustep == 0) {
      // Serial.print("New Hours: ");
      // Serial.println(newhours);
      pixels.fill(off, 0, 14); // minutes
      pixels.fill(off, 28, 29); //dots

      //edit hours
      if (pressedbut == 2) {
        menustep = 1;
      }
      if (pressedbut == 1) {
        if (newhours < 23) {
          newhours += 1;
        }
        else {
          newhours = 0;
        }
      }
      if (pressedbut == 3) {
        if (newhours > 0) {
          newhours -= 1;
        }
        else {
          newhours = 23;
        }
      }
    }
    else if (menustep == 1) {
      // Serial.print("New Minutes: ");
      // Serial.println(newminutes);
      pixels.fill(off, 14, 30); // houts and dots

      //edit minutes
      if (pressedbut == 2) {
        menustep = 0;
      }
      if (pressedbut == 1) {
        if (newminutes < 59) {
          newminutes += 1;
        }
        else {
          newminutes = 0;
        }
      }
      if (pressedbut == 3) {
        if (newminutes > 0) {
          newminutes -= 1;
        }
        else {
          newminutes = 59;
        }
      }
    }

    mapPixels();
    setbrightness();
    pixels.show();


    if (pressedbut == 5) {
      
      //FIXME set date
      rtc.adjust(DateTime(2020, 12, 30, newhours, newminutes, 0));
      DateTime now = rtc.now();
      
      //setTime(newhours, newminutes, 0, 30, 12, 2020); //rtc.setTime(byte hours, byte minutes, byte seconds)
      //RTC.set(now());
      Serial.println("new time saved");
      menu = 0;
      return;
    }
  }
  //exit;

}





//ROLLING NUMBER ANIMATION
//==================================================================================================================
void animate() {

  if (animateflag == 1) {
    unsigned long currentAniMillis = millis();

    if (currentAniMillis - previousAniMillis >= animinterval) {
      previousAniMillis = currentAniMillis;

      //all digits
      number_min2 = number_min2 + 1;
      number_min1 = number_min1 + 1;
      number_hour2 = number_hour2 + 1;
      number_hour1 = number_hour1 + 1;

      if (number_hour1 > 9) {
        number_hour1 = 0;
      }
      if (number_hour2 > 9) {
        number_hour2 = 0;
      }
      if (number_min1 > 9) {
        number_min1 = 0;
      }
      delay(20); //test

      if (number_min2 == 10) {

        //animation every minute
        if (animationsetting == 1) {
          DateTime now = rtc.now();
          number_min2 = (now.minute() % 10); //second digit of the seconds
          digitbuffer = number_min2;
        }

        //animation every 10 minutes
        else if (animationsetting == 2) {
          DateTime now = rtc.now();
          number_min1 = (now.minute() / 10U) % 10;  //first digit of the seconds
          digitbuffer = number_min1;
        }

        // Serial.println(digitbuffer);
        // Serial.println(number_min2);
        animateflag = 0;
      }
    }
  }

  else {
    return;
    //exit;
  }

}





//Format digits for SERIAL
//==================================================================================================================
void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(':');
  if (digits < 10) {
    Serial.print('0');
    Serial.print(digits);
  }
}





//COLORWHEEL
//==================================================================================================================
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

#ifdef OLED
void PopUphandler() {

  if (colorset != old_colorset) {
    popup = 1;
    Serial.print("color-pop triggered:");
    old_colorset = colorset;
    colorpopup();
    delay(1000);
    Serial.println(old_colorset);
    popup = 0;
  }

  if (brightness != old_brightness) {
    Serial.println("brightness-pop triggered");
    //popup = 1;
    //brightnesspopup();
    //delay(1000);
    //old_brightness = brightness;
    //popup = 0;
  }

  if (volume != old_volume) {
    Serial.println("volume-pop triggered");
    popup = 1;
    volumepopup();
    delay(1000);
    old_volume = volume;
    popup = 0;
  }

  else {
    
    OLEDdraw();
  }
}



void colorpopup() {
  u8g2.firstPage();
  do {

    if (popup == 1) {
      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.setCursor(0, 24);
      u8g2.print("NEW COLOR:");
      u8g2.print(colorset);
    }

  } while ( u8g2.nextPage() );
}

void brightnesspopup() {
  u8g2.firstPage();
  do {

    if (popup == 1) {
      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.setCursor(0, 24);
      u8g2.print("NEW BRIGHT:");
      u8g2.print(brightness);
    }

  } while ( u8g2.nextPage() );
}


void volumepopup() {
  u8g2.firstPage();
  do {

    if (popup == 1) {
      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.setCursor(0, 24);
      u8g2.print("NEW VOLUME:");
      u8g2.print(volume);
    }

  } while ( u8g2.nextPage() );
}

void OLEDdraw() {

  //Serial.print("wifiState:");
  //Serial.println(wifiState);
  //Serial.print("module:");
  //Serial.println(wifimodule);


  u8g2.firstPage();
  
  
  do {

      DateTime now = rtc.now();

    if (wifimodule == 0) {
      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.setCursor(0, 24);
      u8g2.print("RTC");

      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.setCursor(0, 40);
      if (now.hour() <= 9) {
        u8g2.print("0");
      }
      u8g2.print(now.hour());
      u8g2.print(":");
      if (now.minute() <= 9) {
        u8g2.print("0");
      }
      u8g2.print(now.minute());
      u8g2.print(":");
      if (now.second() <= 9) {
        u8g2.print("0");
      }
      u8g2.print(now.second());

      u8g2.setCursor(0, 55);
      u8g2.print(daysOfTheWeek[now.dayOfTheWeek()]);
      u8g2.print(" ");
      u8g2.print(now.day());
      u8g2.print(".");
      u8g2.print(now.month());
      u8g2.print(".");
      u8g2.print(now.year());

      u8g2.setCursor(65, 24);
      u8g2.print("T:");
      u8g2.print(newtemp, 1);

#ifndef Temp_F
      u8g2.print(" C");
#endif
#ifdef Temp_F
      u8g2.print(" F");
#endif

#ifdef DHTsensor
      u8g2.setCursor(65, 40);
      u8g2.print("H:");
      u8g2.print(newhum, 0);
      u8g2.print(" %");
#endif
    }

    else if (wifimodule == 1) { //IF WE HAVE A WI FI MODULE

#ifdef WIFI




      if (wifiState == 2) {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(0, 24);
        u8g2.print("WiFi");

        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(0, 40);
        if (wifiHour <= 9) {
          u8g2.print("0");
        }
        u8g2.print(wifiHour);
        u8g2.print(":");
        if (wifiMinute <= 9) {
          u8g2.print("0");
        }
        u8g2.print(wifiMinute);
        u8g2.print(":");
        if (wifiSecond <= 9) {
          u8g2.print("0");
        }
        u8g2.print(wifiSecond);

        u8g2.setCursor(0, 55);
        if (wifiWeekDay == 1) {
          u8g2.print("MO ");
        }
        else if (wifiWeekDay == 2) {
          u8g2.print("DI ");
        }
        else if (wifiWeekDay == 3) {
          u8g2.print("MI ");
        }
        else if (wifiWeekDay == 4) {
          u8g2.print("DO ");
        }
        else if (wifiWeekDay == 5) {
          u8g2.println("FR ");
        }
        else if (wifiWeekDay == 6) {
          u8g2.print("SA ");
        }
        else if (wifiWeekDay == 7) {
          u8g2.print("SO ");
        }
        u8g2.print(wifiDD);
        u8g2.print(".");
        u8g2.print(wifiMM);
        u8g2.print(".");
        u8g2.print(wifiYYYY);

        u8g2.setCursor(65, 24);
        u8g2.print("T:");
        u8g2.print(newtemp, 1);

#ifndef Temp_F
        u8g2.print(" C");
#endif
#ifdef Temp_F
        u8g2.print(" F");
#endif

#ifdef DHTsensor
        u8g2.setCursor(65, 40);
        u8g2.print("Hm:");
        u8g2.print(newhum, 0);
        u8g2.print(" %");
#endif
      }

      else if (wifiState == 1)  {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(0, 24);
        u8g2.print("WIFI OFFLINE");
      }
      else if (wifiState == 0) {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.setCursor(0, 24);
        u8g2.print("WIFI FOUND");
      }


#endif
    }



  } while ( u8g2.nextPage() );



}
#endif

#ifdef WIFI
void wifiupdate(unsigned char src, char command, unsigned char len, char *data) {

  wifimodule = 1; //module checked in

  struct dataStruct *myData = (struct dataStruct *)data;

  if ((myData->hh == 0) && (myData->mm == 0) && (myData->ss == 0)) {
    wifiState = 0; //concider offline yet
    Serial.println("Found a module but Data not valid yet");
  }
  else {


    wifiPacket = myData->Packet;
    wifiHour = myData->hh;
    wifiHour = wifiHour + UTCoffset; //applying user offset
    wifiMinute = myData->mm;
    wifiSecond = myData->ss;
    wifiWeekDay = myData->WeekDay;
    wifiDD = myData->DD;
    wifiMM = myData->MM;
    wifiYYYY = myData->YY;
    wifiState = myData->ModuleStatus;

    oldwifiPacket = wifiPacket;


    if ((newhour != wifiHour) && (wifiYYYY != 0)) { //Set RTC every HOUR if it has data
      
      //set the RTC:
      
      rtc.adjust(DateTime(wifiYYYY, wifiMM, wifiDD, wifiHour, wifiMinute, wifiSecond));
      DateTime now = rtc.now();

      Serial.print("RTC SET TO WIFI-TIME:");
      Serial.print(wifiHour); //Hours
      Serial.print(":");
      Serial.print(wifiMinute); //Minutes
      Serial.print(":");
      Serial.println(wifiSecond); //Seconds

      newhour = wifiHour;
      tone(buzzer, 500, 500);
    }

    return;
  }
}

void checktimeout() {

  if (wifimodule == 1) {
    unsigned long currentTimeoutMillis = millis();

    if (currentTimeoutMillis - previousTimeoutMillis >= 1000) {
      previousTimeoutMillis = currentTimeoutMillis;

      oldwifiPacket = oldwifiPacket + 1; //increase the count each run

      if (oldwifiPacket > 6) { //dont start right away
        Packetdelta = oldwifiPacket - wifiPacket;

        if (Packetdelta > 6) { //if missed 5 packets
          wifimodule = 0;
          wifiState = 0;
          oldwifiPacket = 0;
          Serial.println("WIFI MODULE DISCONNECTED:");
          
          //Reset WIFI packet data
          wifiPacket = 0; 
          wifiHour = 0; 
          wifiMinute = 0;
          wifiSecond = 0;
          wifiWeekDay = 0; 
          wifiDD = 0; 
          wifiMM = 0; 
          wifiYYYY = 0;
          newhour = 0;
        }
        else if (Packetdelta > 50) {
          Packetdelta = 10;
        }

        else {
          wifimodule = 1;
        }

      }
    }
    //Serial.print("Packetdelta:");
    //Serial.println(Packetdelta);
  }



}


#endif
