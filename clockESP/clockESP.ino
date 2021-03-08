
//const char* ssid     = "heyhoffmann";
//const char* password = "honigkuchenpferd";

#define DEBUG //Debug data over Serial

//AP Mode is not visible on display
//wifi off popup takes longer to hide if wifi was set on prior

//TODOS:
//======================
//   x remoove 1000ms update and apply it only to dot animation for faster overall animations
//   x add WifiClient (ideally non blocking)
//    port to fastLED to remove flickering
//    finish digit animation setting (
//    finish pageslide setting
//    create a menu: m01 > set time and alarm; m02 > set pageslide on/ff; m02 > set digit animation; m03 > set brightness; m04 > mic on/off
//    make brightness manually adjustable as a setting (1-10 manual, 0 Auto)
//    change LED order 1dig, 2dig, dots, 3dig, 4dig.


//    NEW HARDWARE:
//======================
//  (ongoing)  OLED support
//    - IR support (attiny) (testboard req)
//    - ADC support over I2C (testboard req)
//     -RTC battery alarm()
//    RADIO SUPPORT (testboard req)


//======================
//    BUILD FINAL PCB!
//======================

//======================
//    BUILD FINAL CLOCK!
//======================

//    WEB UI:
//======================
//   x Enable/disable wifi
//   x sync weather and time at start once then in 10 min interval
//   x create json structure for all possible settings > location id and key missing
//    find out how to store apikey, city, location (or better KEY & cityID)
//    read json full in web-interface on load (reload if something is changed)
//    Create the UI
//    moove from SPIFFS to LittleFS



//Controll & Function notes
//UP  >        x Toggle colors (00-09) (EEPROM). Mic function adds additional color preset (10)?
//UP long >     Set Clock Animation OFF/1min/10min (EEPROM) "CA:00/01/02"
//Center >     x Alarm ON/OFF? (EEPROM) Alarm Indication???
//Center long >x Set the time (+Alarm) > later enter menu
//Down >       x Toggle pages (three pages with tempsensor, two without)
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

//I2C Adresses
//0x3C - OLED (60)
//0x68 - RTC (104)
//0x40 - Si7021 (64)
//0x08 - ATtiny (8)



//Code by Albert Kravcov
//==================================================================================================================




//OPTIONAL USER CONFIG START
//==================================================================================================================

#define AUTOROTATION    //Sliding Digits (Time > Temp > Humidity >)
#define LightSensor     //An external photo-resistor can automatically adjust brightness of the clock
#define Si7021sensor  //The Si7021 sensor will show temperature and humidity data > CONFIGURE SENSOR BELLOW!
#define OLED            //OLED SCEEN < WORK IN PROGRESS
//#define IR            //TODO: IR Remot control via i2c using an ATtiny845
//#define RAD           //TODO: RADIO MODULE - Requres OLED & IRCONTROL option
//#define AudioSensor   //TODO: uncomment if you are using an microphone, adds additional animation mode controlled by sound
//==================================================================================================================

#define RTCtemp

#ifdef Si7021sensor
#undef RTCtemp
#endif


#ifdef Si7021sensor
#include "Adafruit_Si7021.h"
Adafruit_Si7021 sensor = Adafruit_Si7021();
#endif
//==================================================================================================================

//#define Temp_F              //Teperature will be converted from C to F
float tempoffset       =   0.0;   //-1 Temperature adjustment (positive or negative value) no Si7021 Sensor required
float humidityoffset   =   0.0;   //+1 Humidity adjustment (positive or negative value) only with Si7021 sensor
int UTCoffset = +1;               //UTC Time offset in hours e.g. ("1" or "-1") - Used only for WiFi-Sync

//==================================================================================================================
//USER CONFIG END
//==================================================================================================================

//D1-SCL | D2-SDA
#define stateLED        D0   //no PWM, Alarmstate LED
#define bt_set          D3   //OK pulled up, Button
#define bt_up           D4   //OK pulled up, Button
#define LEDPIN          D5   //OK, Neopixel
#define bt_dwn          D6   //OK, Button
#define buzzer          D7   //OK, Buzzer
#define wifiLED         D8   //pulled to GND, Boot fails if pulled HIGH, WiFi state LED
#define lightsens       A0   //OK, Photo Resistor 
#define bt_wifi         RX   //HIGH at boot, WiFi ON/OFF button

//#define vbat          A0  //RTC Battery Monitoring //can be left
//#define mic           A2  //AUX1 pad - Microphone //dont have to be analog

#define colorADDR       0   // EEPROM Adress
#define animationADDR   1   // EEPROM Adress
#define alarmStateADDR  2   // EEPROM Adress
#define alarmHourADDR   3   // EEPROM Adress
#define alarmMinuteADDR 4   // EEPROM Adress
#define wifiStateADDR   5   // EEPROM Adress

#define NUMPIXELS       60
#define longpresstime   500 // in ms

#include <EEPROM.h>
#include <Wire.h>

#include "RTClib.h"
RTC_DS3231 rtc;

#include <Adafruit_NeoPixel.h> //Adafruit NeoPixel
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

#ifdef OLED
#include <U8g2lib.h>
//U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_96X16_ER_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // EastRising 0.69" OLED
//U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif

#ifdef RAD
//do stuff
#endif


byte wireArray[8] = {}; //empty array where to put the numbers comming from the slave
int micvalue = 0;
int adcvalue1 = 0;
int adcvalue2 = 0;

int poptime = 1000; //runtime timeout of the popup

byte wifion = 0;
byte old_wifion = 0;
bool serveronline = false;

int weather_now;
int weather_max;
int weather_min;
byte weather_i;

byte stepcounter = 0;
byte popup = 0;
byte animationsetting = 1;  //0 off; 1 every minute; 2 every 10 minutes
byte page = 0;
byte menu = 0;
byte menustep = 0;
byte pressedbut = 0;
byte irbutton = 0;
byte i_butt = 0;
byte number_hour1 = 0;
byte number_hour2 = 0;
byte number_min1 = 0;
byte number_min2 = 0;
byte animateflag = 0;
byte digitbuffer = 0;
byte dot = 0;

byte nowhour = 0;
byte nowminute = 0;
byte nowsecond = 0;
char* nowweekday;
byte nowday = 0;
byte nowmonth = 0;
int nowyear = 0;
byte newhours;
byte newminutes;
byte alarmhours;
byte alarmminutes;
byte newalarmminutes;
byte newalarmhours;
int lightvalue;
byte tempsamplecount = 0;
float buffertemp = 0;
float bufferhum = 0;
float newtemp = 0;
float newhum = 0;
int16_t digittemp = 0;
int16_t digithum = 0;
uint8_t wifiState = 0; //module status
byte updateflag = 1;
byte sensorupdateflag = 1;

char daysOfTheWeek[7][12] = {"SO", "MO", "DI", "MI", "DO", "FR", "SA"};

byte alarmset = 0;
byte old_alarmset;
byte alarmstate = 0;

byte brightness = 0;
byte old_brightness = 0;
byte volume = 0;
byte old_volume = 0;
byte colorset = 0;
byte old_colorset = 0;
int b = 0; //brightness
byte d = 0; //brightness direction
int popcounter = 0;
byte oledpage = 0;
unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long previousAniMillis = 0;  // Animation Timer
unsigned long previousUpdateMillis = 0;  // Sensor Update Timer
unsigned long previousTimeoutMillis = 0;  // Timeout Timer
unsigned long previousPageMillis = 0;  // Timeout Timer
unsigned long previousPopupMillis = 0;  // Timeout Timer
unsigned long lastUpdate = 0 ; // for millis() when last update occoured

unsigned long patternInterval = 50 ; // time between steps in the color pattern
const long interval = 1000; // interval at which to blink (milliseconds)
const long animinterval = 50;  // interval for the slot-effect (milliseconds)
long rotationtime = 20000;
long pagecycletime;

long timeSinceLastOLEDpage = 0;

long timeSinceLastWUpdate = 0;
const int UPDATE_INTERVAL_SECS = 10 * 60; // Update every 10 minutes
bool AP_mode = false;
//-----------

// Load Wi-Fi library
#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
#include <ESP8266HTTPClient.h>

#include <NTPClient.h> //https://github.com/arduino-libraries/NTPClient/tree/master/examples
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org");

#include "ESPAsyncWebServer.h"
#include <ESPAsyncWiFiManager.h> //https://github.com/tzapu/WiFiManager || https://github.com/alanswx/ESPAsyncWiFiManager

#include "FS.h"   //Include File System Headers
#include "AsyncJson.h"
#include "ArduinoJson.h"

//for LED status
#include <Ticker.h> //https://github.com/esp8266/Arduino/blob/master/libraries/Ticker/src/Ticker.h
Ticker ticker;


void tick()
{
  //toggle state
  int state = digitalRead(wifiLED);  // get the current state of GPIO1 pin
  digitalWrite(wifiLED, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (AsyncWiFiManager *myWiFiManager) {

  Serial.println("Entered AccessPiont mode");
  //Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.1, tick);
  AP_mode = true;
}

const char* PAR1 = "aset"; //ids of the buttons in web ui

AsyncWebServer server(80);
DNSServer dns;

// Replaces placeholder with button section in your web page
String processor(const String& var) {
  //Serial.println(var);

  if (var == "ALARM_BUTTON") {
    String buttons = "";
    String buttonval;

    //modify the button according to state

    if (alarmset == 1) {
      buttonval = "checked";
    }
    else {
      buttonval = "";
    }

    buttons += "<label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"output\" " + buttonval + "><span class=\"slider\"></span></label>";
    return buttons;
  }

  return String();
}


//----------





int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(D2, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(D1, INPUT_PULLUP);

  //delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(D1) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master.
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(D2) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(D1, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(D1, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(D1, INPUT); // release SCL LOW
    pinMode(D1, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(D1) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(D1) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(D2) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(D2, INPUT); // remove pullup.
  pinMode(D2, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(D2, INPUT); // remove output low
  pinMode(D2, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(D2, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(D1, INPUT);
  return 0; // all ok
}







void setup() {
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.clear(); //sett all pixels to off
  pixels.show();

  //-----------

#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif

  //-----------

  int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  } else { // bus clear
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
  }


  EEPROM.begin(512);

  //pinMode(vbat, INPUT);          //RTC Battery voltage

  pinMode(buzzer, OUTPUT);        //SPEAKER
  pinMode(stateLED, OUTPUT);      //STATUS-LED
  pinMode(wifiLED, OUTPUT);      //STATUS-LED
  pinMode(bt_up, INPUT_PULLUP);   //BUTTON
  pinMode(bt_set, INPUT_PULLUP);  //BUTTON
  pinMode(bt_dwn, INPUT_PULLUP);  //BUTTON
  pinMode(bt_wifi, INPUT_PULLUP);  //WIFI BUTTON TEST

  digitalWrite(stateLED, LOW);
  digitalWrite(wifiLED, LOW);


#ifdef LightSensor
  pinMode(lightsens, INPUT);      //PHOTO SENSOR
#endif


#ifdef Si7021sensor
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true)
      ;
  }
#endif

#ifdef AudioSensor
  pinMode(mic, INPUT);            //AUDIO SENSOR
#endif

#ifdef OLED
  u8g2.begin();
  //u8g2.sendF("c", 0x0a7); //invert display
  connectstate();
#endif

  //Get data from EEPROM
  colorset = EEPROM.read(colorADDR);
  if (colorset > 8) {
    colorset = 0;
  }

  //colorset = 0;
  old_colorset = colorset;


  alarmhours = EEPROM.read(alarmHourADDR);

  if (alarmhours > 23) {
    alarmhours = 0;
  }

  alarmminutes = EEPROM.read(alarmMinuteADDR);
  if (alarmminutes > 59) {
    alarmminutes = 0;
  }

  alarmset = EEPROM.read(alarmStateADDR);
  if (alarmset > 1) {
    alarmset = 0;
  }

  old_alarmset = alarmset;

  wifion = EEPROM.read(wifiStateADDR);

  Serial.println();
  Serial.print("STORED ALARM TIME: ");

  if (alarmhours <= 9) {
    Serial.print('0');
  }
  Serial.print(alarmhours);
  Serial.print(":");
  if (alarmminutes <= 0) {
    Serial.print('0');
  }
  Serial.println(alarmminutes);


  if (alarmset == 1) {
    Serial.println("ALARM IS SET TO ON");
  }
  else {
    Serial.println("ALARM IS SET TO OFF");
  }



#ifdef RAD
  //do stuff
#endif

  tone(buzzer, 500, 50);


  //-----------------------
  // Initialize SPIFFS < FIXME
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }


  //runserver();

  Serial.print("STORED WIFI STATE: ");
  Serial.println(wifion);

  if (wifion == 0) {
    WiFi.forceSleepBegin();
  } //Disables WiFi if it is not on

  else if (wifion == 1) {
    runserver();
  }



  rtc.begin();

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost Power");
    // this will adjust to the date and time at compilation
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }


  //we don't need the 32K Pin, so disable it
  rtc.disable32K();

  // Disable and clear both alarms
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);

  pagecycletime = rotationtime;

  Serial.println("Clock ready");

  //delay(300);

  DateTime now = rtc.now();
  nowhour = now.hour();
  nowminute = now.minute(), DEC;
  nowsecond = now.second(), DEC;

  //-------
}






void runserver() {

  if (serveronline == false) {

    // Connect to Wi-Fi
    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED) {
    // delay(1000);
    // Serial.println("Connecting to WiFi..");
    //}


    //WiFiManager
    //set led pin as output
    //pinMode(stateLED, OUTPUT);
    // start ticker with 0.5 because we start in AP mode and try to connect
    ticker.attach(0.6, tick);


    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    AsyncWiFiManager wifiManager(&server, &dns);

    //reset settings - for testing
    //wifiManager.resetSettings();

    //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    wifiManager.setAPCallback(configModeCallback);

    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //and goes into a blocking loop awaiting configuration
    wifiManager.autoConnect("LED-Clock");

    if (!wifiManager.autoConnect()) {
      Serial.println("failed to connect and hit timeout");
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(1000);
    }

    //if you get here you have connected to the WiFi
    AP_mode = false;
    Serial.println("WiFi Connected!");
    ticker.detach();
    //keep LED on
    digitalWrite(wifiLED, HIGH);

    //WIFIMANAGER END

    connectstate();


    // Route for root / web page
    //server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //  request->send_P(200, "text/html", index_html, processor);
    //});

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/index.html", String(), false, processor);
    });

    // Route to load style.css file
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/style.css", "text/css");
    });

    // Route to load jquery.min.js file
    server.on("/jquery.min.js", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/jquery.min.js", "text/javascript");
    });

    server.on("/json", HTTP_ANY, [](AsyncWebServerRequest * request) {

      uint32_t free = system_get_free_heap_size();

      const size_t capacity = JSON_OBJECT_SIZE(28);
      DynamicJsonBuffer jsonBuffer(capacity);

      AsyncJsonResponse * response = new AsyncJsonResponse();
      JsonObject& root = response->getRoot();

      root["utc"] = -1; //UTC Time offset
      root["temp"] = newtemp; //newtemp; //temperature
      root["tcf"] = 0; //temperature type TODO
      root["hum"] = newhum; //humidity
      root["col"] = colorset; //color setting
      root["dig"] = animationsetting; //digitanimation TODO
      root["psl"] = 0; //automatic pageslide TODO
      root["aum"] = 0; //audiomode TODO
      root["alarm"] = alarmset; //alarm is on or off
      root["a_hh"] = alarmhours;
      root["a_mm"] = alarmminutes;
      root["a_type"] = 0; //alarm type (once, daily, only weekdays) TODO
      root["a_sc"] = 0; //alarm scource (buzzer/radio) TODO
      root["vol"] = 255; //volume TODO
      root["br"] = 0; //brightness 0-11, 11 = auto TODO
      root["radio"] = 0; //radio state (0 off, 1 on in vaf mode, 2 on in manual) TODO
      root["act_mhz"] = 10360; //set frequency TODO
      root["f1_mhz"] = 10360; //Favorite1 frequency TODO
      root["f1_name"] = "Radio Niedersachsen"; //Favorite1 name TODO
      root["f2_mhz"] = 10360; //Favorite2 frequency TODO
      root["f2_name"] = "Radio Niedersachsen"; //Favorite2 name TODO
      root["f3_mhz"] = 10360; //Favorite3 frequency TODO
      root["f3_name"] = "Radio Niedersachsen"; //Favorite3 name TODO
      root["com"] = irbutton; //button command
      root["wd1"] = -25; //weather data > actual temp TODO
      root["wd2"] = -25; //weather data > min day temp TODO
      root["wd3"] = -25;  //weather data > max day temp TODO
      root["wd4"] = free;  //weather data > icon TODO

      response->setLength();
      request->send(response);
    });


    // Send a GET request to <ESP_IP>/update?state=<inputMessage>
    server.on("/update", HTTP_GET, [] (AsyncWebServerRequest * request) {
      String inputMessage;
      String inputParam;
      // GET input1 value on <ESP_IP>/update?state=<inputMessage>
      if (request->hasParam(PAR1)) {
        inputMessage = request->getParam(PAR1)->value();
        inputParam = PAR1;
        if (inputMessage == "1") {
          alarmset = 1;
        }
        else {
          alarmset = 0;
        }
      }
      else {
        inputMessage = "No message sent";
        inputParam = "none";
      }
      Serial.println(inputMessage);
      Serial.println(inputParam);

      request->send(200, "text/plain", "OK");
    });

    // Send a GET request to <ESP_IP>/state
    server.on("/aset", HTTP_GET, [] (AsyncWebServerRequest * request) {
      request->send(200, "text/plain", String(alarmset).c_str());
    });

    // Start server
    server.begin();

    timeClient.begin();

    // Print ESP8266 Local IP Address
    Serial.println(WiFi.localIP());

    serveronline = true; //quit initialisiation
    old_wifion = 0; //this will trigger a popup at the beginning
  }

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
    tone(buzzer, 100, 50);
    buttonz = 1; //Up pressed

    if (i_butt > (longpresstime)) {
      tone(buzzer, 100, 500);
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
    tone(buzzer, 100, 50);

    if (i_butt > (longpresstime)) {
      buttonz = 5; //Button center pressed long
      tone(buzzer, 100, 500);
      delay(2);
    }

  }
  else if (digitalRead(bt_dwn) != 1) {
    while (digitalRead(bt_dwn) != 1) {
      delay(2);
      i_butt++;
    }
    buttonz = 3; //Down pressed
    tone(buzzer, 100, 50);

    if (i_butt > (longpresstime)) {
      buttonz = 6; //Button down pressed long
      tone(buzzer, 100, 500);
      delay(2);
    }
  }
  else if (digitalRead(bt_wifi) != 1) {
    while (digitalRead(bt_wifi) != 1) {
      delay(2);
      i_butt++;
    }
    buttonz = 7; //Wifi Button pressed
    tone(buzzer, 100, 50);
  }

  pressedbut = buttonz;
  return buttonz;

}





//MAIN LOOP
//==================================================================================================================
void loop() {

  Wire.requestFrom(0x08, 8);    // request 8 bytes from slave device #8
  int i = 0; //counter for each byte as it arrives
  while (Wire.available()) {
    wireArray[i] = Wire.read(); // every character that arrives it put in order in the empty array "t"
    i = i + 1;
  }


  int val1 = (wireArray[1] << 8) + wireArray[0]; //first two bytes
  int val2 = (wireArray[3] << 8) + wireArray[2]; //second two bytes
  int val3 = (wireArray[5] << 8) + wireArray[4]; //third two bytes
  int val4 = (wireArray[7] << 8) + wireArray[6]; //fourth two bytes

  adcvalue1 = val1;
  adcvalue2 = val3;
  micvalue = val2;

  // Serial.print("BAT: ");
  // Serial.println(val1);   //shows the data in the array t
  // Serial.print("MIC: ");
  // Serial.println(val2);   //shows the data in the array t
  // Serial.print("HEX CODE: ");
  // Serial.println(val3, HEX);   //shows the data in the array t
  // Serial.print("Remote BT: ");
  // Serial.println(val4);   //shows the data in the array t
  // Serial.println();

if (val4 == 7) {
  if(colorset < 8) {
  colorset++;
  }
  else {
    colorset = 0;
    }
  }
  if (val4 == 6) {
   if(colorset > 0) {
  colorset--;
  }
  else {
    colorset = 8;
    }
  }



  //------


  static int pattern = 0, lastReading;
  if (millis() - lastUpdate > patternInterval) updatePattern(pattern);

  buttoncheck(); //ckecks button interaction
  PopUphandler();
  updatetime();


#ifdef AUTOROTATION
  //AUTO - PAGE ROTATION

  unsigned long currentPageMillis = millis();
  if (currentPageMillis - previousPageMillis >= pagecycletime) {
    previousPageMillis = currentPageMillis;

#ifndef Si7021sensor
    if (page < 1) {
      if ((animateflag == 0) && (popup == 0)) { //only slide pages when nothing is happening
        page = page + 1;
        stepcounter = 0;
        pagecycletime = rotationtime / 3; //cycle throught temp and humidity pages faster
      }
    }
#endif

#ifdef Si7021sensor //add humidity page
    if (page < 2) {
      if ((animateflag == 0) && (popup == 0)) { //only slide pages when nothing is happening
        page = page + 1;
        stepcounter = 0;
        pagecycletime = rotationtime / 3; //cycle throught temp and humidity pages faster
      }
    }
#endif


    else {
      page = 0;
      stepcounter = 0;
      pagecycletime = rotationtime;
    }
    Serial.print("Automatic pageslide:");
    Serial.println(page);
  }
#endif




  //LEFT BUTTON
  if (pressedbut == 1) {

    if (colorset < 8) {
      colorset = colorset + 1;
    }
    else {
      colorset = 0;
    }

  }

  //RIGHT BUTTON
  if (pressedbut == 3) {
    stepcounter = 0; //reset slide animation steps

#ifdef Si7021sensor //add humidity page
    if (page < 2) {
      page = page + 1;
    }
#endif

#ifdef RTCtemp
    if (page < 1) {
      page = page + 1;
    }
#endif

    else {
      page = 0;
    }
    Serial.print("Manual page slide:");
    Serial.println(page);
  }

  //CENTER BUTTON
  if (pressedbut == 2) {

    if (alarmstate == 0) {

      if (alarmset == 0) {
        alarmset = 1;

      }
      else if (alarmset == 1) {
        alarmset = 0;

      }
    }
    else if (alarmstate == 1) {
      alarmstate = 0;
      Serial.println("Alarm buzzer off");
    }

  }


  //WIFI BUTTON
  if (pressedbut == 7) {

    if (wifion == 0) {
      wifion = 1;
      Serial.println("WiFi ON");
      EEPROM.put(wifiStateADDR, wifion);
      EEPROM.commit();
      runserver();
    }
    else {
      wifion = 0;
      Serial.println("WiFi OFF");
      EEPROM.put(wifiStateADDR, wifion);
      EEPROM.commit();
      WiFi.forceSleepBegin();
      //WiFi.mode(WIFI_OFF);
      serveronline = false;
    }

  }



  //CENTER BUTTON LONG
  if (pressedbut == 5) {

    stepcounter = 0;
    //get current time and alarm-setting for the setting menu

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

#ifdef OLED
  if (popup != 1) {
    OLEDdraw();
  }
#endif








  if (rtc.alarmFired(1) == true) {

    if (alarmset == 1) {
      alarmstate = 1;
      Serial.println("ALARM TRIGGERED BY RTC!");
      rtc.writeSqwPinMode(DS3231_OFF);
      rtc.clearAlarm(1);
      Serial.println("Alarm cleared");
    }
    else {
      alarmstate = 0;
      rtc.writeSqwPinMode(DS3231_OFF);
      rtc.clearAlarm(1);
    }

  }


  if (alarmset == 1) {
    digitalWrite(stateLED, HIGH);
    //FIXME
  }
  else {
    digitalWrite(stateLED, LOW);
  }


  getTempHum();
  //decodeIR();



  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(wifiLED, HIGH);
  }
  else {
    digitalWrite(wifiLED, LOW);
  }



  if (millis() - timeSinceLastOLEDpage > (1000L * 10)) { //10 sec intervall

    //cycle through oled pages every second

    if (WiFi.status() == WL_CONNECTED) { //if we are online, there is a weather page
      if (oledpage < 2) {
        oledpage++;
      }
      else {
        oledpage = 0;
      }

    }
    else { //if we are not online, there is no weather page
      if (oledpage < 1) {
        oledpage++;
      }
      else {
        oledpage = 0;
      }

    }

    timeSinceLastOLEDpage = millis();
  }


}


void  updatePattern(int pat) { // call the pattern currently being created


  //button actions here

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





}







//LED CODE
//==================================================================================================================


void color_rainbow() { // modified from Adafruit example to make it a state machine
  static uint16_t j = 0;
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, Wheel((i + j) & 255));
  }
  mapPixels();
  setbrightness();
  pixels.show();
  //delay(50);

  j++;
  if (j >= 256) j = 0;
  lastUpdate = millis(); // time for next change to the display
}

void color_rainbowcycle() {
  static uint16_t j = 0;
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
  }
  mapPixels();
  setbrightness();
  pixels.show();
  //delay(50);

  j++;
  if (j >= 256 * 5) j = 0;
  lastUpdate = millis(); // time for next change to the display
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
  //delay(50);


  lastUpdate = millis(); // time for next change to the display

}
void color_red() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 255, 0, 0); //red
  }
  mapPixels();
  setbrightness();
  pixels.show();



  lastUpdate = millis(); // time for next change to the display

}
void color_green() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 0, 255, 0); //green
  }
  mapPixels();
  setbrightness();
  pixels.show();



  lastUpdate = millis(); // time for next change to the display

}
void color_blue() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 0, 0, 255); //blue
  }
  mapPixels();
  setbrightness();
  pixels.show();



  lastUpdate = millis(); // time for next change to the display

}
void color_white() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 255, 255, 255); //white
  }
  mapPixels();
  setbrightness();
  pixels.show();



  lastUpdate = millis(); // time for next change to the display

}
void color_pink() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 255, 0, 255); //pink
  }
  mapPixels();
  setbrightness();
  pixels.show();



  lastUpdate = millis(); // time for next change to the display

}
void color_velvet() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 255, 255, 0); //velvet
  }
  mapPixels();
  setbrightness();
  pixels.show();

  lastUpdate = millis(); // time for next change to the display

}

void color_cyan() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 0, 255, 255); //cyan
  }
  mapPixels();
  setbrightness();
  pixels.show();



  lastUpdate = millis(); // time for next change to the display

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
  else if (number_min1 == 10) { //all off code
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 11) { //"c" code
    pixels.setPixelColor(2 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 13) { //"o up" code
    pixels.setPixelColor(0 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 7, pixels.Color(0, 0, 0));
  }
  else if (number_min1 == 14) { //"o down" code
    pixels.setPixelColor(4 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 7, pixels.Color(0, 0, 0));
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
  else if (number_hour2 == 10) { //all off code
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 11) { //"c" code
    pixels.setPixelColor(2 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 13) { //"o up" code
    pixels.setPixelColor(0 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 14, pixels.Color(0, 0, 0));
  }
  else if (number_hour2 == 14) { //"o down" code
    pixels.setPixelColor(4 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 14, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 14, pixels.Color(0, 0, 0));
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
  else if (number_hour1 == 11) { //"c" code
    pixels.setPixelColor(2 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(6 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 13) { //"o up" code
    pixels.setPixelColor(0 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1 + 21, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2 + 21, pixels.Color(0, 0, 0));
  }
  else if (number_hour1 == 14) { //"o down" code
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

#ifdef Si7021sensor
    pixels.setPixelColor(28 , pixels.Color(0, 0, 0));
    pixels.setPixelColor(29 , pixels.Color(0, 0, 0));
#endif

#ifndef Si7021sensor
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



  if (alarmstate == 0) {

#ifdef LightSensor

    long lightsum = 0;
    for (int p = 0; p < 100; p++) {
      lightsum += analogRead(lightsens); //Photo-Resistor
    }
    lightvalue = lightsum / 100;

    //Serial.print("Photosensor: ");
    //Serial.println(lightvalue);

    if (lightvalue < 20) { //value for dark
      lightvalue = 20;
    }
    else if (lightvalue > 1000) { //value for bright
      lightvalue = 1000;
    }

    lightvalue = map(lightvalue, 20, 1000, 10, 255);

    pixels.setBrightness(lightvalue); //regulates the brightness of the whole strip

#ifdef OLED
    u8g2.setContrast(lightvalue); //set brightness of the OLED
#endif

#endif


#ifndef LightSensor
    pixels.setBrightness(150); //max brightness if "LightSensor" not defined

#ifdef OLED
    u8g2.setContrast(255); //max brightness if "LightSensor" not defined
#endif

#endif

  }

  else if (alarmstate == 1) {
    //do wakeup LED and buzzer routine

    if (d == 0) {

      if (b < 250) {
        b = b + 25;
        if (b == 125) {
          tone(buzzer, 2900, 100);
        }
      }
      else if (b >= 250) {
        d = 1;
        b = 250;
        tone(buzzer, 2900, 100);
      }
    }

    else {
      if (b > 0) {
        b = b - 25;
      }
      else if (b <= 0) {
        d = 0;
        b = 0;
      }
    }


    pixels.setBrightness(b);
  }
}





//CLOCK FUNCTIONS getting time from RTC
//==================================================================================================================
void updateNumber() {

  number_hour2 = (nowhour % 10); //last digit of the hours

  if (nowhour < 10) {
    number_hour1 = 0;
  }
  else {
    number_hour1 = (nowhour / 10U) % 10; //first digit of the hours
  }

  number_min2 = (nowminute % 10); //last digit of the seconds


  if (nowminute < 10) {
    number_min1 = 0;
  }
  else {
    number_min1 = (nowminute / 10U) % 10; //first digit of the seconds
  }



  if (animationsetting == 1) {
    //animation every minute
    if (number_min2 != digitbuffer) {
      animateflag = 1;
      number_min2 = 0;
      Serial.println("Digit animation start - minute setting");
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
      Serial.println("Digit-animation start - ten minute setting");
    }
    else {
      animateflag = 0;
    }
  }

  else {    }


}



void getTempHum() {

  if (sensorupdateflag == 1) { //initial mesurment after boot

#ifdef Si7021sensor
    float c = sensor.readTemperature(); //temp in celsius
    float h = sensor.readHumidity();

#ifdef Temp_F
    c = c * 9.0 / 5.0 + 32.0; //convert to fahrenheit
#endif

#endif


#ifdef RTCtemp
    int16_t rtcTemp = rtc.getTemperature();
    float c = rtcTemp;

#ifdef Temp_F
    c = c * 9.0 / 5.0 + 32.0; //convert to fahrenheit
#endif

#endif

    //apply user temp offset
    c = c + tempoffset;
    newtemp = round(c);

#ifdef Si7021sensor
    //apply user humidity offset
    h = h + humidityoffset;
    newhum = round(h);

#endif

    sensorupdateflag = 0; //disable the firs measurement
  }
  else {



    unsigned long currentUpdateMillis = millis();

    if (currentUpdateMillis - previousUpdateMillis >= 2000) {
      previousUpdateMillis = currentUpdateMillis;

      if (tempsamplecount < 5) {

#ifdef Si7021sensor
        float c = sensor.readTemperature(); //temp in celsius
        float h = sensor.readHumidity();

#ifdef Temp_F
        c = c * 9.0 / 5.0 + 32.0; //convert to fahrenheit
#endif

#endif


#ifdef RTCtemp
        int16_t rtcTemp = rtc.getTemperature();
        //int16_t rtcTemp = Clock.getTemperature();

        float c = rtcTemp;
        //float c = rtcTemp / 4.; //convert to cnumpixelsius

#ifdef Temp_F
        c = c * 9.0 / 5.0 + 32.0; //convert to fahrenheit
#endif

#endif


        //apply user temperature offset
        c = c + tempoffset;
        buffertemp += c;


#ifdef Si7021sensor
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
        Serial.print("AVG Temperature:");
        Serial.println(buffertemp);
        newtemp = round(buffertemp);
        digittemp = newtemp * 10;

        buffertemp = 0; //reset buffer


#ifdef Si7021sensor
        bufferhum = bufferhum / 5;
        Serial.print("AVG Humidity:");
        Serial.println(bufferhum);
        newhum = round(bufferhum);
        digithum = newhum * 10;

        bufferhum = 0; //reset buffer
#endif

        tempsamplecount = 0; //reset the sample counter
        sensorupdateflag = 1;

      }
    }

  }
  return;

}





//TEMPERATURE READOUT
//==================================================================================================================
void showtemp() {

  if (menu == 0) {
    //updatetime();

    //draw a "dash" while waiting for data first time
    if (digittemp == 0) {
      number_hour1 = 15; //dash
      number_hour2 = 15; //dash
      number_min1 = 13; //code for c
      number_min2 = 11; //code for "°"

      //Serial.println("No avg. data yet, waiting…");
    }
    else {

      unsigned long currentAniMillis = millis();

      if (currentAniMillis - previousAniMillis >= 100) {
        previousAniMillis = currentAniMillis;

        if (stepcounter < 4) {
          stepcounter++;

        }
        else {
          stepcounter = 4;
        }


        //Serial.print("stepcounter:");
        //Serial.println(stepcounter);
      }
      if (stepcounter == 1) {
        //Slide Animation
        //1st step
        number_hour1 = (nowhour % 10); //last digit of the hours

        if (nowminute > 9) {
          number_hour2 = (nowminute / 10U) % 10; //first min digit
        }
        else {
          number_hour2 = 0;
        }
        number_min1 = (nowminute % 10); //second min digit
        number_min2 = (digittemp / 100U) % 10; //first digit of temp
      }
      else if (stepcounter == 2) {
        //2nd step
        if (nowminute > 9) {
          number_hour1 = (nowminute / 10U) % 10; //first min digit
        }
        else {
          number_hour1 = 0;
        }
        number_hour2 = (nowminute % 10); //second min digit
        number_min1 = (digittemp / 100U) % 10; //first digit of temp
        number_min2 = (digittemp / 10U) % 10; //first digit of temp
      }

      else if (stepcounter == 3) {
        //3rd step
        number_hour1 = (nowminute % 10); //second min digit
        number_hour2 = (digittemp / 100U) % 10; //first digit of temp
        number_min1 = (digittemp / 10U) % 10; //first digit of temp
        number_min2 = 13;
      }
      else {
        //4th step
        number_hour1 = (digittemp / 100U) % 10; //first digit of temp
        number_hour2 = (digittemp / 10U) % 10; //second digit of temp
        number_min1 = 13; //code for c
        number_min2 = 11; //code for "°"
      }





    }

  }

}





//HUMIDITY READOUT
//==================================================================================================================
void showhumidity() {

  if (newhum == 0) {
    number_hour1 = 15; //dash
    number_hour2 = 15; //dash
    number_min1 = 13; //code for "0up"  %
    number_min2 = 14; //code for "0down"  %
    // Serial.println("No avg. data yet, waiting…");
  }

  else {

    unsigned long currentAniMillis = millis();

    if (currentAniMillis - previousAniMillis >= 100) {
      previousAniMillis = currentAniMillis;

      if (stepcounter < 4) {
        stepcounter++;

      }
      else {
        stepcounter = 4;
      }

    }
    if (stepcounter == 1) {
      //Slide Animation
      //1st step


      number_hour1 = (digittemp / 10U) % 10; //second digit of temp
      number_hour2 = 13; //code for c
      number_min1 = 11; //code for "°"
      number_min2 = (digithum / 100U) % 10; //first digit of temp
    }
    else if (stepcounter == 2) {
      //2nd step
      number_hour1 = 13; //code for c
      number_hour2 = 11; //code for "°"
      number_min1 = (digithum / 100U) % 10; //first digit of temp
      number_min2 = (digithum / 10U) % 10; //second digit of temp
    }

    else if (stepcounter == 3) {
      //3rd step
      number_hour1 = 11; //code for "°"
      number_hour2 = (digithum / 100U) % 10; //first digit of temp
      number_min1 = (digithum / 10U) % 10; //second digit of temp
      number_min2 = 13; //code for "°"  %
    }
    else {
      //4th step
      number_hour1 = (digithum / 100U) % 10; //first digit of temp
      number_hour2 = (digithum / 10U) % 10; //second digit of temp
      number_min1 = 13; //code for "°"  %
      number_min2 = 14; //code for "°"  %
    }


  }


}




//SET TIME FUNCTION
//==================================================================================================================
void settime() {


  newhours = nowhour;
  newminutes = nowminute;
  newalarmhours = alarmhours;
  newalarmminutes = alarmminutes;

  Serial.println("entered menu");
  Serial.print("newhours:");
  Serial.print(newhours);
  Serial.print(", newminutes:");
  Serial.println(newminutes);

  delay(100);

  while (menu == 1) {
    buttoncheck();



    delay(50);

    uint32_t off = pixels.Color(0, 0, 0);
    uint32_t white = pixels.Color(255, 255, 255);
    uint32_t red = pixels.Color(255, 0, 0);



    if ((menustep == 0) || (menustep == 1)) {
      //display time digits
      pixels.fill(white, 0, 29);

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
    }

    else if ((menustep == 2) || (menustep == 3)) {
      //display alarm digits
      pixels.fill(red, 0, 29);


      number_hour2 = (newalarmhours % 10); //last digit of the seconds
      if (newalarmhours < 10) {
        number_hour1 = 0;
      }
      else {
        number_hour1 = (newalarmhours / 10U) % 10; //first digit of the seconds
      }

      number_min2 = (newalarmminutes % 10); //last digit of the seconds
      if (newalarmminutes < 10) {
        number_min1 = 0;
      }
      else {
        number_min1 = (newalarmminutes / 10U) % 10; //first digit of the seconds
      }
    }





    //time set routine:

    if (menustep == 0) {

      pixels.fill(off, 0, 14); // hours
      pixels.fill(off, 28, 29); //dots

      Serial.print("newhours >:");
      Serial.println(newhours);

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
      pixels.fill(off, 14, 30); // minutes and dots

      Serial.print("newminutes >:");
      Serial.println(newminutes);

      //edit minutes
      if (pressedbut == 2) {
        menustep = 2;
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
    else if (menustep == 2) {
      pixels.fill(off, 0, 14); // hours
      pixels.fill(off, 28, 29); //dots

      Serial.print("newalarmhours >:");
      Serial.println(newalarmhours);

      //edit alarm hours
      if (pressedbut == 2) {
        menustep = 3;
      }
      if (pressedbut == 1) {
        if (newalarmhours < 23) {
          newalarmhours += 1;
        }
        else {
          newalarmhours = 0;
        }
      }
      if (pressedbut == 3) {
        if (newalarmhours > 0) {
          newalarmhours -= 1;
        }
        else {
          newalarmhours = 23;
        }
      }
    }
    else if (menustep == 3) {
      pixels.fill(off, 14, 30); // minutes and dots

      Serial.print("newalarmminutes >:");
      Serial.println(newalarmminutes);

      //edit minutes
      if (pressedbut == 2) {
        menustep = 0;
      }
      if (pressedbut == 1) {
        if (newalarmminutes < 59) {
          newalarmminutes += 1;
        }
        else {
          newalarmminutes = 0;
        }
      }
      if (pressedbut == 3) {
        if (newalarmminutes > 0) {
          newalarmminutes -= 1;
        }
        else {
          newalarmminutes = 59;
        }
      }
    }
    mapPixels();
    setbrightness();
    pixels.show();
    delay(10);


    if (pressedbut == 5) {


      //FIXME set date
      if ((newhours != nowhour) || (newminutes != nowminute)) { //check if something was actually changed

        nowhour = newhours;
        nowminute = newhours;

        rtc.adjust(DateTime(2020, 12, 30, newhours, newminutes, 0));
        DateTime now = rtc.now();

        Serial.print("TIME SAVED:");
        Serial.print(newhours);
        Serial.print(":");
        Serial.println(newminutes);
      }

      if ((newalarmhours != alarmhours) || (newalarmminutes != alarmminutes)) { //check if something was actually changed

        rtc.writeSqwPinMode(DS3231_OFF);
        rtc.disableAlarm(2);
        rtc.setAlarm1(DateTime(2020, 12, 30, newalarmhours, newalarmminutes, 0), DS3231_A1_Hour);

        Serial.print("ALARM SAVED:");
        Serial.print(newalarmhours);
        Serial.print(":");
        Serial.println(newalarmminutes);

        EEPROM.put(alarmHourADDR, newalarmhours);
        EEPROM.put(alarmMinuteADDR, newalarmminutes);
        EEPROM.commit();

        alarmhours = newalarmhours;
        alarmminutes = newalarmminutes;
      }

      menustep = 0;
      menu = 0;
    }
  }
}





//ROLLING NUMBER ANIMATION
//==================================================================================================================
void animate() {

  if (menu == 0) {

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

        if (number_min2 == 10) {

          //animation every minute
          if (animationsetting == 1) {

            number_min2 = (nowminute % 10); //second digit of the seconds
            digitbuffer = number_min2;
          }

          //animation every 10 minutes
          else if (animationsetting == 2) {

            number_min1 = (nowminute / 10U) % 10;  //first digit of the seconds
            digitbuffer = number_min1;
          }

          Serial.println("Digit animation end");
          //Serial.println(number_min2);
          animateflag = 0;
        }
      }
    }

    else {

      return;

    }
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
  if (WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


void PopUphandler() {


  if (popup == 1) {
    popcounter++;

    if (popcounter >= poptime) {
      //popup timeout
      Serial.println("Popup timeout");

      EEPROM.put(alarmStateADDR, alarmset);
      EEPROM.put(colorADDR, colorset);
      EEPROM.commit();
      Serial.println("Settings saved");

      popcounter = 0;
      popup = 0;
    }
    else {
      popup = 1;
    }
  }


  if (colorset != old_colorset) {
    tone(buzzer, 100, 50);
    popcounter = 0;
    poptime = 1000; //default value
    popup = 1;
    Serial.print("Color change detected:");
    Serial.println(colorset);
    colorpopup();
    old_colorset = colorset;

  }

  else if (alarmset != old_alarmset) {
    tone(buzzer, 100, 50);
    popcounter = 0;
    poptime = 1000; //default value
    popup = 1;
    Serial.print("Alarm change detected:");
    Serial.println(alarmset);
    alarmpopup();
    old_alarmset = alarmset;

  }

  else if (wifion != old_wifion) {
    tone(buzzer, 100, 50);
    popcounter = 0;
    if (wifion == 1) {
      updateflag = 1;
      poptime = 1000; //needed more time for the popup to read the ip adress
    }
    popup = 1;
    Serial.print("WiFi state change detected:");
    Serial.println(wifion);
    wifipopup();
    old_wifion = wifion;
  }

  else if (brightness != old_brightness) {
    Serial.println("brightness-pop triggered");
    poptime = 1000; //default value
    //popup = 1;

#ifdef OLED
    //brightnesspopup();
#endif

    old_brightness = brightness;
    tone(buzzer, 100, 50);
  }

  else if (volume != old_volume) {
    Serial.println("volume-pop triggered");
    poptime = 1000; //default value
    popup = 1;

#ifdef OLED
    //volumepopup();
#endif

    old_volume = volume;
    tone(buzzer, 100, 50);
  }


  //exit;

}







void connectstate() {
#ifdef OLED
  u8g2.firstPage();
  do {

    if (wifion == 1) {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 24);

      if (WiFi.status() != WL_CONNECTED) {
        u8g2.clear();
        u8g2.print("CONNECTING...");
      }
      else {
        //do stuff
      }

    }

    while (AP_mode == true) {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 24);
      u8g2.print("AP MODE!");
    }
  } while ( u8g2.nextPage() );
#endif
}

void alarmpopup() {
#ifdef OLED
  u8g2.firstPage();
  do {

    if (popup == 1) {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 24);
      u8g2.print("ALARM: ");
      if (alarmset == 1) {
        u8g2.print("ON");
      }
      else {
        u8g2.print("OFF");
      }
    }

  } while ( u8g2.nextPage() );
#endif
}

void wifipopup() {
#ifdef OLED
  u8g2.firstPage();
  do {

    if (popup == 1) {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 21);

      if (wifion == 0) {
        u8g2.print("WIFI: OFF");
      }
      else {
        u8g2.print("WIFI CONNECTED");
        u8g2.setCursor(0, 33);
        u8g2.print("IP: ");
        u8g2.print(WiFi.localIP());
      }

    }

  } while ( u8g2.nextPage() );
#endif
}

void colorpopup() {
#ifdef OLED
  u8g2.firstPage();
  do {

    if (popup == 1) {



      u8g2.setFont(u8g2_font_crox2hb_tf);
      byte width = u8g2.getDisplayWidth();

      if (colorset == 0) {
        byte pos = ((width - (u8g2.getUTF8Width("Rainbow Cycle"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("Rainbow Cycle");
      }
      else if (colorset == 1) {
        byte pos = ((width - (u8g2.getUTF8Width("Rainbow"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("Rainbow");
      }
      else if (colorset == 2) {
        byte pos = ((width - (u8g2.getUTF8Width("CyberPunk"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("CyberPunk");
      }
      else if (colorset == 3) {
        byte pos = ((width - (u8g2.getUTF8Width("White"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("White");
      }
      else if (colorset == 4) {
        byte pos = ((width - (u8g2.getUTF8Width("Pink"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("Pink");
      }
      else if (colorset == 5) {
        byte pos = ((width - (u8g2.getUTF8Width("Yellow"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("Yellow");
      }
      else if (colorset == 6) {
        byte pos = ((width - (u8g2.getUTF8Width("Red"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("Red");
      }
      else if (colorset == 7) {
        byte pos = ((width - (u8g2.getUTF8Width("Green"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("Green");
      }
      else if (colorset == 8) {
        byte pos = ((width - (u8g2.getUTF8Width("Blue"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("Blue");
      }
      else {
        byte pos = ((width - (u8g2.getUTF8Width("X"))) / 2); //calculate the text lenght
        u8g2.setCursor(pos, 21);
        u8g2.print("X");
      }


    }

  } while ( u8g2.nextPage() );
#endif
}

void brightnesspopup() {
#ifdef OLED
  u8g2.firstPage();
  do {

    if (popup == 1) {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 21);
      u8g2.print("NEW BRIGHT:");
      u8g2.print(brightness);
    }

  } while ( u8g2.nextPage() );
#endif
}


void volumepopup() {
#ifdef OLED
  u8g2.firstPage();
  do {

    if (popup == 1) {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 21);
      u8g2.print("NEW VOLUME:");
      u8g2.print(volume);
    }

  } while ( u8g2.nextPage() );
#endif
}


#ifdef OLED
void OLEDdraw() {

  //Serial.print("wifiState:");
  //Serial.println(wifiState);
  //Serial.print("module:");
  //Serial.println(wifimodule);

  u8g2.firstPage();
  do {

    const char DEGREE_SYMBOL[] = { 0xB0, '\0' };

double micvolts = (micvalue*3.3)/1024;

if (micvalue < 200) {
  //micvalue = 200;
  }

    //u8g2.drawFrame(0, 0, 96, 16);
    u8g2.drawRFrame(0, 0, 128, 32, 16);
      
    byte soundval = map(micvolts, 0, 3.3, 1, 90);
    u8g2.drawBox(0, 36, soundval, 6);
    
    u8g2.setCursor(100, 43);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.print(adcvalue1);
    

    if (oledpage == 0) {

      u8g2.setCursor(19, 21);
      u8g2.setFont(u8g2_font_crox2hb_tf);
      u8g2.print(nowweekday);
      u8g2.print(" ");
      if (nowday <= 9) {
        u8g2.print("0");
      }
      u8g2.print(nowday);
      u8g2.print(".");
      if (nowmonth <= 9) {
        u8g2.print("0");
      }
      u8g2.print(nowmonth);
      u8g2.print(".");
      u8g2.print(nowyear);

      u8g2.setFont(u8g2_font_6x10_tf);
    }

    else if (oledpage == 1) {

      u8g2.setCursor(18, 21);
      //u8g2.setFont(u8g2_font_helvB10_tf);
      u8g2.setFont(u8g2_font_crox2hb_tf);

      u8g2.print("T:");
      u8g2.print(newtemp, 0);
      u8g2.print("\xb0"); //degree symbol

#ifndef Temp_F
      u8g2.print("C");
#endif
#ifdef Temp_F
      u8g2.print("F");
#endif

#ifdef Si7021sensor
      u8g2.print(" H:");
      byte hu = round(newhum);
      u8g2.print(hu);
      u8g2.print("%");
#endif
      u8g2.setFont(u8g2_font_6x10_tf);
    }

    else if (oledpage == 2) {
      u8g2.setCursor(18, 21);
      u8g2.setFont(u8g2_font_crox2hb_tf);
      u8g2.print(weather_i);
      u8g2.print(" - ");
      u8g2.print(weather_now);
      u8g2.print("\xb0"); //degree symbol
      //u8g2.print("C");

      //byte weather_max;
      //byte weather_min;
      //byte weather_i;

      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(70, 13);
      u8g2.print("max ");
      u8g2.print(weather_max);
      u8g2.print("\xb0"); //degree symbol

      u8g2.setDrawColor(1);
      u8g2.drawLine(70, 16, 105, 16);

      u8g2.setCursor(70, 25);
      u8g2.print("min ");
      u8g2.print(weather_min);
      u8g2.print("\xb0"); //degree symbol


    }



    if (WiFi.status() == WL_CONNECTED) {
      u8g2.setCursor(8, 20);
      u8g2.print("O");
    }

    if (alarmset == 1) {
      u8g2.setCursor(115, 20);
      u8g2.print("A");
    }

    u8g2.setCursor(0, 54);
    u8g2.print("PAGE: ");
    u8g2.print(page);

    u8g2.setCursor(0, 64);
    if (nowhour <= 9) {
      u8g2.print("0");
    }
    u8g2.print(nowhour);
    u8g2.print(":");
    if (nowminute <= 9) {
      u8g2.print("0");
    }
    u8g2.print(nowminute);
    u8g2.print(":");
    if (nowsecond <= 9) {
      u8g2.print("0");
    }
    u8g2.print(nowsecond);


    u8g2.setCursor(65, 54);
    u8g2.print("T:");
    u8g2.print(newtemp, 0);

#ifndef Temp_F
    u8g2.print(" C");
#endif
#ifdef Temp_F
    u8g2.print(" F");
#endif

#ifdef Si7021sensor
    u8g2.setCursor(65, 64);
    u8g2.print("H:");
    u8g2.print(newhum, 0);
    u8g2.print(" %");
#endif



  } while ( u8g2.nextPage() );

}
#endif

void updatetime() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (WiFi.status() == WL_CONNECTED) {
      timeClient.update(); //do this only if online
      //timeClient.updateTime();
    }

    DateTime now = rtc.now();
    nowhour = now.hour();
    nowminute = now.minute();
    nowsecond = now.second();
    nowweekday = daysOfTheWeek[now.dayOfTheWeek()];
    nowday = now.day();
    nowmonth = now.month();
    nowyear = now.year();


    if (WiFi.status() == WL_CONNECTED) {

      Serial.println("ONLINE");
      getNetworkData();


    }

    //timeClient.getHours();
    //timeClient.getMinutes();
    //timeClient.getSeconds()
    //timeClient.getDay(), DEC;
    // getDD(), DEC;
    //getMM(), DEC;
    //getYY(), DEC;


    Serial.print(F("TIME NOW:"));
    if (nowhour <= 9) {
      Serial.print('0');
    }
    Serial.print(nowhour);
    Serial.print(':');
    if (nowminute <= 9) {
      Serial.print('0');
    }
    Serial.print(nowminute);
    Serial.print(':');
    if (nowsecond <= 9) {
      Serial.print('0');
    }
    Serial.println(nowsecond);

    Serial.print(nowweekday);
    Serial.print(", ");
    Serial.print(nowday);
    Serial.print('.');
    Serial.print(nowmonth);
    Serial.print('.');
    Serial.print(nowyear);
    Serial.println();
    Serial.println();
    Serial.flush();


    //Dot animation
    if (dot == 0) {
      dot = 1;
    }
    else {
      dot = 0;
    }




  }
}



uint16_t getYY() {
  time_t rawtime = timeClient.getEpochTime();
  struct tm * ti;
  ti = localtime (&rawtime);

  uint16_t year = ti->tm_year + 1900;
  uint16_t yearStr = year;
  return yearStr;
}


uint8_t getMM() {
  time_t rawtime = timeClient.getEpochTime();
  struct tm * ti;
  ti = localtime (&rawtime);

  uint8_t month = ti->tm_mon + 1;
  uint8_t monthStr = month < 10 ? 0 + uint8_t(month) : uint8_t(month);
  return monthStr;
}

uint8_t getDD() {
  time_t rawtime = timeClient.getEpochTime();
  struct tm * ti;
  ti = localtime (&rawtime);

  uint8_t day = ti->tm_mday;
  uint8_t dayStr = day < 10 ? 0 + uint8_t(day) : uint8_t(day);
  return dayStr;
}

void getNetworkData() {

  String ApiKey = "cd0567dd27516ea32592344c5f133c3f";
  String City = "Hamburg";
  String CountryCode = "DE";
  long CityID = 0;


  if (updateflag == 1) {
    //get weather

    const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(1) + 2 * JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(6) + JSON_OBJECT_SIZE(13) + 400;
    DynamicJsonBuffer jsonBuffer(capacity);

    String serverPath = "http://api.openweathermap.org/data/2.5/weather?q=" + City + "," + CountryCode + "&APPID=" + ApiKey + "&mode=json&units=metric&cnt=2%20HTTP/1.1";
    //http://api.openweathermap.org/data/2.5/weather?q=Hamburg,DE&appid=cd0567dd27516ea32592344c5f133c3f&mode=json&units=metric&cnt=2%20HTTP/1.1
    JsonObject& root = jsonBuffer.parseObject(httpGETRequest(serverPath.c_str()));
    //int coord_lon = root["coord"]["lon"]; // 10
    //float coord_lat = root["coord"]["lat"]; // 53.55

    JsonObject& weather_0 = root["weather"][0];
    //int weather_0_id = weather_0["id"]; // 600
    //const char* weather_0_main = weather_0["main"]; // "Snow"
    //const char* weather_0_description = weather_0["description"]; // "light snow"
    String weather_icon = weather_0["icon"]; // "13n"
    //const char* base = root["base"]; // "stations"

    JsonObject& main = root["main"];
    float main_temp = main["temp"]; // -1.69
    //float main_feels_like = main["feels_like"]; // -5.6
    float main_temp_min = main["temp_min"]; // -2.22
    float main_temp_max = main["temp_max"]; // -1
    //int main_pressure = main["pressure"]; // 998
    //int main_humidity = main["humidity"]; // 86
    //int visibility = root["visibility"]; // 9000
    //float wind_speed = root["wind"]["speed"]; // 2.06
    //int wind_deg = root["wind"]["deg"]; // 100
    //int clouds_all = root["clouds"]["all"]; // 75
    //long dt = root["dt"]; // 1612201131

    JsonObject& sys = root["sys"];
    //int sys_type = sys["type"]; // 1
    //int sys_id = sys["id"]; // 1263
    const char* sys_country = sys["country"]; // "DE"
    //long sys_sunrise = sys["sunrise"]; // 1612163116
    //long sys_sunset = sys["sunset"]; // 1612195309
    //int timezone = root["timezone"]; // 3600
    //long id = root["id"]; // 2911298
    const char* city = root["name"]; // "Hamburg"
    //int cod = root["cod"]; // 200

    weather_now = round(main_temp);
    weather_max = round(main_temp_max);
    weather_min = round(main_temp_min);

    if ((weather_icon == "01d") || (weather_icon == "01n")) {
      weather_i = 1; //clear sky
    }
    else if ((weather_icon == "02d") || (weather_icon == "02n")) {
      weather_i = 2; //few clouds
    }
    else if ((weather_icon == "03d") || (weather_icon == "03n")) {
      weather_i = 3; //scattered clouds
    }
    else if ((weather_icon == "04d") || (weather_icon == "04n")) {
      weather_i = 4; //broken clouds
    }
    else if ((weather_icon == "09d") || (weather_icon == "09n")) {
      weather_i = 9; //shower rain
    }
    else if ((weather_icon == "10d") || (weather_icon == "10n")) {
      weather_i = 10; //rain
    }
    else if ((weather_icon == "11d") || (weather_icon == "11n")) {
      weather_i = 11; //thunderstorm
    }
    else if ((weather_icon == "13d") || (weather_icon == "13n")) {
      weather_i = 13; //snow
    }
    else if ((weather_icon == "50d") || (weather_icon == "50n")) {
      weather_i = 50; //mist
    }
    else {
      weather_i = 0;
    }


    Serial.println("TIME TO SYNC ONLINE DATA:");
    Serial.println("=====================================");
    Serial.print(city);
    Serial.print(", ");
    Serial.println(sys_country);
    Serial.print("Temperature: ");
    Serial.println(main_temp);
    Serial.print("temp_min: ");
    Serial.println(main_temp_min);
    Serial.print("temp_max: ");
    Serial.println(main_temp_max);
    Serial.print("ICON: ");
    Serial.println(weather_icon);
    Serial.println("=====================================");


    /////////////////


    //timeclient data
    long val = UTCoffset * 3600;
    timeClient.setTimeOffset(val);

    Serial.print("Network time: ");
    Serial.print(timeClient.getHours());
    Serial.print(":");
    Serial.print(timeClient.getMinutes());
    Serial.print(":");
    Serial.print(timeClient.getSeconds());
    Serial.print(" - ");
    Serial.print(getDD());
    Serial.print(".");
    Serial.print(getMM());
    Serial.print(".");
    Serial.println(getYY());
    Serial.println("=====================================");

    //Serial.println(timeClient.getFormattedTime());

    //set the RTC if there is a difference
    if ((nowhour != timeClient.getHours()) || (nowminute != timeClient.getMinutes())) {
      //set the clock
      Serial.println("Time difference detected - syncing RTC");
      rtc.adjust(DateTime(getYY(), getMM(), getDD(), timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds()));
      //DateTime now = rtc.now();
    }
    updateflag = 0; //set update to zero
  }
  else {

    if (millis() - timeSinceLastWUpdate > (1000L * UPDATE_INTERVAL_SECS)) {

      updateflag = 1; //toggle the update

      timeSinceLastWUpdate = millis();
    }

  }




  // Serial.print("Updateflag: ");
  // Serial.println(updateflag);
}


String httpGETRequest(const char* serverName) {
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    //Serial.print("HTTP Response code: ");
    //Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

extern "C" {
#include "user_interface.h"
}
