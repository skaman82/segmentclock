//Code by Albert Kravcov
//==================================================================================================================
// TODOS
// - avg. measurements from DHT() and lightsensor()
// - temp + humidity user offsetts()
// - EEPROM support() & menus for Animation() + Color()
// - RTC battery alarm()
// - Alarm setting support()
// - Microphone support()



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


//==================================================================================================================

#include <Adafruit_NeoPixel.h> //Adafruit NeoPixel
#include <DS3232RTC.h> // https://github.com/JChristensen/DS3232RTC
#include <EEPROM.h>


//OPTIONAL USER CONFIG START
//==================================================================================================================
#define LightSensor   //An external photo-resistor can automatically adjust brightness of the clock
#define AudioSensor   //uncomment if you are using an microphone, adds additional animation mode controlled by sound
#define DHTsensor     //The DHT sensor will show temperature and humidity data > CONFIGURE SENSOR BELLOW!
//==================================================================================================================

#ifdef DHTsensor
#include <DHT.h>            //"Adafruit DHT" + "Ardafruit Unified Sensor" libraries required;
#define DHTPIN          7   //AUX2 pad (DHT22, DHT21 or DHT11) 

//UNCOMMENT THE RIGHT SENSOR
//#define DHTTYPE DHT22     // DHT 22   (AM2302), AM2321
//#define DHTTYPE DHT21     // DHT 21   (AM2301)
#define DHTTYPE DHT11       // DHT 11
#endif

//==================================================================================================================

float tempoffset       =   -0.5;  //Temperature adjustment (positive or negative value) no DHT Sensor required
float humidityoffset   =   0.0;   //Humidity adjustment (positive or negative value) only with DHT sensor

//==================================================================================================================
//USER CONFIG END


//#define serial //Debug data over Serial


#define LEDPIN          3
#define bt_up           4   //Button up
#define bt_set          5   //Button set
#define bt_dwn          6   //Button down
#define lightsens       A1  //Button down
#define buzzer          9   //Speaker
#define vbat            A0  //RTC Battery Monitoring
#define mic             A2  //AUX1 pad - Microphone 

#define colorADDR       1   // EEPROM Adress
#define animationADDR   2   // EEPROM Adress
#define alarmStateADDR  3   // EEPROM Adress
#define alarmHourADDR   4   // EEPROM Adress
#define alarmMinuteADDR 5   // EEPROM Adress

#define NUMPIXELS       60
#define longpresstime   500 // in ms


byte colorset = 1;
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
byte humsamplecount = 0;
int16_t buffertemp = 0;
int16_t bufferhum = 0;
int16_t hum = 0;
int16_t temp = 0;
int16_t newtemp = 0;
int16_t newhum = 0;


unsigned long previousMillis = 0; // will store last time LED was updated
unsigned long previousAniMillis = 0;  // will store last time LED was updated
unsigned long previousUpdateMillis = 0;  // will store last time LED was updated


const long interval = 1000; // interval at which to blink (milliseconds)
const long animinterval = 100;  // interval at which to blink (milliseconds)

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);
#ifdef DHTsensor
DHT dht(DHTPIN, DHTTYPE);
#endif






void setup() {
  Serial.begin(9600);
  pinMode(vbat, INPUT);           //RTC Battery voltage
  pinMode(buzzer, OUTPUT);        //SPEAKER
  pinMode(bt_up, INPUT_PULLUP);   //BUTTON
  pinMode(bt_set, INPUT_PULLUP);  //BUTTON
  pinMode(bt_dwn, INPUT_PULLUP);  //BUTTON

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.show(); // This initializes the NeoPixel library.

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

  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  //Get data from EEPROM
  colorset = EEPROM.read(colorADDR);
  if (colorset > 8) {
    colorset = 0;
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

  ledcontrol();

  //sync time every five minutes
  unsigned long currentUpdateMillis = millis();
  if (currentUpdateMillis - previousUpdateMillis >= 300000) {
    previousUpdateMillis = currentUpdateMillis;
    //   setSyncProvider(RTC.get);
    //   Serial.println("TIME SYNCED");
  }

}





//LED CODE
//==================================================================================================================
void ledcontrol() {
  uint16_t i, j;
  for (j = 0; j < 256; j++) { // cycle of all colors on wheel

    buttoncheck();

    if (pressedbut == 1) {
      if (colorset < 9) {
        colorset = colorset + 1;
        Serial.println(colorset);
      }
      else {
        colorset = 0;
        Serial.println(colorset);
      }
    }
    if (pressedbut == 3) {
#ifdef DHTsensor //add humidity page
      if (page < 2) {
        page = page + 1;
        //reset temp and humidity samples
        buffertemp = 0;
        bufferhum = 0;
        tempsamplecount = 0;
        humsamplecount = 0;

        Serial.println(page);
      }
#endif

#ifndef DHTsensor
      if (page < 1) {
        page = page + 1;

        //reset temp samples
        buffertemp = 0;
        tempsamplecount = 0; //reset temp and humidity samples



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
      newhours = hour();
      newminutes = minute();
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


    if (colorset == 0) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels()) + j) & 255));
        pixels.setPixelColor(28, Wheel((((i * 256 / pixels.numPixels()) + j) & 255) + 80)); //color offset fpr the dot pixels
        pixels.setPixelColor(29, Wheel((((i * 256 / pixels.numPixels()) + j) & 255) + 80)); //color offset fpr the dot pixels
      }
    }

    else if (colorset == 1) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, Wheel((i + j) & 255));
      }
    }
    else if (colorset == 2) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 255, 255, 255);
      }
    }
    else if (colorset == 3) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 255, 0, 0);
      }
    }

    else if (colorset == 4) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 0, 255, 0);
      }
    }

    else if (colorset == 5) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 0, 0, 255);
      }
    }
    else if (colorset == 6) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 255, 0, 255);
      }
    }
    else if (colorset == 7) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 255, 255, 0);
      }
    }
    else if (colorset == 8) {
      for (i = 0; i < pixels.numPixels(); i++) {
        pixels.setPixelColor(i, 0, 255, 255);
      }
    }
    else if (colorset == 9) {
      for (i = 0; i < pixels.numPixels(); i++) {

        uint32_t mint = pixels.Color(0, 250, 180);
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
    }


    mapPixels();
    setbrightness();
    pixels.show();
    delay(100);
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

  // Serial.print("Fhotosensor: ");
  // Serial.println(lightvalue);

  if (lightvalue < 40) { //value for dark
    lightvalue = 40;
  }
  else if (lightvalue > 1000) { //value for bright
    lightvalue = 1000;
  }

  lightvalue = map(lightvalue, 40, 1000, 50, 255);
  pixels.setBrightness(lightvalue); //regulates the brightness of the whole strip
#endif
#ifndef LightSensor
  pixels.setBrightness(255); //max brightness if "LightSensor" not defined
#endif
}





//CLOCK FUNCTIONS
//==================================================================================================================
void updateNumber() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    number_hour2 = (hour() % 10); //last digit of the hours

    if (hour() < 10) {
      number_hour1 = 0;
    }
    else {
      number_hour1 = (hour() / 10U) % 10; //first digit of the hours
    }

    number_min2 = (minute() % 10); //last digit of the seconds


    if (minute() < 10) {
      number_min1 = 0;
    }
    else {
      number_min1 = (minute() / 10U) % 10; //first digit of the seconds
    }


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






//TEMPERATURE READOUT (RTC & DHT)
//==================================================================================================================
void showtemp() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;

    if (tempsamplecount < 5) {

#ifndef DHTsensor
      int16_t rtcTemp = RTC.temperature();
      float c = rtcTemp / 4.; //convert to celsius
#endif

#ifdef DHTsensor
      float c = dht.readTemperature(); //temp in celsius

      if (isnan(c)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
      }
#endif

      Serial.print(F("Sensor Temperature: "));
      Serial.println(c);

      //apply user temperature offset
      c = c + tempoffset;

      temp = c * 10 ;

      buffertemp += temp;
      tempsamplecount++;
    }

    else if (tempsamplecount == 5) {
      //get the avarege of the five measurements
      buffertemp = buffertemp / 5;
      buffertemp = buffertemp; //adjust the result if neccesary

      Serial.print("AVG Temp:");
      Serial.println(buffertemp);

      newtemp = buffertemp;
      tempsamplecount = 0; //reset the sample counter
      buffertemp = 0;
    }

    //draw a "dash" while waiting for data first time
    if (newtemp == 0) {
      number_hour1 = 15; //dash
      number_hour2 = 15; //dash
      number_min1 = 13; //code for c
      number_min2 = 11; //code for "°"

      Serial.println("No avg. data yet, waiting…");
    }
    else {
      number_hour1 = (newtemp / 100U) % 10; //first digit of temp
      number_hour2 = (newtemp / 10U) % 10; //second digit of temp
      number_min1 = 13; //code for c
      number_min2 = 11; //code for "°"
    }


  }

}





//HUMIDITY READOUT (DHT required)
//==================================================================================================================
void showhumidity() {
#ifdef DHTsensor

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;


    if (humsamplecount < 5) {
      float h = dht.readHumidity();
      hum = h * 10;

      if (isnan(h)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
      }
      else {
        Serial.print(F("Sensor Humidity: "));
        Serial.println(h);
      }

      //apply user temperature offset
      hum = hum + humidityoffset;

      bufferhum += hum;
      humsamplecount++;
    }

    else if (humsamplecount == 5) {
      //get the avarege of the five measurements
      bufferhum = bufferhum / 5;

      Serial.print("AVG Hum:");
      Serial.println(bufferhum);

      newhum = bufferhum;
      humsamplecount = 0; //reset the sample counter
      bufferhum = 0;
    }


    //draw a "dash" while waiting for data first time
    if (newhum == 0) {
      number_hour1 = 15; //dash
      number_hour2 = 15; //dash
      number_min1 = 14; //code for "°"  %
      number_min2 = 14; //code for "°"  %

      Serial.println("No avg. data yet, waiting…");
    }
    else {
      number_hour1 = (newhum / 100U) % 10; //first digit of temp
      number_hour2 = (newhum / 10U) % 10; //second digit of temp
      number_min1 = 14; //code for "°"  %
      number_min2 = 14; //code for "°"  %
    }
  }

#endif
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
      setTime(newhours, newminutes, 0, 30, 12, 2020); //rtc.setTime(byte hours, byte minutes, byte seconds)
      RTC.set(now());
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
      delay(20);

      if (number_min2 == 10) {

        //animation every minute
        if (animationsetting == 1) {
          number_min2 = (minute() % 10); //second digit of the seconds
          digitbuffer = number_min2;
        }

        //animation every 10 minutes
        else if (animationsetting == 2) {
          number_min1 = (minute() / 10U) % 10;  //first digit of the seconds
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
