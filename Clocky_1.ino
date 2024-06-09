#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <MD_MAX72xx.h>
#include <Adafruit_SHT31.h>
#include "time.h"
#include "sntp.h"
#include <WiFiManager.h>
#include <Preferences.h>
#include "IRrecv.h"
#include "DHT.h"
#include <cmath> 
#include <BH1750.h>
#include <Wire.h>

#define DHTPIN 32     // Digital pin connected to the DHT sensor
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTTYPE DHT11   // DHT 11

#define  DELAYTIME  100  // in milliseconds
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES	4
#define CS_PIN    5  

#define SDA_PIN 32
#define SCL_PIN 33

#define BUZZER_PIN 22
#define PIR_PIN 21

const uint16_t RECV_PIN = 4; // for ESP32 micrcontroller

const unsigned long BUTTON_PRESS_0 = 0;
const unsigned long BUTTON_PRESS_1 = 1;
const unsigned long BUTTON_PRESS_2 = 2;
const unsigned long BUTTON_PRESS_3 = 3;
const unsigned long BUTTON_PRESS_4 = 4;
const unsigned long BUTTON_PRESS_5 = 5;
const unsigned long BUTTON_PRESS_6 = 6;
const unsigned long BUTTON_PRESS_7 = 7;
const unsigned long BUTTON_PRESS_8 = 8;
const unsigned long BUTTON_PRESS_9 = 9;
const unsigned long BUTTON_PRESS_OK = 10;
const unsigned long BUTTON_PRESS_STAR = 11;
const unsigned long BUTTON_PRESS_HASH = 12;
const unsigned long BUTTON_PRESS_UP = 13;
const unsigned long BUTTON_PRESS_DOWN = 14;
const unsigned long BUTTON_PRESS_LEFT = 15;
const unsigned long BUTTON_PRESS_RIGHT = 16;

const unsigned long REMOTE_BUTTONS[][17] = {
    {
        0xFF4AB5, 0xFF6897, 0xFF9867, 0xFFB04F, 0xFF30CF, 0xFF18E7,
        0xFF7A85, 0xFF10EF, 0xFF38C7, 0xFF5AA5, 0xFF02FD, 0xFF42BD,
        0xFF52AD, 0xFF629D, 0xFFA857, 0xFF22DD, 0xFFC23D
    },
    {
        0xFF9867, 0xFFA25D, 0xFF629D, 0xFFE21D, 0xFF22DD, 0xFF02FD,
        0xFFC23D, 0xFFE01F, 0xFFA857, 0xFF906F, 0xFF38C7, 0xFF6897,
        0xFFB04F, 0xFF18E7, 0xFF4AB5, 0xFF10EF, 0xFF5AA5
    }
};

int current_remote = 0; //TODO - make this configurable
int celsius; 
bool priorityDisplaySet = false;

// Define constants for button press messages
const char* BUTTON_PRESS_MSG_0 = "Pressed 0";
const char* BUTTON_PRESS_MSG_1 = "Pressed 1";
const char* BUTTON_PRESS_MSG_2 = "Pressed 2";
const char* BUTTON_PRESS_MSG_3 = "Pressed 3";
const char* BUTTON_PRESS_MSG_4 = "Pressed 4";
const char* BUTTON_PRESS_MSG_5 = "Pressed 5";
const char* BUTTON_PRESS_MSG_6 = "Pressed 6";
const char* BUTTON_PRESS_MSG_STAR = "Pressed *";
const char* BUTTON_PRESS_MSG_HASH = "Pressed #";
const char* BUTTON_PRESS_MSG_UP = "Pressed Up";
const char* BUTTON_PRESS_MSG_DOWN = "Pressed Down";
const char* BUTTON_PRESS_MSG_LEFT = "Pressed Left";
const char* BUTTON_PRESS_MSG_RIGHT = "Pressed Right";
const char* BUTTON_PRESS_MSG_DEFAULT = "Unexpected button press";

const char* DISPLAY_MSG_OFF = "Off";
const char* DISPLAY_MSG_TIME = "Time";
const char* DISPLAY_MSG_TEMPERATURE = "Temp";
const char* DISPLAY_MSG_HUMIDITY = "Humid";
const char* DISPLAY_MSG_DATE = "Date";
const char* DISPLAY_MSG_ALL = "ALL";
const char* DISPLAY_MSG_SCROLL = "Scroll";
const char* DISPLAY_MSG_DEMO = "Demo";
const char* DISPLAY_MSG_SETUP = "Setup";

const char* DISPLAY_MSG_24HR = "24 hr?";
const char* DISPLAY_MSG_12HR = "12 hr?";
const char* DISPLAY_MSG_ALARM = "Alarm";
const char* DISPLAY_MSG_ALARM_ON  = "On?";
const char* DISPLAY_MSG_ALARM_OFF  = "Off?";
const char* DISPLAY_MSG_ALARM_MOTION_ON  = "Alarm Motion On?";
const char* DISPLAY_MSG_ALARM_MOTION_OFF  = "Alarm Motion Off?";
const char* DISPLAY_MSG_PIR = "PIR";
const char* DISPLAY_MSG_LED_MOTION_ON  = "LED Motion On?";
const char* DISPLAY_MSG_LED_MOTION_OFF  = "LED Motion Off?";
const char* DISPLAY_MSG_12_24_HR = "12|24?";
const char* DISPLAY_MSG_WIFI = "Wi-Fi?";
const char* DISPLAY_MSG_RESET = "Reset?";
const char* DISPLAY_MSG_C_F = "C/F?";
const char* DISPLAY_MSG_C = "C?";
const char* DISPLAY_MSG_F = "F?";
const char* DISPLAY_MSG_ZONE = "Zone?";
const char* DISPLAY_MSG_TK = "Tokyo?";
const char* DISPLAY_MSG_UK = "UK?";
const char* DISPLAY_MSG_HK = "Hong Kong?";
const char* DISPLAY_MSG_EXIT = "Exit?";
const char* DISPLAY_MSG_OK = "OK";

//DisplayMode 
const int SHOW_NOTHING = 0;
const int SHOW_TIME = 1;
const int SHOW_DATE = 2;
const int SHOW_TEMPERATURE = 3;
const int SHOW_HUMIDITY = 4;
const int SHOW_ALL = 5;
const int SCROLL_ALL = 6;
const int DEMO_MODE = 7;
const int SETUP_MODE = 8;

const int SETUP_STEP_ALARM = 1;
const int SETUP_STEP_12_24 = 2;
const int SETUP_STEP_WIFI = 3;
const int SETUP_STEP_C_F = 4;
const int SETUP_STEP_PIR = 5;
const int SETUP_STEP_ZONE = 6;
const int SETUP_STEP_EXIT = 7;

const int SETUP_SUBSTEPS[8] = {0,3,2,1,2,2,3,0};

const int SETUP_MAX = 7; 

volatile bool motionDetected = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 6000; // Adjust debounce delay as needed

unsigned long lastMovementTime = 0;
unsigned long lastLightCheckTime = 0;
unsigned long lightCheckPeriod = 1000;
const unsigned long screensaverTimeout = 60000; // 1 minute
bool Screensaver = false;
 
int setupStep = 1;
int setupSubstep = 0;
int alarmOn;
int alarmMotionOn;
int LEDMotionOn;
bool alarmBeeping = false;

const int DEGREE_CHARACTER = 144;
const int SHT31 = 1;
const int AM2302 = 2;
const int temperature_sensor = SHT31;

// SPI hardware interface
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

Adafruit_SHT31 sht31; 
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;


Preferences preferences;

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
//const long  gmtOffset_sec = 8 * 3600;
//const int   daylightOffset_sec = 3600;
//const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)
char displayTime[10]=""; 
char displayDate[10]="";
char displayDay[10]="";
char displayTemperature[10]=""; 
char displayHumidity[10]="";
String alarmString="";
String timeZone; 

//String setupString = "";

struct tm timeinfo;


IRrecv irrecv(RECV_PIN);
decode_results results;

WiFiManager wifiManager; // Initialize WiFiManager

int currentMode;

int hour24 = 1;
int intensity;

TaskHandle_t DisplayTask_Handle, TriggerTask_Handle;

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
  strftime(displayTime, sizeof(displayTime), "%H:%M", &timeinfo);
}

void SetPriorityDisplay()
{
  priorityDisplaySet = true;
} 

void ReleasePriorityDisplay()
{
  priorityDisplaySet = false;
}

void getTime()
{
   if (hour24)
      strftime(displayTime, sizeof(displayTime), "%H:%M", &timeinfo);
   else
      strftime(displayTime, sizeof(displayTime), "%l:%M%", &timeinfo);
}

String getDatePostfix(const struct tm* timeinfo) {
    int date = timeinfo->tm_mday; // Extract the day of the month from the struct tm
    if (date >= 11 && date <= 13) {
        return "th";
    } else {
        switch (date % 10) {
            case 1: return "st";
            case 2: return "nd";
            case 3: return "rd";
            default: return "th";
        }
    }
}
void getDay(bool fullDay = false)
{
    if (fullDay)       
        strftime(displayDay, sizeof(displayDay), "%A", &timeinfo); // Format day of month
    else
        strftime(displayDay, sizeof(displayDay), "%a", &timeinfo); // Format day of month
 
}
void getDate(bool addPostfix = false)
{
    char monthPart[4];
        
    strftime(displayDate, sizeof(displayDate), "%e", &timeinfo); // Format day of month
    strftime(monthPart, sizeof(monthPart), "%b", &timeinfo); // Format day of month

    if (addPostfix) {
        String postfix = getDatePostfix(&timeinfo);
        strcat(displayDate, postfix.c_str()); // Append postfix
    }

    strcat(displayDate, " ");
    strcat(displayDate, monthPart);
}

float ConvertTemperatureToF(float temperature)
{
  if (std::isnan(temperature)) {
    return 0;
  }
  return (temperature*1.8)+32;
}

void getTemperature()
{
  if (temperature_sensor == SHT31)
  {
    if (celsius)
      sprintf(displayTemperature, "%.1f%cC", sht31.readTemperature(), DEGREE_CHARACTER);
    else
      sprintf(displayTemperature, "%.1f%cF", ConvertTemperatureToF(sht31.readTemperature()), DEGREE_CHARACTER);
  }
   else
  {
   if (celsius)
     sprintf(displayTemperature, "%.1f%cC", dht.readTemperature(), DEGREE_CHARACTER);
   else 
     sprintf(displayTemperature, "%.1f%cF", ConvertTemperatureToF(dht.readTemperature()), DEGREE_CHARACTER);
  }
}

void getHumidity()
{
  if (temperature_sensor == SHT31)
    sprintf(displayHumidity, "%.1f%%", sht31.readHumidity());
  else
    sprintf(displayHumidity, "%.1f%%", dht.readHumidity());

}

void getLight()
{
  if ((millis() - lastLightCheckTime) < lightCheckPeriod) return;
  
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  
  if (lux < 20)
    mx.control(MD_MAX72XX::INTENSITY, 0);
  else
    mx.control(MD_MAX72XX::INTENSITY, intensity);

  lastLightCheckTime = millis();
}

void CentreText(const char *p, bool PriorityDisplay = false)
{
  uint8_t charWidth;
  uint8_t totalCharWidth = 0;
  uint8_t cBuf[8];  // this should be ok for all built-in fonts
  int offset = 0;

  if (priorityDisplaySet && !PriorityDisplay)
    return;

  mx.clear();

  while (*p != '\0')
  {
    //put the character into cBuf and find the width
    charWidth = mx.getChar(*p++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
    totalCharWidth += charWidth;
    totalCharWidth += 1;
    mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

    for (uint8_t character_column=0; character_column<=charWidth; character_column++)	
    {
      if (character_column < charWidth) 
         mx.setColumn(31-offset, cBuf[character_column]);
      else 
         mx.setColumn(31-offset, 0);
  
      offset++;
    }
  }

  totalCharWidth -= 1;

  for (int x = 0; x < (32 - totalCharWidth)/2; x++)
    mx.transform(MD_MAX72XX::TSR);

  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  delay(DELAYTIME * 5);
}

void ScrollText(const char *p)
{
  uint8_t charWidth;
  uint8_t cBuf[8];  // this should be ok for all built-in fonts

  if (priorityDisplaySet) return; //don't get in the way of priority display updates

  mx.clear();
  
  int oldMode = currentMode;

  while (*p != '\0' && oldMode == currentMode && !priorityDisplaySet)
  {
    //put the character into cBuf and find the width
    charWidth = mx.getChar(*p++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
    for (uint8_t character_column=0; character_column<=charWidth; character_column++)	// allow space between characters
    {
      if (oldMode != currentMode)
        return;

      //shift the whole lot left by 1
      mx.transform(MD_MAX72XX::TSL);
 
      if (character_column < charWidth) 
         mx.setColumn(0, cBuf[character_column]);
      else 
         mx.setColumn(0, 0);
  
      delay(DELAYTIME);
    }
  }
}

bool randomSpot()
// Animation of a bouncing ball
{
  const int minC = 0;
  const int maxC = mx.getColumnCount()-1;
  const int minR = 0;
  const int maxR = ROW_SIZE-1;

  int  nCounter = 0;
  int  r = 0, c = 2;
  int oldMode = currentMode;
  
  mx.clear();

  while (nCounter++ < 200  && oldMode == currentMode)
  {
    mx.setPoint(r, c, false);
    r = random(minR, maxR);
    c = random(minC, maxC);
    mx.setPoint(r, c, true);
    delay(DELAYTIME/10);
  }
   return (oldMode == currentMode);
}

bool rain()
// Animation of a bouncing ball
{
  int colRowValue[33];
  int  nCounter = 0, c;
  int oldMode = currentMode;

  memset(colRowValue, -1, sizeof(colRowValue));
  
  mx.clear();
  
  while (nCounter++ < 350)
  {
    //start the rain at that column
    c = random(0, 32); //get the random column
    //Let's see if the randomly selected column has not started raining yet...
    if (colRowValue[c]<0 and nCounter++ < 300)
    {
       colRowValue[c]++;
       mx.setPoint(colRowValue[c], c, true); //start the rain
    }

    //loop through the other columns, increment if the rain has started
    for (int col = 0; col < 33; col++) {
        if (colRowValue[col]>3)
           mx.setPoint(colRowValue[col]-4, col, false); 

        if (colRowValue[col]>=0) //if the rain has started in this column...
        {
          //drop the raindrop one point further down
          colRowValue[col]++; 
          if (colRowValue[col]<8)
            mx.setPoint(colRowValue[col], col, true); 
          
          //This raindrop and it's tail are off the bottom of the screen
          if (colRowValue[col]>11) 
            colRowValue[col]=-1;
        }
        delay(1);
    }
  
  }
  return (oldMode == currentMode);
}





bool spectrum1()
{
    const int minC = 0;
    const int maxC = mx.getColumnCount();
    const int minR = 0;
    const int maxR = ROW_SIZE;
    const int FFT_SIZE= 8;

    int nCounter = 0;
    //int r = 0, c = 2;
  int oldMode = currentMode;


 //   mx.clear();

    while (nCounter++ < 20  && oldMode == currentMode)
    {
      // Simulate audio input (you can replace this with actual audio data)
        // For demonstration purposes, let's assume we have an array of magnitude values
        float spectrum[32];
        for (int i = 0; i < 32; ++i) {
            spectrum[i] = random(0, 7); // Replace with actual FFT magnitude values
        }
        mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
        mx.clear();

        for (int col = 32; col >=0 ; col--) {
          for (int row = 7; row >= spectrum[col]; row--)
          {
             mx.setPoint(row, col, true);
              delay(1);
          }
        }
        mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);

        delay(DELAYTIME/20);
    }
     return (oldMode == currentMode);

}

bool spectrum2()
{
    const int minC = 0;
    const int maxC = mx.getColumnCount();
    const int minR = 0;
    const int maxR = ROW_SIZE;
    const int FFT_SIZE= 8;

    int nCounter = 0;
    int r = 0, c = 2;

  int oldMode = currentMode;

    while (nCounter++ < 40  && oldMode == currentMode)
    {
        // Simulate audio input (you can replace this with actual audio data)
        // For demonstration purposes, let's assume we have an array of magnitude values
        float spectrum[FFT_SIZE];
        for (int i = 0; i < FFT_SIZE; ++i) {
            spectrum[i] = random(0, 255); // Replace with actual FFT magnitude values
        }

        // Update the display based on the spectrum
        for (int i = 0; i < FFT_SIZE; ++i) {
            mx.setColumn(c, 0x00);
            int barHeight = map(spectrum[i], 0, 255, minR, maxR);
                 mx.setPoint(barHeight, c, true);
                 delay(5);
  /*              if (j < barHeight-1)
                {
                  delay(5);
                  mx.setPoint(j, c, false);
                }  */
            // }
            c = (c + 1) % maxC; // Move to the next column
        }

     //   delay(DELAYTIME / 10);
    }
     return (oldMode == currentMode);

}

bool rows()
// Demonstrates the use of setRow()
{

  int oldMode = currentMode;
  
 // mx.clear();
  

  for (uint8_t row=0; row<ROW_SIZE; row++)
  { 
   if (oldMode != currentMode)
     break;
    mx.setRow(row, 0xff);
    delay(2*DELAYTIME);
    mx.setRow(row, 0x00);
  }
 return (oldMode == currentMode);


}

int8_t dR = 1, dC = 1;	// delta row and column
 int  r = 5, c = 5;

bool bounce()
// Animation of a bouncing ball
{
  const int minC = 0;
  const int maxC = mx.getColumnCount()-1;
  const int minR = 0;
  const int maxR = ROW_SIZE-1;

  int  nCounter = 0;

   
   int oldMode = currentMode;
   mx.clear();

  while (nCounter++ < 200 && oldMode == currentMode)
  {
    mx.setPoint(r, c, false);
    r += dR;
    c += dC;
    mx.setPoint(r, c, true);
    delay(DELAYTIME/2);

    if ((r == minR) || (r == maxR))
      dR = -dR;
    if ((c == minC) || (c == maxC))
      dC = -dC;
  }

  return (oldMode == currentMode);
}

bool bounce2()
// Animation of a bouncing ball
{
  const int minC = 0;
  const int maxC = mx.getColumnCount()-1;
  const int minR = 0;
  const int maxR = ROW_SIZE-1;

  int  nCounter = 0;

  //int  r = 0, c = 2;
 // int8_t dR = 1, dC = 1;	// delta row and column

   mx.clear();

  int delete_count = 0; 
  int oldMode = currentMode;
 
   while (nCounter++ < 200 && oldMode == currentMode)
  {
    if  (delete_count>3){
       mx.setPoint(r, c, false);
    }
   
    if  (delete_count>6)
      delete_count=0;

    if (dC>0 && c>=3)
       mx.setColumn(c-3, 0x00);

   if (dC<0 && c<29)
       mx.setColumn(c+3, 0x00);
 


    r += dR;
    c += dC;
    mx.setPoint(r, c, true);
    delay(DELAYTIME/2);

    if ((r == minR) || (r == maxR))
      dR = -dR;
    if ((c == minC) || (c == maxC))
      dC = -dC;

    delete_count++;  
  }
   return (oldMode == currentMode);

}

// Animation of a sine wave
bool sineWave()
{
  const int minC = 0;
  const int maxC = mx.getColumnCount() - 1;
  const int minR = 0;
  const int maxR = ROW_SIZE - 1;

  int oldMode = currentMode;
  mx.clear();

  float angle = 0.5;
  float angularVelocity = 0.8; // Adjust this value for the speed of the wave

  while (angle < TWO_PI * 10 && oldMode == currentMode)
  {
    mx.clear();

    for (int c = minC; c <= maxC; c+=4)
    {
      int r = 4 + (int)((maxR + 1) / 2 * sin(angle + c * 0.5)); // Adjust the multiplier for wave height
      mx.setPoint(r, c, true);
    }

    angle += angularVelocity;
    delay(DELAYTIME); // Adjust delay for animation speed
  }

  return (oldMode == currentMode);
}

void columns2(bool bClear)
// Demonstrates the use of setColumn()
{
   if (bClear)
    mx.clear();

  for (uint8_t col=mx.getColumnCount(); col>0; col--)
  {
    mx.setColumn(col, 0xff);
    delay(DELAYTIME/MAX_DEVICES);
    mx.setColumn(col, 0x00);
  }
}

void RightUp(const char *p)
{
  uint8_t charWidth;
  uint8_t totalCharWidth = 0;
  uint8_t cBuf[8];  // this should be ok for all built-in fonts
  int offset = 0;

  mx.clear();

  int oldMode = currentMode;

  while (*p != '\0' && oldMode == currentMode)
  {
    //put the character into cBuf and find the width
    charWidth = mx.getChar(*p++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
    totalCharWidth += charWidth;
    totalCharWidth += 1;
    for (uint8_t character_column=0; character_column<=charWidth; character_column++)	// allow space between characters
    {
      if (character_column < charWidth) 
         mx.setColumn(31-offset, cBuf[character_column]);
      else 
         mx.setColumn(31-offset, 0);
  
      offset++;

        delay(DELAYTIME);
    }
  }
  
  if (oldMode != currentMode) 
     return;
  
  delay(1000);
 
  //and scroll up
  totalCharWidth -= 1;
    for (int x = 0; x < 8; x++)
    {
      if (oldMode != currentMode) 
        return;

      mx.transform(MD_MAX72XX::TSU);
      delay(DELAYTIME);
    }
}

bool randomFill(bool clear, bool fill)
{
   const int minC = 0;
  const int maxC = mx.getColumnCount();
  const int minR = 0;
  const int maxR = ROW_SIZE;

  int oldMode = currentMode;

  int  nCounter = 0;

  int  r,c;
  
  if (clear)
    mx.clear();

  while (nCounter++ < 2000)
  {
    r = random(minR, maxR);
    c = random(minC, maxC);
    mx.setPoint(r, c, fill);
    delay(DELAYTIME/50);
  }

  return (oldMode == currentMode);

}

bool bullseye()
// Demonstrate the use of buffer based repeated patterns
// across all devices.
{
  int oldMode = currentMode;
  mx.clear();
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  for (uint8_t n=0; n<3; n++)
  {
    byte  b = 0xff;
    int   i = 0;

    while (b != 0x00)
    {
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
        mx.setRow(j, i, b);
        mx.setColumn(j, i, b);
        mx.setRow(j, ROW_SIZE-1-i, b);
        mx.setColumn(j, COL_SIZE-1-i, b);
      }
      mx.update();
      delay(3*DELAYTIME);
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
        mx.setRow(j, i, 0);
        mx.setColumn(j, i, 0);
        mx.setRow(j, ROW_SIZE-1-i, 0);
        mx.setColumn(j, COL_SIZE-1-i, 0);
      }

      bitClear(b, i);
      bitClear(b, 7-i);
      i++;
    }

    while (b != 0xff)
    {
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
        mx.setRow(j, i, b);
        mx.setColumn(j, i, b);
        mx.setRow(j, ROW_SIZE-1-i, b);
        mx.setColumn(j, COL_SIZE-1-i, b);
      }
      mx.update();
      delay(3*DELAYTIME);
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
        mx.setRow(j, i, 0);
        mx.setColumn(j, i, 0);
        mx.setRow(j, ROW_SIZE-1-i, 0);
        mx.setColumn(j, COL_SIZE-1-i, 0);
      }

      i--;
      bitSet(b, i);
      bitSet(b, 7-i);
    }
  }

  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  return (oldMode == currentMode);

}

bool randomDown()
{
  const int minC = 0;
  const int maxC = mx.getColumnCount()-1;
  const int minR = 0;
  const int maxR = ROW_SIZE-1;
  int colRowValue[33];
  int oldMode = currentMode;

  memset(colRowValue, -1, sizeof(colRowValue));

  int  nCounter = 0;

  int  r = 0, c = 2;
  int8_t dR = 1, dC = 1;	// delta row and column

  mx.clear();

  while (nCounter++ < 500 && oldMode == currentMode)
  {
    c = random(minC, maxC+1); //get the random column

    if (colRowValue[c]>=0)
       mx.setPoint(colRowValue[c], c, false);

    if (colRowValue[c]<7)
       colRowValue[c]++;

    mx.setPoint(colRowValue[c], c, true);
    delay(DELAYTIME/50);
  }
   delay(DELAYTIME *6);

    return (oldMode == currentMode);

}

bool cross()
// Combination of setRow() and setColumn() with user controlled
// display updates to ensure concurrent changes.
{
  int oldMode = currentMode;

  mx.clear();
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  // diagonally down the display R to L
  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      mx.setColumn(j, i, 0xff);
      mx.setRow(j, i, 0xff);
    }
    mx.update();
    delay(DELAYTIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      mx.setColumn(j, i, 0x00);
      mx.setRow(j, i, 0x00);
    }
  }

  // moving up the display on the R
  for (int8_t i=ROW_SIZE-1; i>=0; i--)
  {
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      mx.setColumn(j, i, 0xff);
      mx.setRow(j, ROW_SIZE-1, 0xff);
    }
    mx.update();
    delay(DELAYTIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      mx.setColumn(j, i, 0x00);
      mx.setRow(j, ROW_SIZE-1, 0x00);
    }
  }

  // diagonally up the display L to R
  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      mx.setColumn(j, i, 0xff);
      mx.setRow(j, ROW_SIZE-1-i, 0xff);
    }
    mx.update();
    delay(DELAYTIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      mx.setColumn(j, i, 0x00);
      mx.setRow(j, ROW_SIZE-1-i, 0x00);
    }
  }
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);

  return (oldMode == currentMode);

}


void Demo()
{
   if (!rows())
     return;

    getTime();
    CentreText(displayTime);
    ScreenDelay();

    if (!cross())
      return;
  
    ScreenDelay();
 
    getDay();
    CentreText(displayDay);
    if (!bullseye())
      return;
  
    getDate();
    CentreText(displayDate);
    ScreenDelay();

    if (!bounce())
      return;
    if (!bounce2())
      return;
    if(!spectrum1())
      return;
    if (!spectrum2())
      return;

    getTemperature();
    CentreText(displayTemperature);
    ScreenDelay();

    if (!randomSpot())
      return;
    if (!randomFill(true, true))
      return;
    if (!randomFill(false, false))
      return;

    getHumidity();
    CentreText(displayHumidity);
    ScreenDelay();

    if (!randomDown())
      return;
    if (!sineWave())
      return;
    if (!rain())
      return;
}

void SaveDisplayMode()
{ 
    preferences.begin("Settings", false);
    preferences.putInt("display_mode", currentMode); 
    Serial.println("display mode has been Saved");
    preferences.end();
}

void SaveAlarmString()
{ 
    preferences.begin("Settings", false);
    preferences.putString("alarm", alarmString); 
    Serial.println("Alarm been Saved");
    preferences.end();
}

void SaveTimeZone()
{ 
    preferences.begin("Settings", false);
    preferences.putString("timezone", timeZone); 
    Serial.println("timeZone has been Saved");
    preferences.end();
}

void SaveCurrentRemote()
{ 
    preferences.begin("Settings", false);
    preferences.putInt("remote_control", current_remote); 
    Serial.println("Current remote has been Saved");
    Serial.println(current_remote);
    preferences.end();
}

void SaveTemperatureScale()
{ 
    preferences.begin("Settings", false);
    preferences.putInt("celsius", celsius); 
    Serial.println("Temperature scale has been Saved");
    Serial.println(celsius);
    preferences.end();
}

void SaveIntensity()
{ 
    preferences.begin("Settings", false);
    preferences.putInt("intensity", intensity); 
    Serial.println("Intensity has been Saved");
    Serial.println(intensity);
    preferences.end();
}

void SaveAlarmOn()
{  
    preferences.begin("Settings", false);
    preferences.putInt("alarm_on", alarmOn); 
    Serial.println("alarmOn has been Saved");
    Serial.println(alarmOn);
    preferences.end();
}

void SaveAlarmMotionOn()
{  
    preferences.begin("Settings", false);
    preferences.putInt("alarm_motion_on", alarmMotionOn); 
    Serial.println("alarmMotionOn has been Saved");
    Serial.println(alarmMotionOn);
    preferences.end();
}

void SaveLEDMotionOn()
{  
    preferences.begin("Settings", false);
    preferences.putInt("motion_screen_on", LEDMotionOn); 
    Serial.println("LEDMotionOn has been Saved");
    Serial.println(LEDMotionOn);
    preferences.end();
}

void GetIntensity()
{ 
    preferences.begin("Settings", false);
    intensity = preferences.getInt("intensity", 0);
    mx.control(MD_MAX72XX::INTENSITY, intensity);
    preferences.end(); // Close the Preferences object  
}

void GetAlarmString()
{ 
    preferences.begin("Settings", false);
    alarmString = preferences.getString("alarm", "06:00"); 
    Serial.println("Alarm been retrieved -" + alarmString);
    preferences.end();
}

void GetTimeZone()
{ 
    preferences.begin("Settings", false);
    timeZone = preferences.getString("timezone", "JST-9"); 
    Serial.println("TimeZone been retrieved -" + timeZone);
    preferences.end();
}



void GetTemperatureScale()
{ 
    preferences.begin("Settings", false);
    celsius = preferences.getInt("celsius", 1);
    Serial.println("temperature_scale has been loaded");
    Serial.println(celsius);

    preferences.end();
}


void CentreTextPriority(const char * msg)
{
  SetPriorityDisplay();
  CentreText(msg, true);
  delay(500);
  ReleasePriorityDisplay();
}


void DisplayCurrentMode()
{
    switch (currentMode) {
       case SHOW_NOTHING:
         CentreTextPriority(DISPLAY_MSG_OFF);
         break;
       case SHOW_TIME:
          CentreTextPriority(DISPLAY_MSG_TIME);
          break;
       case SHOW_DATE:
          CentreTextPriority(DISPLAY_MSG_DATE);
          break;
       case SHOW_TEMPERATURE:
          CentreTextPriority(DISPLAY_MSG_TEMPERATURE);
          break;
       case SHOW_HUMIDITY:
          CentreTextPriority(DISPLAY_MSG_HUMIDITY);
          break;
     case SHOW_ALL:
          CentreTextPriority(DISPLAY_MSG_ALL);
          break;
    case SCROLL_ALL:
          CentreText(DISPLAY_MSG_SCROLL);
          break;
    case DEMO_MODE:
          CentreTextPriority(DISPLAY_MSG_DEMO);
          break;
    case SETUP_MODE:
          CentreTextPriority(DISPLAY_MSG_SETUP);
          break;
    }
}

void SetSaveandDisplayMode(int newMode, bool SaveMode = true)
{
   currentMode = newMode;
   if (SaveMode)
     SaveDisplayMode();
   
   DisplayCurrentMode();
}

bool ValidateAlarmStringAddition(String newChar)
{
  if (alarmString.length() == 0) {
    switch (newChar.charAt(0)) { // Consider only the first character
      case '0':
      case '1':
      case '2':
        return true;
      default:
       Beep();
       return false;
    }
  }
  
  if (alarmString.length() == 1) {
    if(alarmString.equals("0") || alarmString.equals("1"))
       return true; //any number allowed
 
    switch (newChar.charAt(0)) { // Consider only the first character
      case '0':
      case '1':
      case '2':
      case '3':
        return true;
      default:
      Beep();
      return false;
    }
  }
  
  if (alarmString.length() == 2) {
    switch (newChar.charAt(0)) { // Consider only the first character
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
        return true;
      default:
      Beep();
      return false;
    }
  }

  if (alarmString.length() == 4 || alarmString.length() == 5) 
    return true; //any number is allowed
 
 
  return false; // Return false if alarmString is not empty

}

void AddToAlarmString(String newChar)
{
  
  if (setupStep!=SETUP_STEP_ALARM || setupSubstep!=1)
    return; 
  
  if(!ValidateAlarmStringAddition(newChar)) return; 

  if(alarmString.length()==2) //add the : as well..
    alarmString+=":";

  if(alarmString.length()<5)
  {
    alarmString+=newChar;
    if(alarmString.length()==5) //add the ? as well..
      alarmString+="?";
  }
  else //must be 5 in length e.g. 12:00
    alarmString=newChar; //so we want to set a brand new alarm time
}

void ShowOK()
{
  SetPriorityDisplay(); 
  CentreText(DISPLAY_MSG_OK, true);
  delay(1000); // Delay between scrolls or updates 
  ReleasePriorityDisplay(); 
}

void DeleteFromAlarmString()
{
  if(alarmString.length()>0)
  {
    alarmString.remove(alarmString.length() - 1); // Removes the last character from the string
  }
  else
  {
    Beep();
    return;
  }

  if(alarmString.length()==3 || alarmString.length()==6) //remove the : or ? as well..
  {
    alarmString.remove(alarmString.length() - 1); // Removes the last character from the string
  }
}

bool isValueInArray(unsigned long value) {
    for (size_t j = 0; j < 17; j++) {
        if (REMOTE_BUTTONS[current_remote][j] == value) {
            return true; // Value found in the array
        }
    }
    return false; // Value not found in the array
}


void checkIR() {
    char buffer[10]; // Assuming the number won't exceed 10 characters including the sign and null terminator

    if (irrecv.decode(&results)) {
        //    if (results.value >> 32)  // print() & println() can't handle printing long longs. (uint64_t)
        //      Serial.print((uint32_t) (results.value >> 32), HEX);  // print the first part of the message
        Serial.println((uint32_t)(results.value & 0xFFFFFFFF), HEX); // print the second part of the message
        
        if(alarmBeeping)
        {
            if (isValueInArray(results.value)) //Is it a recognised IR code (from the Clocky remote control)
            {
              Serial.print("Switching off alarm");  
              alarmBeeping=false;
              alarmOn=0;
              SaveAlarmOn();
              irrecv.resume(); // Receive the next value
              return; // - no need to process this IR key press
            }
        }

        if (currentMode == SETUP_MODE)
        {
          if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_STAR])
          {
            GetDisplayMode(); //restore the previous mode
            GetAlarmString(); //restore the previous alarm string
          }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_UP])
          {
            if (setupStep < SETUP_MAX) 
              setupStep++;
            else  
              Beep(); 

            setupSubstep=0;  
          }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_DOWN])
          {
            if (setupStep > 1) 
              setupStep--;
            else  
              Beep(); 

            setupSubstep=0;  
          }
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_OK])
          {
           if (setupStep==SETUP_STEP_ALARM)
           {
              if (setupSubstep==1)
              {
                if(alarmString.length() == 6)
                {
                  alarmString.remove(alarmString.length() - 1); // Removes the last character '?' from the string
                  SaveAlarmString();
                  alarmOn=1;  //Turn on the alarm
                  SaveAlarmOn();
                  ShowOK();
                } 
                else
                  Beep();
              }
              else if (setupSubstep==2)
              {
                if (alarmOn==1)
                  alarmOn=0;
                else
                  alarmOn=1;
               
                SaveAlarmOn();
                ShowOK();
              }
            else if (setupSubstep==3)
              {
                if (alarmMotionOn==1)
                  alarmMotionOn=0;
                else
                  alarmMotionOn=1;
               
                SaveAlarmMotionOn();
                ShowOK();
              }
           }
           else if (setupStep==SETUP_STEP_12_24)
              {
                if (setupSubstep==1)
                {
                  hour24 = 0;
                  ShowOK();
                }
                else if (setupSubstep==2)
                {
                  hour24 = 1;
                  ShowOK();
                }
              }
              else if (setupStep==SETUP_STEP_WIFI && setupSubstep==1)
              {
                 wifiManager.resetSettings();
                 ShowOK();
                 ESP.restart();
              }
            else if (setupStep==SETUP_STEP_C_F)
              {
                 if (setupSubstep==1)
                   celsius = 1;
                 else if (setupSubstep==2)
                   celsius = 0;
                   
                 SaveTemperatureScale();
                 ShowOK();
              }
        else if (setupStep==SETUP_STEP_PIR)
              {
                 if (setupSubstep==1) //Alarm Off with PIR
                 {
                   if (alarmMotionOn==1)
                     alarmMotionOn=0;
                   else
                     alarmMotionOn=1;
               
                   SaveAlarmMotionOn();
                   ShowOK();
                 }
                 else if (setupSubstep==2) //Screen On with PIR
                 {
                  if (LEDMotionOn==1)
                     LEDMotionOn=0;
                   else
                     LEDMotionOn=1;
               
                   SaveLEDMotionOn();
                   ShowOK();
                 }
                   
                 SaveTemperatureScale();
                 ShowOK();
              }
           else if (setupStep==SETUP_STEP_ZONE)
              {
                 if (setupSubstep==1) //Tokyo
                 {
                   SetTimezone("JST-9"); 
                   SaveTimeZone(); 
                 }
                 else if (setupSubstep==2) //UK
                 {
                   SetTimezone("GMT0BST,M3.5.0/1,M10.5.0"); 
                   SaveTimeZone(); 
                 }
                 else if (setupSubstep==3) //HK
                 {
                   SetTimezone("HKT-8"); 
                   SaveTimeZone(); 
                 }
                   
         //        SaveTemperatureScale();
                 ShowOK();
              }
         else if (setupStep==SETUP_STEP_EXIT)
              {
                 GetDisplayMode(); //restore the previous mode
                 GetAlarmString(); //restore the previous alarm string
                 ShowOK();
              }
          }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_LEFT])
          {
             if (setupSubstep > 0)
              setupSubstep--; 
             else
               Beep();    
          }
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_RIGHT])
         {
           if (setupSubstep < SETUP_SUBSTEPS[setupStep])
             setupSubstep++;
           else
             Beep();    
         }
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_0])
            AddToAlarmString("0");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_1])
            AddToAlarmString("1");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_2])
            AddToAlarmString("2");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_3])
            AddToAlarmString("3");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_4])
            AddToAlarmString("4");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_5])
            AddToAlarmString("5");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_6])
            AddToAlarmString("6");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_7])
            AddToAlarmString("7");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_8])
            AddToAlarmString("8");
         else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_9])
            AddToAlarmString("9");
         else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_HASH])
            DeleteFromAlarmString();
        }
        else //We are not in settings mode
        {
          // Check for button presses using switch statements
          if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_0])
            SetSaveandDisplayMode(SHOW_NOTHING);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_1])
            SetSaveandDisplayMode(SHOW_TIME);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_2])
            SetSaveandDisplayMode(SHOW_DATE);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_3])
            SetSaveandDisplayMode(SHOW_TEMPERATURE);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_4])
            SetSaveandDisplayMode(SHOW_HUMIDITY);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_5])
            SetSaveandDisplayMode(SHOW_ALL);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_6])
            SetSaveandDisplayMode(SCROLL_ALL);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_9])
            SetSaveandDisplayMode(DEMO_MODE);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_STAR])
          {
            SetPriorityDisplay();
            currentMode = SETUP_MODE; 
            setupStep=1;
            setupSubstep=0;
            CentreText(DISPLAY_MSG_SETUP, true);
            delay(1000);
            ReleasePriorityDisplay(); 
         }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_UP])
          {
                  if (intensity < 10) {
                      intensity++;
                      mx.control(MD_MAX72XX::INTENSITY, intensity);
                      SaveIntensity();
                  } 
                  else 
                    Beep();
                    
                  delay(1000); // Delay between scrolls or updates
                  ReleasePriorityDisplay(); 
           }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_DOWN])
            {
                  if (intensity > 0) {
                      intensity--;
                      mx.control(MD_MAX72XX::INTENSITY, intensity);
                      SaveIntensity();
                  } 
                  else 
                    Beep();
              }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_LEFT])
            {
                    if (currentMode > 0){
                      SetSaveandDisplayMode(currentMode-1);
                    } 
                    else
                      SetSaveandDisplayMode(6);
            }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_RIGHT])
            {
                    if (currentMode < 6) 
                      SetSaveandDisplayMode(currentMode+1);
                    else
                      SetSaveandDisplayMode(0);
            }
        else
        {
              Serial.println(BUTTON_PRESS_MSG_DEFAULT);
        }  
     }
     irrecv.resume(); // Receive the next value
    }
}

void ScreenDelay()
{
  if (currentMode == SHOW_ALL)
    delay(500); // Delay between scrolls or updates
  else
    delay(1000); // Delay between scrolls or updates
}


// Task function for Task1
void DisplayTask(void *pvParameters) {
 
  ScrollText("Welcome to Clocky!");
  delay(500);
  columns2(false);
  
  for (;;) {
 
    if (WiFi.status() == WL_CONNECTED) //Should probably put some kind of reconnect in here? TODO
    {
      if (!getLocalTime(&timeinfo)) 
        return;
    }    

    if (currentMode == SHOW_NOTHING)
         mx.clear();
    
    if (Screensaver && currentMode != SETUP_MODE)
    {
      mx.clear();
      delay(1000); // Delay between scrolls or updates
      continue;
    } 
    
    if (currentMode == SHOW_TIME || currentMode == SHOW_DATE || currentMode == SHOW_ALL || currentMode == SCROLL_ALL)
    {
        if (currentMode == SHOW_TIME || currentMode == SHOW_ALL){
          getTime();
          CentreText(displayTime);
          ScreenDelay();
        }

        if (currentMode == SHOW_DATE || currentMode == SHOW_ALL){
          getDay(false);
          CentreText(displayDay);
          ScreenDelay();
        }
          
        if (currentMode == SHOW_DATE || currentMode == SHOW_ALL){ //check the mode again, in case it has changed
          getDate();
          CentreText(displayDate);
          ScreenDelay();
        }
    }  

    if (currentMode == SHOW_TEMPERATURE || currentMode == SHOW_ALL){
      getTemperature();
      CentreText(displayTemperature);
      ScreenDelay();
    }

    if (currentMode == SHOW_HUMIDITY || currentMode == SHOW_ALL){
       getHumidity();
       CentreText(displayHumidity);
       ScreenDelay();
    }
    else if (currentMode == DEMO_MODE)
       Demo();
    else if (currentMode == SETUP_MODE){
      if (setupStep==SETUP_STEP_ALARM)
      {
        if (setupSubstep==0)
          CentreText(DISPLAY_MSG_ALARM);
        else if (setupSubstep==1)
          CentreText(alarmString.c_str());
        else if (setupSubstep==2)
        {
          if(alarmOn)
            CentreText(DISPLAY_MSG_ALARM_OFF);      
          else 
            CentreText(DISPLAY_MSG_ALARM_ON);      

        } 
        else if (setupSubstep==3)
        {
          if(alarmMotionOn)
            ScrollText(DISPLAY_MSG_ALARM_MOTION_OFF);      
          else 
            ScrollText(DISPLAY_MSG_ALARM_MOTION_ON);      
        } 
      }
      else if (setupStep==SETUP_STEP_12_24)
      {
        if (setupSubstep==0)
          CentreText(DISPLAY_MSG_12_24_HR);
        else if (setupSubstep==1)
          CentreText(DISPLAY_MSG_12HR);
        else if (setupSubstep==2)
          CentreText(DISPLAY_MSG_24HR);
      }
      else if (setupStep==SETUP_STEP_WIFI)
      {
        if (setupSubstep==0)
          CentreText(DISPLAY_MSG_WIFI);
        else if (setupSubstep==1)
          CentreText(DISPLAY_MSG_RESET);
       }
      else if (setupStep==SETUP_STEP_C_F)
      {
       if (setupSubstep==0)
         CentreText(DISPLAY_MSG_C_F);
       else if (setupSubstep==1)
         CentreText(DISPLAY_MSG_C);
       else if (setupSubstep==2)
         CentreText(DISPLAY_MSG_F);
       }
      else if (setupStep==SETUP_STEP_PIR)
      {
        if (setupSubstep==0)
          CentreText(DISPLAY_MSG_PIR);
        else if (setupSubstep==1)
        {
           if(alarmMotionOn)
            ScrollText(DISPLAY_MSG_ALARM_MOTION_OFF);      
          else 
            ScrollText(DISPLAY_MSG_ALARM_MOTION_ON);      
        } 
        else if (setupSubstep==2)
        {
          if(LEDMotionOn)
            ScrollText(DISPLAY_MSG_LED_MOTION_OFF);      
          else 
            ScrollText(DISPLAY_MSG_LED_MOTION_ON);         
        }
      }
      else if (setupStep==SETUP_STEP_ZONE)
      {
        if (setupSubstep==0)
          CentreText(DISPLAY_MSG_ZONE);
        else if (setupSubstep==1)
          CentreText(DISPLAY_MSG_TK);
        else if (setupSubstep==2)
          CentreText(DISPLAY_MSG_UK);
        else if (setupSubstep==3)
          ScrollText(DISPLAY_MSG_HK);
      }
     else if (setupStep==SETUP_STEP_EXIT)
      {
        CentreText(DISPLAY_MSG_EXIT);
      }
   } 
    else if (currentMode == SCROLL_ALL){
        char scrollBuffer[50];  
        if (!getLocalTime(&timeinfo)) {
          Serial.println("No time available (yet)");
          return;
        }
        getTime();
        getDate(true);
        getDay(true);
        getTemperature();
        getHumidity();
 
        sprintf(scrollBuffer, "%s %s %s %s %s", displayTime, displayDay, displayDate, displayTemperature, displayHumidity); 
        mx.clear();
        ScrollText(scrollBuffer);
        delay(100); // Delay between scrolls or updates
    } 
   
    getLight();

    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for a bit
  }
}

// Task function for Task2 - Trigger task - triggered by IR remote, PIR
void TriggerTask(void *pvParameters) {
  irrecv.enableIRIn();
  for (;;) {
    checkIR();
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for a bit

    if (alarmOn==1 && strcmp(displayTime, alarmString.c_str()) == 0) 
      alarmBeeping=true; //switch on the alarm beeping

    if (alarmBeeping)
    {
      Serial.println("Alarm beeping");
      Beep();
    }
  
    if (motionDetected) 
    {
      motionDetected = false;
      lastMovementTime = millis();
      Serial.println("Motion detected!");
      // Handle motion detection event here
      if (alarmMotionOn==1 && alarmBeeping)
      {
        alarmOn=0;
        SaveAlarmOn();
        alarmBeeping=false;
      }
    }
    
    //Check light level and adjust displayt if required
    getLight();

    // Check if screensaver should be activated
    Screensaver = (LEDMotionOn==1 && ((millis() - lastMovementTime) > screensaverTimeout));

  }
}

void GetDisplayMode()
{ 
    preferences.begin("Settings", false);
    currentMode = preferences.getInt("display_mode", SHOW_ALL);
    preferences.end(); // Close the Preferences object  
}

void GetCurrentRemote()
{ 
    preferences.begin("Settings", false);
    current_remote = preferences.getInt("remote_control", 0);
    preferences.end(); // Close the Preferences object  
}

void GetAlarmOn()
{ 
    preferences.begin("Settings", false);
    alarmOn = preferences.getInt("alarm_on", 0);
    preferences.end(); // Close the Preferences object  
}

void GetAlarmMotionOn()
{ 
    preferences.begin("Settings", false);
    alarmMotionOn = preferences.getInt("alarm_motion_on", 0);
    preferences.end(); // Close the Preferences object  
}

void GetLEDMotionOn()
{ 
    preferences.begin("Settings", false);
    LEDMotionOn = preferences.getInt("motion_screen_on", 0);
    preferences.end(); // Close the Preferences object  
}

void SetTimezone(String timezone){
  timeZone = timezone;
  Serial.printf("  Setting Timezone to %s\n",timezone.c_str());
  setenv("TZ",timezone.c_str(),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

void Beep()
{
   digitalWrite(BUZZER_PIN, HIGH);  // turn the LED on (HIGH is the voltage level)
   delay(100);                      // wait for a second
   digitalWrite(BUZZER_PIN, LOW);   // turn the LED off by making the voltage LOW
}

void IRAM_ATTR detectsMovement() {
    unsigned long currentTime = millis();
    if ((currentTime - lastDebounceTime) > debounceDelay) 
        motionDetected = true;
    }

void setup() {
  Serial.begin(9600);
//  GetCurrentRemote();
  GetTimeZone();
  GetAlarmString();
  GetAlarmOn();
  GetAlarmMotionOn();
  GetLEDMotionOn();
  GetDisplayMode();
  GetTemperatureScale();

  pinMode(BUZZER_PIN, OUTPUT);
 
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(PIR_PIN, INPUT_PULLDOWN);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), detectsMovement, RISING);


  Serial.println("Initializing I2C...");
  Wire.begin(SDA_PIN, SCL_PIN);
 
  lightMeter.begin();

  if (temperature_sensor == SHT31)
  {
    sht31 = Adafruit_SHT31();
    Serial.println("Initializing SHT31 sensor...");
    if (!sht31.begin(0x44)) {
      Serial.println("Couldn't find SHT31 sensor, check wiring!");
      while (1);
    }
  }  
  else
    dht.begin();

  if (!mx.begin())
    Serial.println("\nMD_MAX72XX initialization failed");

  GetIntensity();

  CentreText("Wi-fi");
 
  wifiManager.autoConnect("Clocky"); // Connect to saved network or create AP
 
 // set notification call-back function
  sntp_set_time_sync_notification_cb( timeavailable );
  sntp_servermode_dhcp(1);    // (optional)
  configTime(0, 0, ntpServer1, ntpServer2);
  SetTimezone(timeZone); //https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv


  xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 4096, NULL, 1, &DisplayTask_Handle, 0);
  xTaskCreatePinnedToCore(TriggerTask, "TriggerTask", 4096, NULL, 2, &TriggerTask_Handle, 1);

   Beep();  
}

void loop() {
  // Main loop can be used for non-time-critical tasks
}
