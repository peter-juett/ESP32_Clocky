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
#include <cmath> 
#include <BH1750.h>
#include <Wire.h>

#define DELAY_TIME  100  // in milliseconds
#define SCROLL_DELAY_TIME 50
#define SCREEN_DELAY_TIME  500  // in milliseconds
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

const char* BUTTON_PRESS_MSG_DEFAULT = "Unexpected button press";

const char* DISPLAY_MSG_OFF = "Off";
const char* DISPLAY_MSG_TIME = "Time";
const char* DISPLAY_MSG_TEMPERATURE = "Temp";
const char* DISPLAY_MSG_HUMIDITY = "Humid";
const char* DISPLAY_MSG_DATE = "Date";
const char* DISPLAY_MSG_ALL = "ALL";
const char* DISPLAY_MSG_ALL2 = "ALL2";
const char* DISPLAY_MSG_SCROLL = "Scroll";
const char* DISPLAY_MSG_DEMO = "Demo";
const char* DISPLAY_MSG_LIGHT = "Light";

const char* DISPLAY_MSG_SETUP = "Setup";

const char* DISPLAY_MSG_24HR = "24 hr?";
const char* DISPLAY_MSG_12HR = "12 hr?";
const char* DISPLAY_MSG_ALARM = "Alarm";
const char* DISPLAY_MSG_ALARM_ON  = "On?";
const char* DISPLAY_MSG_ALARM_OFF  = "Off?";
const char* DISPLAY_MSG_ALARM_SCREEN_ON  = "Motion+?";
const char* DISPLAY_MSG_ALARM_MOTION_OFF  = "Motion-?";
const char* DISPLAY_MSG_PIR = "Display";
const char* DISPLAY_MSG_DISPLAY_MOTION_ON  = "Mtn+?";
const char* DISPLAY_MSG_DISPLAY_MOTION_OFF  = "Mtn-?";
const char* DISPLAY_MSG_DISPLAY_MOTION_DARK  = "Mtndk?";
const int DISPLAY_MOTION_OFF = 0;
const int DISPLAY_MOTION_ON = 1;
const int DISPLAY_MOTION_DARK = 2;

const int LIGHT_LEVEL_DARK = 1;
const int LIGHT_LEVEL_LOW = 20;

const char* DISPLAY_MSG_12_24_HR = "12/24";
const char* DISPLAY_MSG_WIFI = "Wi-Fi";
const char* DISPLAY_MSG_RESET = "Reset?";
const int DEGREE_CHARACTER = 144;
const char* DISPLAY_MSG_ZONE = "TZone";
const char* DISPLAY_MSG_TK = "Tokyo?";
const char* DISPLAY_MSG_UK = "UK?";
const char* DISPLAY_MSG_HK = "HK?";
const char* DISPLAY_MSG_EXIT = "Exit?";
const char* DISPLAY_MSG_OK = "OK";

//DisplayMode 
const int SHOW_NOTHING = 0;
const int SHOW_TIME = 1;
const int SHOW_DATE = 2;
const int SHOW_TEMPERATURE = 3;
const int SHOW_HUMIDITY = 4;
const int SHOW_ALL = 5;
const int SHOW_ALL2 = 6;
const int SCROLL_ALL = 7;
const int SHOW_LIGHT = 8;
const int DEMO_MODE = 9;

const int SETUP_MODE = 10;

const int SETUP_STEP_ALARM = 1;
const int SETUP_STEP_ALARM_SUBSTEP_0_DISPLAY_ALARM = 0;
const int SETUP_STEP_ALARM_SUBSTEP_1_EDIT_TIME = 1;
const int SETUP_STEP_ALARM_SUBSTEP_2_ON_OFF = 2;
const int SETUP_STEP_ALARM_SUBSTEP_3_MOTION_OFF = 3;

const int SETUP_STEP_12_24 = 2;
const int SETUP_STEP_WIFI = 3;
const int SETUP_STEP_C_F = 4;
const int SETUP_STEP_DISPLAY = 5;
const int SETUP_STEP_ZONE = 6;
const int SETUP_STEP_EXIT = 7;

const int SETUP_SUBSTEPS[8] = {0,3,1,1,1,1,3,0};

const int SETUP_MAX = 7; 

volatile bool motionDetected = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 6000; // Adjust debounce delay as needed
bool keyHit = false;

unsigned long lastMovementTime = 0;
unsigned long lastLightCheckTime = 0;
unsigned long lightCheckPeriod = 1000;
const unsigned long screenSaverTimeout = 60000; // 1 minute
bool screenSaver = false;
float lightLevel = 0;
 
int setupStep = 1;
int setupSubstep = 0;
int alarmOn;
int alarmMotionOn;
int DisplayMotionOn;
bool alarmBeeping = false;


// SPI hardware interface
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

Adafruit_SHT31 sht31; 

BH1750 lightMeter;

Preferences preferences;

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";

char displayTime[10]=""; 
String alarmString="";
String timeZone; 
int currentMode;
int hour24;
int intensity;

struct tm timeinfo;

IRrecv irrecv(RECV_PIN);

WiFiManager wifiManager; // Initialize WiFiManager

TaskHandle_t DisplayTask_Handle, TriggerTask_Handle;

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
  strftime(displayTime, sizeof(displayTime), "%H:%M", &timeinfo);
}

void SetPriorityDisplay(bool on)
{
  priorityDisplaySet = on;
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
String getDay(bool fullDay = false)
{
    char dayBuffer[20]; // Sufficient size for full day name or abbreviated day name
    
    if (fullDay)       
        strftime(dayBuffer, sizeof(dayBuffer), "%A", &timeinfo); // Full day name
    else
        strftime(dayBuffer, sizeof(dayBuffer), "%a", &timeinfo); // Abbreviated day name
    
    return String(dayBuffer);
 
}
String getDate(bool addPostfix = false)
{
    char monthPart[4];
    String displayDate;

    // Format day of month
    char dayPart[3];
    strftime(dayPart, sizeof(dayPart), "%e", &timeinfo);
    displayDate = String(dayPart);

    // Format month part
    strftime(monthPart, sizeof(monthPart), "%b", &timeinfo);

    // Append postfix if needed
    if (addPostfix) {
        String postfix = getDatePostfix(&timeinfo);
        displayDate += postfix;
    }

    // Append space and month part
    displayDate += " ";
    displayDate += String(monthPart);

    // Return displayDate
    return displayDate;
}


float ConvertTemperatureToF(float temperature)
{
  if (std::isnan(temperature)) {
    return 0;
  }
  return (temperature*1.8)+32;
}

String getTemperature()
{
  char displayTemperature[10]; 

  if (celsius==1)
    sprintf(displayTemperature, "%.1f%cC", sht31.readTemperature(), DEGREE_CHARACTER);
  else
    sprintf(displayTemperature, "%.0f%cF", ConvertTemperatureToF(sht31.readTemperature()), DEGREE_CHARACTER);

  return String(displayTemperature);
}

String getHumidity()
{
  return String(sht31.readHumidity(), 1) + "%";
}

void getLight()
{
  if ((millis() - lastLightCheckTime) < lightCheckPeriod) return;
  
  lightLevel = lightMeter.readLightLevel();

 // Serial.println(lightLevel);

  if (lightLevel < LIGHT_LEVEL_LOW)
    mx.control(MD_MAX72XX::INTENSITY, 0);
  else
    mx.control(MD_MAX72XX::INTENSITY, intensity);

  lastLightCheckTime = millis();
}

bool IsDark()
{
  return (lightLevel < LIGHT_LEVEL_DARK);
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
  delay(DELAY_TIME * 5);
}

void CentreText(const String &p, bool PriorityDisplay = false)
{
    uint8_t charWidth;
    uint8_t totalCharWidth = 0;
    uint8_t cBuf[8];  // This should be ok for all built-in fonts
    int offset = 0;

    if (priorityDisplaySet && !PriorityDisplay)
        return;

    mx.clear();

    for (unsigned int i = 0; i < p.length(); i++)
    {
        // Put the character into cBuf and find the width
        charWidth = mx.getChar(p[i], sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
        totalCharWidth += charWidth;
        totalCharWidth += 1;
        mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

        for (uint8_t character_column = 0; character_column <= charWidth; character_column++)
        {
            if (character_column < charWidth)
                mx.setColumn(31 - offset, cBuf[character_column]);
            else
                mx.setColumn(31 - offset, 0);

            offset++;
        }
    }

    totalCharWidth -= 1;

    for (int x = 0; x < (32 - totalCharWidth) / 2; x++)
        mx.transform(MD_MAX72XX::TSR);

    mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
    delay(DELAY_TIME * 5);
}

void ScrollText(const char *p, bool slow = false)
{
  uint8_t charWidth;
  uint8_t cBuf[8];  // this should be ok for all built-in fonts

  if (priorityDisplaySet) return; //don't get in the way of priority display updates
  
  keyHit=false; //reset the IR Key hit

  mx.clear();
  
  int oldMode = currentMode;

  while (*p != '\0' && oldMode == currentMode && !priorityDisplaySet)
  {
    //put the character into cBuf and find the width
    charWidth = mx.getChar(*p++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
    for (uint8_t character_column=0; character_column<=charWidth; character_column++)	// allow space between characters
    {
      if (keyHit) //If a valid command was received then leave the scrolling
      {
        keyHit=false;
        return;
      }

      //shift the whole lot left by 1
      mx.transform(MD_MAX72XX::TSL);
 
      if (character_column < charWidth) 
         mx.setColumn(0, cBuf[character_column]);
      else 
         mx.setColumn(0, 0);
  
      if (slow)
        delay(SCROLL_DELAY_TIME*2);
      else   
        delay(SCROLL_DELAY_TIME);
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
    delay(DELAY_TIME/10);
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
            if (crossCheckBreak(oldMode)) return false;
            mx.setPoint(row, col, true);
            delay(1);
          }
        }
        mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);

        delay(DELAY_TIME/20);
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

   // mx.clear();

    while (nCounter++ < 80  && oldMode == currentMode)
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
            if (crossCheckBreak(oldMode)) return false;
            int barHeight = map(spectrum[i], 0, 255, minR, maxR);
            mx.setPoint(barHeight, c, true);
            delay(5);
            c = (c + 1) % maxC; // Move to the next column
        }

     //   delay(DELAY_TIME / 10);
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
    delay(2*DELAY_TIME);
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
    delay(DELAY_TIME/2);

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
    delay(DELAY_TIME/2);

    if ((r == minR) || (r == maxR))
      dR = -dR;
    if ((c == minC) || (c == maxC))
      dC = -dC;

    delete_count++;  
  }
   return (oldMode == currentMode);

}

void dot(bool on)
{
  mx.setPoint(6, 1, on);
}

void AlarmDot(bool on)
{
  mx.setPoint(6, 30, on);
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
    delay(DELAY_TIME); // Adjust delay for animation speed
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
    delay(DELAY_TIME/MAX_DEVICES);
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

        delay(DELAY_TIME);
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
      delay(DELAY_TIME);
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
    delay(DELAY_TIME/50);
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

  for (uint8_t n=0; n<2; n++)
  {
    byte  b = 0xff;
    int   i = 0;

    while (b != 0x00)
    {
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {    
        if (crossCheckBreak(oldMode)) return false;
        mx.setRow(j, i, b);
        mx.setColumn(j, i, b);
        mx.setRow(j, ROW_SIZE-1-i, b);
        mx.setColumn(j, COL_SIZE-1-i, b);
      }
      mx.update();
      delay(3*DELAY_TIME);
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
         if (crossCheckBreak(oldMode)) return false;
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
         if (crossCheckBreak(oldMode)) return false;
        mx.setRow(j, i, b);
        mx.setColumn(j, i, b);
        mx.setRow(j, ROW_SIZE-1-i, b);
        mx.setColumn(j, COL_SIZE-1-i, b);
      }
      mx.update();
      delay(3*DELAY_TIME);
      for (uint8_t j=0; j<MAX_DEVICES+1; j++)
      {
         if (crossCheckBreak(oldMode)) return false;
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
    delay(DELAY_TIME/50);
  }
   delay(DELAY_TIME *6);

    return (oldMode == currentMode);

}

bool crossCheckBreak(int old_mode)
{
  if (old_mode != currentMode)
  {
    mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
    return true;
  }
  return false;
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
      if (crossCheckBreak(oldMode)) return false;
      mx.setColumn(j, i, 0xff);
      mx.setRow(j, i, 0xff);
    }
    mx.update();
    delay(DELAY_TIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      if (crossCheckBreak(oldMode)) return false;
      mx.setColumn(j, i, 0x00);
      mx.setRow(j, i, 0x00);
    }

     if (crossCheckBreak(oldMode)) return false;
  }

  // moving up the display on the R
  for (int8_t i=ROW_SIZE-1; i>=0; i--)
  {
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      if (crossCheckBreak(oldMode)) return false;
     mx.setColumn(j, i, 0xff);
      mx.setRow(j, ROW_SIZE-1, 0xff);
    }
    mx.update();
    delay(DELAY_TIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
     if (crossCheckBreak(oldMode)) return false;
      mx.setColumn(j, i, 0x00);
      mx.setRow(j, ROW_SIZE-1, 0x00);
    }

      if (crossCheckBreak(oldMode)) return false;
  }

  // diagonally up the display L to R
  for (uint8_t i=0; i<ROW_SIZE; i++)
  {
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
      if (crossCheckBreak(oldMode)) return false;
      mx.setColumn(j, i, 0xff);
      mx.setRow(j, ROW_SIZE-1-i, 0xff);
    }
    mx.update();
    delay(DELAY_TIME);
    for (uint8_t j=0; j<MAX_DEVICES; j++)
    {
     if (crossCheckBreak(oldMode)) return false;
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
    delay(SCREEN_DELAY_TIME); // Delay between scrolls or updates


    if (!cross())
      return;
  
    delay(SCREEN_DELAY_TIME); // delay between scrolls or updates

    CentreText(getDay());
    if (!bullseye())
      return;
  
    CentreText(getDate());
    delay(SCREEN_DELAY_TIME);

    if (!bounce())
      return;
    if (!bounce2())
      return;
    if(!spectrum1())
      return;
    if (!spectrum2())
      return;

    CentreText(getTemperature());
    delay(SCREEN_DELAY_TIME);

    if (!randomSpot())
      return;
    if (!randomFill(true, true))
      return;
    if (!randomFill(false, false))
      return;

    CentreText(getHumidity());
    delay(SCREEN_DELAY_TIME);

    if (!randomDown())
      return;
    if (!sineWave())
      return;
    if (!rain())
      return;
}

void SaveSetting(const char * key, int value)
{ 
    preferences.begin("Settings", false);
    preferences.putInt(key, value); 
    preferences.end();
}

void SaveSetting(const char * key, String value)
{ 
    preferences.begin("Settings", false);
    preferences.putString(key, value); 
    preferences.end();
}

void GetAlarmString()
{ 
    preferences.begin("Settings", false);
    alarmString = preferences.getString("alarm", "06:00"); 
    Serial.println("Alarm been retrieved -" + alarmString);
    preferences.end();
}

void GetSettings()
{ 
    preferences.begin("Settings", false);
    timeZone = preferences.getString("timezone", "JST-9"); 
    alarmOn = preferences.getInt("alarm_on", 0);
    alarmMotionOn = preferences.getInt("al_screen_on", 0);
    DisplayMotionOn = preferences.getInt("m_screen_on", DISPLAY_MOTION_OFF);
    Serial.print("DisplayMotionOn");
    Serial.println(DisplayMotionOn);
    celsius = preferences.getInt("celsius", 1);
    hour24 = preferences.getInt("hour24", 1);
    intensity = preferences.getInt("intensity", 0);
    preferences.end();
}

void CentreTextPriority(const char * msg)
{
  SetPriorityDisplay(true);
  CentreText(msg, true);
  delay(SCREEN_DELAY_TIME);
  SetPriorityDisplay(false);
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
     case SHOW_ALL2:
          CentreTextPriority(DISPLAY_MSG_ALL2);
          break;
    case SCROLL_ALL:
          CentreText(DISPLAY_MSG_SCROLL);
          break;
    case DEMO_MODE:
          CentreTextPriority(DISPLAY_MSG_DEMO);
          break;
    case SHOW_LIGHT:
          CentreTextPriority(DISPLAY_MSG_LIGHT);
          break;
    }
}

void SetSaveandDisplayMode(int newMode, bool SaveMode = true)
{
  Serial.print("Mode changed to: ");
  Serial.print(newMode);

   currentMode = newMode;
   if (SaveMode)
     SaveSetting("display_mode", currentMode);
   
   DisplayCurrentMode();
}

bool ValidateAlarmStringAddition(String newChar)
{
  if (alarmString.length() == 0) 
  {
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
  SetPriorityDisplay(true); 
  CentreText(DISPLAY_MSG_OK, true);
  delay(SCREEN_DELAY_TIME); // delay between scrolls or updates 
  SetPriorityDisplay(false); 
}

void DeleteFromAlarmString()
{
  if(alarmString.length()>0)
    alarmString.remove(alarmString.length() - 1); // Removes the last character from the string
  else
  {
    Beep();
    return;
  }

  if(alarmString.length()==3 || alarmString.length()==6) //remove the : or ? as well..
    alarmString.remove(alarmString.length() - 1); // Removes the last character from the string
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
    decode_results results;

    if (irrecv.decode(&results)) {
         
        keyHit =  (isValueInArray(results.value)); //Is it a recognised IR code (from the Clocky remote control)
        if(alarmBeeping)
        {
            if (isValueInArray(results.value)) //Is it a recognised IR code (from the Clocky remote control)
            {
              Serial.print("Switching off alarm");  
              alarmBeeping=false;
              alarmOn=0;
              SaveSetting("alarm_on", alarmOn);
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
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_DOWN])
          {
            if (setupStep < SETUP_MAX) 
              setupStep++;
            else  
              Beep(); 

            setupSubstep=0;  
          }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_UP])
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
              if (setupSubstep==SETUP_STEP_ALARM_SUBSTEP_1_EDIT_TIME)
              {
                if(alarmString.length() == 6)
                {
                  alarmString.remove(alarmString.length() - 1); // Removes the last character '?' from the string
                  SaveSetting("alarm", alarmString);
                  alarmOn=1;  //Turn on the alarm
                  SaveSetting("alarm_on", alarmOn);
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
               
                SaveSetting("alarm_on", alarmOn);
                ShowOK();
              }
            else if (setupSubstep==3)
              {
                if (alarmMotionOn==1)
                  alarmMotionOn=0;
                else
                  alarmMotionOn=1;
               
                SaveSetting("al_screen_on", alarmMotionOn);
                ShowOK();
              }
           }
           else if (setupStep==SETUP_STEP_12_24 && setupSubstep==1)
           {
            if (hour24==1)
              hour24 = 0;
            else
              hour24 = 1;
            SaveSetting("hour24", hour24);
            ShowOK();
           }
           else if (setupStep==SETUP_STEP_WIFI && setupSubstep==1)
           {
              wifiManager.resetSettings();
              ShowOK();
              ESP.restart();
           }
           else if (setupStep==SETUP_STEP_C_F  && setupSubstep==1)
           {
             if (celsius==1)
               celsius = 0;
             else
               celsius = 1;
             SaveSetting("celsius", celsius);
             ShowOK();
           }
           else if (setupStep==SETUP_STEP_DISPLAY)
           {
                 if (setupSubstep==1) //Screen On with motion
                 {
                  switch (DisplayMotionOn) 
                  {
                    case DISPLAY_MOTION_OFF:
                      DisplayMotionOn = DISPLAY_MOTION_ON;
                      break;
                    case DISPLAY_MOTION_ON:
                      DisplayMotionOn = DISPLAY_MOTION_DARK;
                      break;
                    case DISPLAY_MOTION_DARK:
                      DisplayMotionOn = DISPLAY_MOTION_OFF;
                      break;
                  }
                   SaveSetting("m_screen_on", DisplayMotionOn);
                   ShowOK();
                 }
           }
           else if (setupStep==SETUP_STEP_ZONE)
              {
                 if (setupSubstep==1) //Tokyo
                 {
                   SetTimezone("JST-9"); 
                   SaveSetting("timezone", timeZone); 
                 }
                 else if (setupSubstep==2) //UK
                 {
                   SetTimezone("GMT0BST,M3.5.0/1,M10.5.0"); 
                   SaveSetting("timezone", timeZone); 
                 }
                 else if (setupSubstep==3) //HK
                 {
                   SetTimezone("HKT-8"); 
                   SaveSetting("timezone", timeZone); 
                 }
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

         if (setupStep==SETUP_STEP_ALARM && setupSubstep==SETUP_STEP_ALARM_SUBSTEP_1_EDIT_TIME)
         {
            if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_0])
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
            SetSaveandDisplayMode(SHOW_ALL2);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_7])
            SetSaveandDisplayMode(SCROLL_ALL);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_8])
            SetSaveandDisplayMode(SHOW_LIGHT);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_9])
            SetSaveandDisplayMode(DEMO_MODE);
          else if (results.value == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_STAR])
          {
            SetPriorityDisplay(true);
            currentMode = SETUP_MODE; 
            setupStep=1;
            setupSubstep=0;
            CentreText(DISPLAY_MSG_SETUP, true);
            delay(1000);
            SetPriorityDisplay(false); 
         }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_UP])
          {
                  if (intensity < 10) {
                      intensity++;
                      mx.control(MD_MAX72XX::INTENSITY, intensity);
                      SaveSetting("intensity", intensity);
                  } 
                  else 
                    Beep();
                    
                  delay(SCREEN_DELAY_TIME); // delay between scrolls or updates
                  SetPriorityDisplay(false);
           }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_DOWN])
            {
                  if (intensity > 0) {
                      intensity--;
                      mx.control(MD_MAX72XX::INTENSITY, intensity);
                      SaveSetting("intensity", intensity);
                  } 
                  else 
                    Beep();
              }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_LEFT])
            {
              if (currentMode > 0)
                SetSaveandDisplayMode(currentMode-1);
              else
                SetSaveandDisplayMode(9);
            }
          else if (results.value  == REMOTE_BUTTONS[current_remote][BUTTON_PRESS_RIGHT])
            {
              if (currentMode < 9) 
                SetSaveandDisplayMode(currentMode+1);
              else
                SetSaveandDisplayMode(0);
            }
     }
     irrecv.resume(); // Receive the next value
    }
}

// Task function for Task1
void DisplayTask(void *pvParameters) {
 
  ScrollText("Welcome to Clocky!", true);
  delay(SCREEN_DELAY_TIME);
  columns2(false);
  
  for (;;) {
 
    if (WiFi.status() == WL_CONNECTED) //Should probably put some kind of reconnect in here? TODO
    {
      if (!getLocalTime(&timeinfo)) 
        return;
    }    

    if (currentMode == SHOW_NOTHING)
         mx.clear();
    
    if (screenSaver && currentMode != SETUP_MODE)
    {
      if (alarmOn==1)
        getTime(); //Still need to check the time for the alarm
      mx.clear();
      delay(SCREEN_DELAY_TIME); // delay between scrolls or updates
      continue;
    } 
    
    if (currentMode == SHOW_TIME || currentMode == SHOW_DATE || currentMode == SHOW_ALL || currentMode == SHOW_ALL2 || currentMode == SCROLL_ALL)
    {
        if (currentMode == SHOW_TIME || currentMode == SHOW_ALL || currentMode == SHOW_ALL2)
        {
          getTime();
          CentreText(displayTime);
          if (currentMode == SHOW_TIME)
          {
           dot(true);
           AlarmDot(alarmOn==1);
          }
          delay(SCREEN_DELAY_TIME); // delay between scrolls or updates
        }
        if (currentMode == SHOW_DATE || currentMode == SHOW_ALL || currentMode == SHOW_ALL2){
          if (currentMode == SHOW_ALL2)
             ScrollText(getDay(true).c_str());
          else   
             CentreText(getDay(false));
          delay(SCREEN_DELAY_TIME); // delay between scrolls or updates
        }
          
        if (currentMode == SHOW_DATE || currentMode == SHOW_ALL || currentMode == SHOW_ALL2) //check the mode again, in case it has changed
        {
          if (currentMode == SHOW_ALL2)
            RightUp(getDate().c_str());
          else
            CentreText(getDate());
          delay(SCREEN_DELAY_TIME); // delay between scrolls or updates
        }
    }  

    if (currentMode == SHOW_TEMPERATURE || currentMode == SHOW_ALL || currentMode == SHOW_ALL2)
    {
      CentreText(getTemperature());
      delay(SCREEN_DELAY_TIME); // delay between scrolls or updates
    }

    if (currentMode == SHOW_HUMIDITY || currentMode == SHOW_ALL || currentMode == SHOW_ALL2)
    {
       CentreText(getHumidity());
       delay(SCREEN_DELAY_TIME); // delay between scrolls or updates
    }
    else if (currentMode == SHOW_LIGHT)
    {
      CentreText(String(lightLevel));
      delay(SCREEN_DELAY_TIME); // delay between scrolls or updates
   }
   else if (currentMode == DEMO_MODE)
    {
       Demo();
    }
    else if (currentMode == SETUP_MODE)
    {
      if (setupStep==SETUP_STEP_ALARM)
      {
        if (setupSubstep==SETUP_STEP_ALARM_SUBSTEP_0_DISPLAY_ALARM)
          CentreText(DISPLAY_MSG_ALARM);
        else if (setupSubstep==SETUP_STEP_ALARM_SUBSTEP_1_EDIT_TIME)
          CentreText(alarmString);
        else if (setupSubstep==SETUP_STEP_ALARM_SUBSTEP_2_ON_OFF)
        {
          if(alarmOn)
            CentreText(DISPLAY_MSG_ALARM_OFF);      
          else 
            CentreText(DISPLAY_MSG_ALARM_ON);      

        } 
        else if (setupSubstep==SETUP_STEP_ALARM_SUBSTEP_3_MOTION_OFF)
        {
          if(alarmMotionOn)
            CentreText(DISPLAY_MSG_ALARM_MOTION_OFF);      
          else 
            CentreText(DISPLAY_MSG_ALARM_SCREEN_ON);      
        } 
      }
      else if (setupStep==SETUP_STEP_12_24)
      {
        if (setupSubstep==0)
          CentreText(DISPLAY_MSG_12_24_HR);
        else if (setupSubstep==1)
        {
          if (hour24==1)
            CentreText(DISPLAY_MSG_12HR);
          else
            CentreText(DISPLAY_MSG_24HR);
        }
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
       {
        char cBuffer[5];
        sprintf(cBuffer, "%cC/%cF", DEGREE_CHARACTER, DEGREE_CHARACTER);
        CentreText(cBuffer);
       }
       else if (setupSubstep==1)
       {
         if (celsius==1)
         {
           char cBuffer[5];
           sprintf(cBuffer, "%cF?", DEGREE_CHARACTER);
           CentreText(cBuffer);
         }
         else
         {
           char cBuffer[5];
           sprintf(cBuffer, "%cC?", DEGREE_CHARACTER);
           CentreText(cBuffer);
         }
       }
      }
      else if (setupStep==SETUP_STEP_DISPLAY)
      {
        if (setupSubstep==0)
          CentreText(DISPLAY_MSG_PIR);
        else if (setupSubstep==1)
        {
          switch (DisplayMotionOn) 
          {
            case DISPLAY_MOTION_OFF:
              CentreText(DISPLAY_MSG_DISPLAY_MOTION_ON);
              break;
            case DISPLAY_MOTION_ON:
              CentreText(DISPLAY_MSG_DISPLAY_MOTION_DARK);
              break;
            case DISPLAY_MOTION_DARK:
              CentreText(DISPLAY_MSG_DISPLAY_MOTION_OFF);
              break;
          }
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
          CentreText(DISPLAY_MSG_HK);
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
 
        sprintf(scrollBuffer, "%s %s %s %s %s", displayTime, getDay(), getDate(true).c_str(), getTemperature().c_str(), getHumidity().c_str()); 
        mx.clear();
        ScrollText(scrollBuffer);
        delay(100); // delay between scrolls or updates
    } 
   
    vTaskDelay(pdMS_TO_TICKS(100)); // delay for a bit

  } //For loop forever
}

// Task function for Task2 - Trigger task - triggered by IR remote, PIR
void TriggerTask(void *pvParameters) {
  irrecv.enableIRIn();
  for (;;) {
    checkIR();
    vTaskDelay(pdMS_TO_TICKS(100)); // delay for a bit

    alarmBeeping = (alarmOn==1 && strcmp(displayTime, alarmString.c_str()) == 0); 
    
    if (alarmBeeping)
    {
      Serial.println("Alarm beeping");
      Beep();
    }
  
    if (motionDetected) 
    {
      motionDetected = false;
      lastMovementTime = millis();
 
       // Handle motion detection event here
      if (alarmMotionOn==1 && alarmBeeping)
      {
        alarmOn=0;
        SaveSetting("alarm_on", alarmOn);
        alarmBeeping=false;
      }
    }
    //Check light level and adjust display if required
    getLight();

    // Check if screenSaver should be activated
    bool isMotionOn = (DisplayMotionOn == DISPLAY_MOTION_ON);
    bool isDarkAndMotionDark = (DisplayMotionOn == DISPLAY_MOTION_DARK && IsDark());
    bool isScreenSaverTimeout = ((millis() - lastMovementTime) > screenSaverTimeout);
    screenSaver = (isMotionOn || isDarkAndMotionDark) && isScreenSaverTimeout;
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

void SetTimezone(String timezone){
  timeZone = timezone;
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
 
  GetSettings();
 
  GetAlarmString();
  GetDisplayMode();
 
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT_PULLDOWN);
 
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), detectsMovement, RISING);

  Wire.begin(SDA_PIN, SCL_PIN);
 
  lightMeter.begin();

  sht31 = Adafruit_SHT31();
  Serial.println("Initializing SHT31 sensor...");
  if (!sht31.begin(0x44)) 
  {
    Serial.println("Couldn't find SHT31 sensor, check wiring!");
    while (1);
  }
  
  if (!mx.begin())
    Serial.println("\nMD_MAX72XX initialization failed");

   mx.control(MD_MAX72XX::INTENSITY, intensity);

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
