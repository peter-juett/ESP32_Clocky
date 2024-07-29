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

const int current_remote = 1; //TODO - make this configurable
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
const int LIGHT_LEVEL_LOW = 15;

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

const unsigned long debounceDelay = 6000; // Adjust debounce delay as needed
const unsigned long lightCheckPeriod = 1000;
const unsigned long screenSaverTimeout = 60000; // 1 minute

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";