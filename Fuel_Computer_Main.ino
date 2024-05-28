
// Sketch "Fuel Computer" Author : Jim Anderson Stuart Florida

/* Sketch takes conditioned negative - going pulse from fuel injector
   to determine engine speed and fuel consumption rate.
   RPM = 120/(injection pulse period)
   Fuel Consumption per pulse = 285 mCC / (microsecond injection duration)
   Current fuel tank level is determined as proportional to voltage at sender pin.
   Battery voltage is also proportionately calculated. Information is stored in 
   1 megabyte flash memory.
   A GPS module provides position, course and speed and that information is
   used to calculate miles  per gallon and endurance. Trip function and various
   history parameters (total hours, fuel, etc.) are provided. All information is
   presented on a 3.5" TFT touch screen lcd display.
   Sketch runs on Arduino Mega or higher board.
*/

//Latest working version
String  Version = "Ver. 5/18/24";

// 8/9/23 Added gps time to nav display
// 10/8/23 Fix DST calculations
// 10/21/23 moved declaration of gps to try to fix freezes
// 11/11/23 Let's try the gps object on the heap. Just this for now.
//    don't forget that we've added a constructor in Setup()
//  11/24/23 Implement GPS stale timer and destructor / reconstructor
//  11/26/23 MAXSTALETIME from 30 to 10
//  11/27/23 ... and back to 30 sec.
// 11/28/23 commented out RESETFLAG in screenoff(). Don't restart program
//   on on off button press.
//  11/30/23 Removed readVCcc(). Was never used.
//  12/1/23 Move gps object pointer to the bottom of the stack
//  12/2/23 Added code to implement program restart by holding
//    History button.
// 12/4/23 Modified Button5() logic to not change screen background repeatedly
//  while button is down.
// 2/5/24  Moved GPS encode to top of Loop.
// 2/23/24 Repllaced "engine.Last." course and speed variables in engine and nav pages with "gps->"
//  to see if that fixes periodic "blinks" of the gps data.
//
//  3/5/24 Fuel useage seems about 8% low, so let's bump down the #define MICROSECPERMILLICC 285
//         to 262
//  3/19/24 Modified condition to calculate endurance in UpdatePage2() (Navigation). Removed
//          Dependance on good course.
//  4/27/24 Moved screenStatus into engine structure so it will be saved to memory.
//          Also changed stale handling function from restarting gps object to program reset.
//  5/4/24  Added engine housekeeping variable to restor last display page on restart from stale GPS data.
//  5/7/24 Fixed issue of time going negative
//  5/11/24 Fixed issue of wrong page drawn on stale restart.
//  5/18/24 changed speed clamp in UpdatePage2() to print float equal or below 9.9. Was
//   less than 10.0


//*****************************************************************************
//
//                           Serial Monitor Command List
//
//*****************************************************************************
//   All followed by ";"
//
//  memorydiagnostics_on
//  memorydiagnostics_off
//  realtimeengine_on
//  realtimeengine_off
//  pulsediagnostics_on
//  pulsediagnostics_off
//  speedarraydiagnostics_on
//  speedarraydiagnostics_off
//  realtimeenginefuel_on
//  realtimeenginefuel_off
//  gpsdiagnostics_on
//  gpsdiagnostics_off
//  listparameters
//
//*******************************************************************************

//*******************************************************************************
//
//                 Method List
//
//******************************************************************************
//void  SetupButtons()
//void  DrawButtons()
//void  DrawTripButtons()
//void  DrawMaintButton()
//void  DrawPage1()
//void  DrawPage2()
//void  DrawPage3()
//void  DrawPage4()
//void  DrawPage5()
//void  UpdatePage1()
//void  UpdatePage2()
//void  UpdatePage3()
//void  UpdatePage4()
//void  UpdatePage5()
//void  Button1()
//void  Button2()
//void  Button3()
//void  Button4()
//void  Button5()
//void  TripAStartButton()
//void  TripAStopButton()
//void  TripBStartButton()
//void  TripBStopButton()
//void  MaintResetButton()
//uint8_t readFT6236TouchRegister( uint8_t reg )
//uint8_t readFT6236TouchAddr( uint8_t regAddr, uint8_t * pBuf, uint8_t len )
//uint8_t readFT6236TouchLocation( TouchLocation * pLoc, uint8_t num )
//void writeFT6236TouchRegister( uint8_t reg, uint8_t val)
//uint32_t dist(const TouchLocation & loc)
//bool sameLoc( const TouchLocation & loc, const TouchLocation & loc2 )
//bool  IsInRect(TouchLocation point, Rect rect)
//void  ScreenOff()
//void  ScreenOn()
//float GetBattVoltage()
//void GetSpedandFuel()
//void  GetFuelLevel()
//unsigned long SmoothFuel()
//unsigned long SmoothSpeeds()
//void UpdateSpeedRanges(int speed)
//void  UpdateSpeedArray()
//void  CalcGPH()
//void  GPSUpdate()
//void  UpdateTrips()
//bool  CheckRunning()
//void serialEvent()
//void  ProcessCommand(String command)
//void  OnSecond()
//void  OnMinute()
//void  ListParameters()
//void  ResetMemoryData()
//void  ZeroData()
//bool  WriteMemory(uint32_t addr)
//bool  ReadMemory(uint32_t addr)
//void  UpdateFlash()
//void  MemoryParameterCheck()
//void  StartPulse()
//void  EndPulse()
//ISR(TIMER1_COMPA_vect)
//void(* resetFunc)()
//void setup()
//void loop()
//******************************************************************************


//*********************************************************
// Includes
//*********************************************************

// TFT display
#include <stdint.h>
#include <UTFT.h>
#include <Wire.h>
#include <avr/pgmspace.h>


// For flash memory
#include<SPIMemory.h>

// GPS Module
#include <TinyGPS++.h>

// For Arduino timer interrupts
#include <avr/io.h>
#include <avr/interrupt.h>

//************************************************************
// Defines
//************************************************************

// Hardware:


// Display touch controller interrupt digital pin
#define FT6236_INT   5

// Display Graphics

// Colors
#define BACKGND           VGA_WHITE
#define TEXT              VGA_BLACK

bool   WHITESCREEN;

// TFT layout def's
#define SCREEN            0, 0, 479, 319
#define PAGERECT          0, 0, 479, 260
#define BUTTONRECT        0, 261, 479, 319

#define ENGINESTOPDELAY 1.0e6 // microseconds
#define SPEEDARRAYSIZE  10 // Size of rpm smoothing array

#define STOP  0
#define RUN   1
#define RESET 2
#define DOWN  0
#define UP    1
#define OFF   0
#define ON    1

// Flag for corrupted memory
#define ERASEDDATA (engine.Total.hours >= maxMicros)

// GPS stale timer
#define MAXSTALETIME  30   //seconds

// Button press time to reset trips and maintenance timer
// and number of loops to recognize release
#define RESET_TIME          3000 // milliseconds
#define MAINT_RESET_TIME    7000 // milliseconds
#define BUTTON_UP_TIME      1000 // milliseconds

// Maximum unsigned long. This is when micros rolls over//
#define maxMicros 4294967295

// Set adresses in flash memory for our permanent parameters
#define ENGINEMEMORYADDRESS    0x0000
#define BACKUPMEMORYADDRESS    0x1000

// Input voltage divider ratios
// Based on nominal resistance values
//#define BATTVOLTAGEDIV  0.3102
//#define FUELVOLTAGEDIV  0.757

// Corrected for Beta prototype 5/7/23
#define BATTVOLTAGEDIV  0.3120
#define FUELVOLTAGEDIV  0.7617

#define TANKCAPACITY  33              // gallons
#define MAINTENANCEINTERVAL 6000     // 100 hours

#define GALLONSPERCC  .000264172
/*#define MICROSECPERMILLICC 285       //From Suzuki data and tests*/

#define MICROSECPERMILLICC 262 // 3/5/24
//*****************************************************************
// Globals
//*****************************************************************


// Diagnostic flags
bool MEMORYDIAGNOSTICS;
bool REALTIMEENGINE;
bool PULSEDIAGNOSTICS;
bool SPEEDARRAYDIAGNOSTICS;
bool REALTIMEENGINEFUEL;
bool GPSDIAGNOSTICS;

// TFT display and touch control
// Remember to change the model parameter to suit your display module!
UTFT tft(ILI9488_16, 7, 38, 9, 10); //(byte model, int RS, int WR, int CS, int RST, int SER)
uint16_t tx, ty;


// For touch controller internal use
enum {
  eNORMAL = 0,
  eTEST   = 0x04,
  eSYSTEM = 0x01
};

struct TouchLocation
{
  uint16_t x;
  uint16_t y;
};

TouchLocation touchLocations[2];

// Display Pages
enum  {
  Instruments,
  Navigation,
  TripA,
  TripB,
  History
};


uint8_t buf[30];
uint8_t addr  = 0x38;  //Touch controller CTP IIC ADDRESS


// FONTS Declare which fonts we will be using
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];
extern uint8_t GroteskBold16x32[];
extern uint8_t GroteskBold32x64[];
extern uint8_t arial_bold[];

// Button icons
extern const uint16_t       InstrumentsIcon[];
extern const unsigned short NavigationIcon[];
extern const unsigned short TripIcon1[];
extern const unsigned short TripIcon2[];
extern const unsigned short HistoryIcon[];



// Buttons
struct  Rect
{

  uint16_t left;
  uint16_t top;
  uint16_t right;
  uint16_t bottom;
};

Rect  button1Rect, button2Rect, button3Rect, button4Rect, button5Rect;
Rect  startButtonRect, stopButtonRect, maintButtonRect;
uint8_t currentDisplayPage ;

uint8_t         tripAButtonState, tripBButtonState, maintButtonState, histButtonState;
unsigned long   tripAResetTimer, tripBResetTimer, maintResetTimer, progResetTimer;
unsigned long   resetAStart, resetBStart, resetMaintStart, resetProgStart;
bool            tripAButtonLatch, tripBButtonLatch, maintButtonLatch, histButtonLatch;
unsigned long   buttonUpStart, buttonUpTimer;

//bool         screenStatus;


/*
   These global variables enable assembly of the user serial
   input command.
*/
bool g_command_ready(false);
String g_command;

// Flash memory
SPIFlash flash;


// Reading flags
bool  COURSESTALE;
bool  SPEEDSTALE;
bool  LATLONSTALE;
bool  STALEFLAG;


// Timers
unsigned int  secondReload = 0xF424; //Timer1 period 16 us, so 1 sec = 62,500
unsigned int  staleTimer;  // for stale gps data

// Hardware pin assignments
// Digital Input must use one pin per ISR
int injPinA = 18;     // start of pulse
int injPinB = 19;     // end of pulse
int fuelPin  = A8;
int battPin = A9;
int ledPin = 13;
int screenPinOff = 2;
int screenPinOn = 3;





// Memory parameters
// Total parameters are permanent and accumulating
struct  Engine {
  struct TOTAL  {
    unsigned long   hours;
    unsigned long   minutes;
    unsigned long   starts;
    unsigned long   speed_time_array[7]; //0-1k,1-2k,2-3k,3-4k,4-5k,5-6k,6+k
    float           fuelUsed;
  } Total;

  struct LAST {
    int             rpm;
    float           battVoltage;
    double          lat;
    double          lon;
    float           knots;
    float           range;  // nautical miles
    float           course;  // degrees
    float           fuelRemain;  // gallons
    float           fuelAmount;
    float           gph;
    float           nmpg;
    float           tripAFuel;
    float           tripBFuel;
    float           tripAMiles; // nautical miles
    float           tripBMiles;
    int             tripAState;
    int             tripBState;
    unsigned long   tripATime;  // minuutes
    unsigned long   tripBTime;
    double          tripALat;
    double          tripALon;
    double          tripBLat;
    double          tripBLon;
    unsigned long   progTime;
    unsigned long   maintenanceTimer; // minutes
    unsigned long   pulseWidth;
  } Last;

  // Housekeeping variables stored
  uint8_t      lastPage;
  bool         screenStatus;
  bool         RESTART;

};
Engine engine;


// Engine parameters
int           rpm;
unsigned long speeds[SPEEDARRAYSIZE];
float         fuelLevel;
float         fuelTankLevel[10];
float         fuelQty, lastFuelQty, cycleFuel;
float         battVoltage;
int           speed_time_accum[7];


// Input Signal Range
int vMax, vMin;

// Various timers
unsigned long trip_A, trip_B, programTime;
unsigned long motor_timer;

// Timer flags
bool  secondFlag, minuteFlag;

// Counter for accumulated engine pulses
int   intervalPulseCount;

// Missed interrupt flag
bool missedISR;



// Volitile globals used by ISRs
volatile unsigned long  programSeconds, programMinutes;
volatile bool           pulseReceived, pulseStart, running;


// times of fuel injection edges since program start
volatile unsigned long start, finish, pulsePeriod, lastPulse, thisPulse;


// Restart Program
bool  RESETFLAG;

// GPS Object;
// Let's put the gps on the heap so we can destroy it
// and reinitiate it if the values become stale for
// too long. We'll change the instance to a pointer here and
// change the direction symbol "." to "->" everywhere in the code
//TinyGPSPlus *gps = new TinyGPSPlus();

//TinyGPSPlus gps; // Let's put it on the heap

TinyGPSPlus* gps;


//****************************************************************
//
//
//                        Methods
//
//****************************************************************



//***************************************************************
//
//    TFT touch display routines
//
//**************************************************************

void  SetupButtons()
{
  button1Rect.left = 20;
  button1Rect.top = 263;
  button1Rect.right = 85;
  button1Rect.bottom = 303;

  button2Rect.left = 110;
  button2Rect.top = 263;
  button2Rect.right = 175;
  button2Rect.bottom = 303;

  button3Rect.left = 200;
  button3Rect.top = 263;
  button3Rect.right = 265;
  button3Rect.bottom = 303;

  button4Rect.left = 290;
  button4Rect.top = 263;
  button4Rect.right = 355;
  button4Rect.bottom = 303;

  button5Rect.left = 380;
  button5Rect.top = 263;
  button5Rect.right = 445;
  button5Rect.bottom = 303;

  startButtonRect.left = 116;
  startButtonRect.top = 195;
  startButtonRect.right = 181;
  startButtonRect.bottom = 230;

  stopButtonRect.left = 213;
  stopButtonRect.top = 195;
  stopButtonRect.right = 278;
  stopButtonRect.bottom = 230;


  maintButtonRect.left = 345;
  maintButtonRect.top = 115;
  maintButtonRect.right = 410;
  maintButtonRect.bottom = 155;

}


void  DrawButtons()
{

  //tft.setColor(BACKGND);

if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);

  
  tft.fillRect(BUTTONRECT);

  tft.drawBitmap(button1Rect.left, button1Rect.top, 65, 38, InstrumentsIcon);
  tft.drawBitmap(button2Rect.left, button2Rect.top, 65, 40, NavigationIcon);
  tft.drawBitmap(button3Rect.left, button3Rect.top, 65, 40, TripIcon1);
  tft.drawBitmap(button4Rect.left, button4Rect.top, 65, 40, TripIcon2);
  tft.drawBitmap(button5Rect.left, button5Rect.top, 65, 40, HistoryIcon);


}

void  DrawTripButtons()
{

  // If trip is stopped, button label should turn to reset
  // and start button should be active
  if (((currentDisplayPage  == TripA) && (engine.Last.tripAState == STOP)) ||
      ((currentDisplayPage  == TripB) && (engine.Last.tripBState == STOP)))
  {
    
    
    tft.setColor(0, 255, 0); // green
    tft.fillRect(startButtonRect.left, startButtonRect.top, startButtonRect.right, startButtonRect.bottom);
    tft.setColor(VGA_BLACK);
    tft.fillRect(stopButtonRect.left, stopButtonRect.top, stopButtonRect.right, stopButtonRect.bottom);
    tft.setColor(TEXT);
    tft.setBackColor(VGA_WHITE);
    tft.setFont(SmallFont);
    tft.print("START", startButtonRect.left + 12, startButtonRect.top + 11);
    tft.print("RESET", stopButtonRect.left + 12, stopButtonRect.top + 11);
  }

  else // Start button should be inactive stop should be active
  {
    tft.setColor(VGA_RED);
    tft.fillRect(stopButtonRect.left, stopButtonRect.top, stopButtonRect.right, stopButtonRect.bottom);
    tft.setColor(VGA_GRAY); // green
    tft.fillRect(startButtonRect.left, startButtonRect.top, startButtonRect.right, startButtonRect.bottom);
    
    tft.setColor(TEXT);
    tft.setBackColor(VGA_WHITE);
    tft.setFont(SmallFont);
    tft.print("START", startButtonRect.left + 12, startButtonRect.top + 11);
    tft.print("STOP", stopButtonRect.left + 15, stopButtonRect.top + 11);
  }

}


void  DrawMaintButton()
{
  tft.setColor(VGA_RED);
  tft.setBackColor(VGA_WHITE);
  tft.fillRect(maintButtonRect.left,maintButtonRect.top,maintButtonRect.right,maintButtonRect.bottom);

  tft.setColor(TEXT);
  tft.setFont(SmallFont);
  tft.print("RESET", maintButtonRect.left+10,maintButtonRect.top+15);
}



// Instrument Page Graphics - Page 1
#define LATLONRECT        20, 15, 457, 55
#define LATLABELLOC       22, 25
#define LATLOC            90, 20
#define LATCLEAR          85, 22, 230, 53
#define LONLABELLOC       230, 25
#define LONLOC            294, 20
#define LONCLEAR          294, 23, 450, 53
#define RPMRECT           20, 65, 190, 198
#define RPMLABELLOC       79, 77
#define RPMLOC            31, 105
#define RPMCLEAR          31, 98,159, 160
#define GPMRECT           20, 210, 190, 250
#define GPHLABELLOC       22, 220
#define GPHLOC            82, 214
#define GPMCLEAR          85, 216, 184, 245
#define FUELRECT          196, 65, 333, 198
#define FUELLABELLOC      225, 77
#define FUELLOC           210, 105
#define FUELCLEAR         210, 110, 326, 189
#define GALLABELLOC       230, 175
#define SPEEDRECT         341, 65, 475, 198
#define SPEEDLABELLOC     360, 77
#define SPEEDLOC          343, 105
#define SPEEDCLEAR        343, 105, 472, 189
#define KTSLABELLOC       375, 175

void  DrawPage1()  // Instruments
{


  if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);
  tft.fillRect(PAGERECT);

  // Outline data regions
  tft.setColor(VGA_BLACK);
  tft.drawRect(LATLONRECT);
  tft.drawRect(RPMRECT);
  tft.drawRect(GPMRECT);
  tft.drawRect(FUELRECT);
  tft.drawRect(SPEEDRECT);


  // Labels
  tft.setColor(TEXT);
  //tft.setBackColor(BACKGND);

if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);
  
  tft.setFont(BigFont);
  tft.print("Lat: ", LATLABELLOC );
  tft.print("Lon: ", LONLABELLOC );
  tft.print("RPM", RPMLABELLOC);
  tft.print("GPH: ", GPHLABELLOC );
  tft.print("FUEL", FUELLABELLOC );
  tft.print("Gal.", GALLABELLOC );
  tft.print("SPEED", SPEEDLABELLOC );
  tft.print("Kts", KTSLABELLOC );



}

// Navigation Page Graphics - Page 2

#define COURSERECT        20, 70, 157, 213
#define COURSECLEAR       30, 98, 150, 180
#define COURSELABELLOC    38, 75
#define COURSELOC         36, 108
#define DEGTRUELABELLOC   28, 185
#define SPEEDRECT2        165, 70, 281, 213
#define SPEEDLABELLOC2    178, 75
#define SPEEDLOC2         170, 108
#define SPEEDCLEAR2       172, 103, 265, 190
#define KTSLABELLOC2      195, 185
#define NMPGRECT           296, 86, 461, 126
#define NMPGLABELLOC       335, 65
#define NMPG_LOC           299, 96
#define NMPGCLEAR          350, 87, 460,125
#define NMPGLOC            375, 90
#define EDCTIMERECT       296, 156, 461, 196
#define EDCTIMECLEAR      345, 157, 460, 192
#define EDCHMLABELLOC     297, 167
#define EDCTIMELOC        355, 161
#define EDCTIMELOCX       355
#define EDCTIMELOCY       163
#define EDCDISTRECT       296, 201, 461, 241
#define EDCDISTCLEAR      345, 205, 458, 238
#define EDCNMLABELLOC     297, 210
#define EDCDISTLOC        353, 202
#define EDCLABELLOC       310, 132



void  DrawPage2() // Navigation
{

  //tft.setColor(BACKGND);
  if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);
  tft.fillRect(PAGERECT);

  // Outline data regions
  tft.setColor(VGA_BLACK);

  tft.drawRect(LATLONRECT);
  tft.drawRect(COURSERECT);
  tft.drawRect(SPEEDRECT2);
  tft.drawRect(EDCTIMERECT);
  tft.drawRect(EDCDISTRECT);
  tft.drawRect(NMPGRECT);

  // Labels
  tft.setColor(TEXT);
  //tft.setBackColor(BACKGND);


if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);

  
  tft.setFont(BigFont);

  tft.print("Lat: ", LATLABELLOC );
  tft.print("Lon: ", LONLABELLOC );
  tft.print("COURSE", COURSELABELLOC);
  tft.print("Deg True", DEGTRUELABELLOC);
  tft.print("SPEED", SPEEDLABELLOC2);
  tft.print("Kts", KTSLABELLOC2);
  tft.print("NMPG", NMPGLABELLOC);
  tft.print("NMPG: ", NMPG_LOC);
  tft.print("ENDURANCE", EDCLABELLOC);
  tft.print("H:M: ", EDCHMLABELLOC);
  tft.print("NM: ", EDCNMLABELLOC);


}

// Trip Page Graphics - Page 3 & 4

#define TRIPDISTRECT      23, 68, 139, 173
#define TRIPDISTLABELLOC  49, 70
#define TRIPNMLABELLOC    60, 147
#define TRIPDISTCLEAR     24, 70, 138, 136
#define TRIPDISTLOC       28, 95
#define TRIPLABELRECT     24, 176, 98, 259
#define TRIPLABELLOC      28, 178
#define TRIPIDLOC         45, 195
#define TRIPTIMERECT      145, 68, 289, 173
#define TRIPTIMELABELLOC  175, 70
#define TRIPTIMEHMLOC     179, 147
#define TRIPTIMECLEAR     147, 70, 285, 136
#define TRIPTIMELOCH      160, 95
#define TRIPTIMELOCHX     147
#define TRIPTIMELOCHY     95
#define TRIPTIMELOCM      225, 95
#define TRIPFUELRECT      296, 86, 461, 126
#define TRIPFUELLABELLOC  310, 62
#define TRIPGALLOC        299, 96
#define TRIPFUELCLEAR     350, 87, 460,125
#define TRIPFUELLOC       352, 90
#define EDCTIMERECT2       296, 156, 461, 196
#define EDCTIMECLEAR2      345, 157, 460, 192
#define EDCHMLABELLOC2     297, 167
#define EDCTIMELOC2        355, 161
#define EDCTIMELOCX2       355
#define EDCTIMELOCY2       163
#define EDCDISTRECT2       296, 201, 461, 241
#define EDCDISTCLEAR2      345, 205, 458, 238
#define EDCNMLABELLOC2     297, 210
#define EDCDISTLOC2        353, 202
#define EDCLABELLOC2       310, 129


void  DrawPage3()  // Trips A & B
{

  //tft.setColor(BACKGND);
  if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);
  tft.fillRect(PAGERECT);

  // Outline data regions
  tft.setColor(VGA_BLACK);
  tft.drawRect(LATLONRECT);
  tft.drawRect(TRIPDISTRECT);
  tft.drawRect(TRIPLABELRECT);
  tft.drawRect(TRIPTIMERECT);
  tft.drawRect(TRIPFUELRECT);
  tft.drawRect(EDCTIMERECT2);
  tft.drawRect(EDCDISTRECT2);


  // Labels
  tft.setColor(TEXT);
  //tft.setBackColor(BACKGND);

  if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);
  tft.setFont(BigFont);

  tft.print("Lat: ", LATLABELLOC );
  tft.print("Lon: ", LONLABELLOC );
  tft.print("DIST", TRIPDISTLABELLOC);
  tft.print("NM", TRIPNMLABELLOC);
  tft.print("TRIP", TRIPLABELLOC);
  tft.print("TIME", TRIPTIMELABELLOC);
  tft.print("H:M", TRIPTIMEHMLOC);
  tft.print("FUEL USED", TRIPFUELLABELLOC);
  tft.print("Gal.: ", TRIPGALLOC);

  tft.print("ENDURANCE", EDCLABELLOC2);
  tft.print("H:M ", EDCHMLABELLOC2);
  tft.print("NM: ", EDCNMLABELLOC2);

  switch(currentDisplayPage)
  {
    case TripA:
      if(engine.Last.tripAState == RUN)
      {
        tft.setColor(VGA_LIME);
        tft.setBackColor(VGA_LIME);
      }
      else 
      {
        tft.setColor(VGA_RED);
        tft.setBackColor(VGA_RED);
      }
      tft.fillRect(TRIPLABELRECT);
    break;

    case TripB:
      if(engine.Last.tripBState == RUN)
      {
        tft.setColor(VGA_LIME);
        tft.setBackColor(VGA_LIME);
      }
      else 
      {
        tft.setColor(VGA_RED);
        tft.setBackColor(VGA_RED);
      }
      tft.fillRect(TRIPLABELRECT);
    break;
  
  } // end switch

  DrawTripButtons();

}

void  DrawPage4()
{

  // Same as page 3 - Trip
  DrawPage3();
  DrawTripButtons();


}

// History Page Graphics
#define HISTORYLABELLOC       203, 2
#define ENGINETOTALLABELLOC   50, 15
#define TOTALHRSRECT          87, 40, 189, 61
#define TOTALHRSLABELLOC      0,  43
#define TOTALHRSCLEAR         90, 41, 187, 59
#define TOTALHRSLOC           91, 42
#define TOTALMINRECT          117, 68, 219, 92
#define TOTALMINLABELLOC      1, 73
#define TOTALMINCLEAR         119, 70, 217, 90
#define TOTALMINLOC           120, 72
#define TOTALSTARTSRECT       99, 104, 201, 128
#define TOTALSTARTSLABELLOC   2, 110
#define TOTALSTARTSCLEAR      101, 106, 199, 126
#define TOTALSTARTSLOC        102, 107
#define TOTALFUELRECT         196, 142, 298, 166
#define TOTALFUELLABELLOC     3,  146
#define TOTALFUELCLEAR        198, 144, 296, 164
#define TOTALFUELLOC          200, 146
#define MAINTTIMERECT         327, 76, 429, 100
#define MAINTTIMECLEAR        329, 78, 427, 98
#define MAINTTIMELOC          330, 79
#define TIMESINCELASTLABELLOC 320, 45
#define MAINTENANCELABELLOC   335, 59
#define MINATRMPLABELLOC      145, 180
#define RPMMINLABELLOC        15, RPMMINRECTTOP - 15
#define RPMMINRECTTOP         217
#define RPMMINRECTBOT         257
#define RPMMINRECT1           4, RPMMINRECTTOP, 64, RPMMINRECTBOT
#define RPMMINCLEAR1          5, RPMMINRECTTOP+2, 62, RPMMINRECTBOT-2
#define RPMMINLOC1            5, RPMMINRECTTOP+15
#define RPMMINRECT2           73, RPMMINRECTTOP, 133, RPMMINRECTBOT
#define RPMMINCLEAR2          74, RPMMINRECTTOP+2, 130, RPMMINRECTBOT-2
#define RPMMINLOC2            74, RPMMINRECTTOP+15
#define RPMMINRECT3           141, RPMMINRECTTOP, 201, RPMMINRECTBOT
#define RPMMINCLEAR3          142, RPMMINRECTTOP+2, 199, RPMMINRECTBOT-2
#define RPMMINLOC3            142, RPMMINRECTTOP+15
#define RPMMINRECT4           209, RPMMINRECTTOP, 269, RPMMINRECTBOT
#define RPMMINCLEAR4          210, RPMMINRECTTOP+2, 267, RPMMINRECTBOT-2
#define RPMMINLOC4            210, RPMMINRECTTOP+15
#define RPMMINRECT5           277, RPMMINRECTTOP, 337, RPMMINRECTBOT
#define RPMMINCLEAR5          278, RPMMINRECTTOP+2, 335, RPMMINRECTBOT-2
#define RPMMINLOC5            278, RPMMINRECTTOP+15
#define RPMMINRECT6           345, RPMMINRECTTOP, 405, RPMMINRECTBOT
#define RPMMINCLEAR6          346, RPMMINRECTTOP+2, 403, RPMMINRECTBOT-2
#define RPMMINLOC6            346, RPMMINRECTTOP+15
#define RPMMINRECT7           414, RPMMINRECTTOP, 474, RPMMINRECTBOT
#define RPMMINCLEAR7          415, RPMMINRECTTOP+2, 472, RPMMINRECTBOT-2
#define RPMMINLOC7            415, RPMMINRECTTOP+15
#define VERSIONLOC            322, 20




void  DrawPage5()  // History
{

  //tft.setColor(BACKGND);
  if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);
  tft.fillRect(PAGERECT);

  // Outline data regions
  tft.setColor(VGA_BLACK);
  tft.drawRect(TOTALHRSRECT);
  tft.drawRect(TOTALMINRECT);
  tft.drawRect(TOTALSTARTSRECT);
  tft.drawRect(TOTALFUELRECT);
  tft.drawRect(MAINTTIMERECT);
  tft.drawRect(RPMMINRECT1);
  tft.drawRect(RPMMINRECT2);
  tft.drawRect(RPMMINRECT3);
  tft.drawRect(RPMMINRECT4);
  tft.drawRect(RPMMINRECT5);
  tft.drawRect(RPMMINRECT6);
  tft.drawRect(RPMMINRECT7);

  // Labels
  tft.setColor(TEXT);
  //tft.setBackColor(BACKGND);

  if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);
  
  tft.setFont(SmallFont);
  // Show the verson
  tft.print(Version, VERSIONLOC);
  
  tft.setFont(GroteskBold16x32);
  tft.print("HISTORY", HISTORYLABELLOC);
  
  tft.setFont(arial_bold);
  tft.print("TOTAL", ENGINETOTALLABELLOC); 
  tft.print("Hours", TOTALHRSLABELLOC);
  tft.print("Minutes", TOTALMINLABELLOC);
  tft.print("Starts", TOTALSTARTSLABELLOC);
  tft.print("Fuel Used(g)", TOTALFUELLABELLOC);
  tft.setFont(SmallFont);
  tft.print("HOURS SINCE LAST", TIMESINCELASTLABELLOC);
  tft.print("MAINTENANCE", MAINTENANCELABELLOC);
  tft.setFont(arial_bold);
  tft.print("Minutes at RPM", MINATRMPLABELLOC);
  
  tft.setFont(SmallFont);
  tft.print("0-1k    1k-2k    2k-3k    3k-4k   4k-5k    5k-6k    >6k", RPMMINLABELLOC);
  
  
  // Draw maintenance reset button 
  DrawMaintButton();
  

}

void  UpdatePage1()  // Instruments
{
  int speed = engine.Last.rpm;


//  if(WHITESCREEN == true)
//    tft.setColor(BACKGND);
//  else tft.setColor(VGA_YELLOW);

  // Update data
  tft.setColor(TEXT);


if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);
  
  tft.setFont(GroteskBold16x32);

  if((gps->location.isValid())&&(!LATLONSTALE))
  {
    tft.printNumF(engine.Last.lat, 5, LATLOC);
    tft.printNumF(engine.Last.lon, 5, LONLOC);
    
  }
  else
  {
    tft.print("--------", LATLOC);
    tft.print("---------", LONLOC);
  }  

  // GPH
  if((engine.Last.gph > 0) ||(engine.Last.gph < 25.0))
    tft.printNumF(engine.Last.gph, 1, GPHLOC,'.',5);    
  else tft.print("-----",GPHLOC);

  tft.setFont(GroteskBold32x64);
  // clamp
  if (abs(speed) > 9999) speed = 9999;
  tft.printNumI(speed, RPMLOC, 4, ' ');
  tft.printNumI(engine.Last.fuelRemain, FUELLOC, 3, ' ');

  if(/*gps->speed.isValid()&&*/(!SPEEDSTALE)) tft.printNumF(gps->speed.knots(), 1, SPEEDLOC,'.',4);
  else tft.print("----", SPEEDLOC);

  // Battery Voltage
  tft.setFont(BigFont);
  tft.print("Batt. Volts: ",205,220);  
  tft.printNumF(engine.Last.battVoltage, 1, 400,220);

}

void  UpdatePage2()  // Navigation
{

  if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);

  // Update data
  tft.setColor(TEXT);
 
  if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);
  
  tft.setFont(GroteskBold16x32);
  if((gps->location.isValid())&&(!LATLONSTALE))
  {
    tft.printNumF(engine.Last.lat, 5, LATLOC);
    tft.printNumF(engine.Last.lon, 5, LONLOC);
    
  }
  else
  {
    tft.print("--------", LATLOC);
    tft.print("---------", LONLOC);
  }  


  if((CheckRunning())&&(!SPEEDSTALE))

  { 
    // Nautical Miles per Gallon
    tft.printNumF(engine.Last.nmpg, 1, NMPGLOC, '.',5);
     
    // Calculate endurance time

    float enduranceTime = engine.Last.fuelRemain / engine.Last.gph;
    int   enduranceHours = int(enduranceTime);
    int   enduranceMinutes = fmodf(enduranceTime , enduranceHours) * 60;

    tft.printNumI(enduranceHours, EDCTIMELOC,3);
    tft.print(":" , EDCTIMELOCX + 55 , EDCTIMELOCY);
    tft.printNumI(enduranceMinutes, EDCTIMELOCX + 65 , EDCTIMELOCY,2);

    // Calculate endurance miles

    float enduranceMiles = enduranceTime * gps->speed.knots();
    
    tft.printNumF(enduranceMiles, 1, EDCDISTLOC,'.',6);
  } 
     
  else
  {
    tft.print("-----", NMPGLOC);
    tft.print("------", EDCTIMELOC);
    tft.print("------", EDCDISTLOC);
  }    

  tft.setFont(GroteskBold32x64);

  // Speed and Course
  if(!SPEEDSTALE) {
   if(gps->speed.knots() <= 9.9)
    tft.printNumF(gps->speed.knots(), 1, SPEEDLOC2,'.',3);
    else tft.printNumI((int) round(gps->speed.knots()), SPEEDLOC2,3);
    }  
   else tft.print("---", SPEEDLOC2);

  if((!COURSESTALE)&&(gps->course.deg() <= 360.)&&(gps->course.deg() >= 0.))
    tft.printNumI((int)(round(gps->course.deg())), COURSELOC,3);
  else  tft.print("---", COURSELOC);

  // Let's see how many satellites we have
  tft.setFont(BigFont);
  tft.print("Satellites: ",20,220);

  int satellites = gps->satellites.value();
  if (satellites > 99) satellites = 99;
  tft.printNumI(satellites,210,220,2);

  // GPS Time Here
  // Let's adjust and get local (Eastern) time
 
  if(gps->time.isValid())
   {

    int Hour = gps->time.hour();
    bool      DST;
    uint8_t   Day; 
    uint8_t   Month;
    uint16_t  Year;
  
    Hour -= 5;  // Eastern Standard Time Zone correction
    if(Hour < 1) Hour += 12;
  
    //get date from GPS module and check for DST
    if (gps->date.isValid())
    {
      Day   = gps->date.day();
      Month = gps->date.month();
      Year  = gps->date.year();
    
      // Check for DST
      DST = false;
      if((Month > 3)&&(Month < 11)) DST = true;
      
      if(Month == 3)  {
        if (Day >= 12) DST = true;
      }

      if(Month == 11)  {
        if(Day <= 5) DST = true;
      }
      

  
      if(DST == true) Hour +=1;     
    }  // end if date valid
    
     tft.print("Local Time: ",20,240);
    
     tft.printNumI(Hour,200,240,2);
     tft.print(":",233,240);
     tft.printNumI(gps->time.minute(),250,240,2, '0');

   } // end if time is valid
   
   else tft.print("-----",200,240);


   
}




void  UpdatePage3()  //Trip A
{


  if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);


  // Update data
  tft.setColor(TEXT);
 

  if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);

 
  tft.setFont(GroteskBold32x64);


  if(engine.Last.tripAState == RUN)
  {
    tft.setColor(VGA_LIME);
    tft.setBackColor(VGA_LIME);
  }
  else 
  {
    tft.setColor(VGA_RED);
    tft.setBackColor(VGA_RED);
  }
 

  tft.setColor(TEXT);
  tft.setFont(BigFont);
  tft.print("TRIP", TRIPLABELLOC);
  tft.setFont(GroteskBold32x64);
  tft.print("A",TRIPIDLOC);

  tft.setFont(GroteskBold16x32);
  

  if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);

  
  tft.printNumF(engine.Last.tripAMiles, 1, TRIPDISTLOC,'.',4);

  // Calculate trip time
  int   tripHours = engine.Last.tripATime / 60;
  int   tripMinutes = engine.Last.tripATime%60;

  tft.setFont(GroteskBold16x32);
  tft.printNumI(tripHours, TRIPTIMELOCH, 2);
  tft.print(":",TRIPTIMELOCHX+ 60, TRIPTIMELOCHY);
  tft.printNumI(tripMinutes, TRIPTIMELOCM, 2);

  tft.setFont(GroteskBold16x32);
 if((gps->location.isValid())&&(!LATLONSTALE))
  {
    tft.printNumF(engine.Last.lat, 5, LATLOC);
    tft.printNumF(engine.Last.lon, 5, LONLOC);
    
  }
  else
  {
    tft.print("-------", LATLOC);
    tft.print("--------", LONLOC);
  }  
  
  tft.printNumF(engine.Last.tripAFuel, 1, TRIPFUELLOC);
  

  if((gps->speed.isValid())&&(CheckRunning())&&(!SPEEDSTALE)) 
  {
    // Calculate endurance time

    float enduranceTime = engine.Last.fuelRemain / engine.Last.gph;
    int   enduranceHours = int(enduranceTime);
    int   enduranceMinutes = fmodf(enduranceTime , (float)enduranceHours) * 60;

    tft.printNumI(enduranceHours, EDCTIMELOC2,3);
    tft.print(":" , EDCTIMELOCX2 + 55 , EDCTIMELOCY2);
    tft.printNumI(enduranceMinutes, EDCTIMELOCX2 + 65 , EDCTIMELOCY2,2);
    
    
    // Calculate endurance miles

    float enduranceMiles = enduranceTime * engine.Last.knots;
    tft.printNumF(enduranceMiles, 1, EDCDISTLOC2,'.',6);
  }

  else
  {
    tft.print("------", EDCTIMELOC2);      
    tft.print("------", EDCDISTLOC2);
  }
  

}

void  UpdatePage4()  // Trip B
{


 // Clear data
  //tft.setColor(BACKGND);

  if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);


  // Update data
  tft.setColor(TEXT);
  //tft.setBackColor(BACKGND);

if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);
  
  tft.setFont(GroteskBold32x64);


  if(engine.Last.tripBState == RUN)
  {
    tft.setColor(VGA_LIME);
    tft.setBackColor(VGA_LIME);
  }
  else 
  {
    tft.setColor(VGA_RED);
    tft.setBackColor(VGA_RED);
  }
  

  tft.setColor(TEXT);
  tft.setFont(BigFont);
  tft.print("TRIP", TRIPLABELLOC);
  tft.setFont(GroteskBold32x64);
  tft.print("B",TRIPIDLOC);

  tft.setFont(GroteskBold16x32);
 


if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);
  
  tft.printNumF(engine.Last.tripBMiles, 1, TRIPDISTLOC,'.',4);

  // Calculate trip time
  
  int   tripHours = engine.Last.tripBTime / 60;
  int   tripMinutes = engine.Last.tripBTime%60;

  tft.setFont(GroteskBold16x32);
  tft.printNumI(tripHours, TRIPTIMELOCH, 2);
  tft.print(":",TRIPTIMELOCHX+ 60, TRIPTIMELOCHY);
  tft.printNumI(tripMinutes, TRIPTIMELOCM, 2);

  tft.setFont(GroteskBold16x32);
  if((gps->location.isValid())&&(!LATLONSTALE))
  {
    tft.printNumF(engine.Last.lat, 5, LATLOC);
    tft.printNumF(engine.Last.lon, 5, LONLOC);
    
  }
  else
  {
    tft.print("-------", LATLOC);
    tft.print("--------", LONLOC);
  }  

  tft.printNumF(engine.Last.tripBFuel, 1, TRIPFUELLOC);
  
 if((gps->speed.isValid())&&(CheckRunning())&&(!SPEEDSTALE)) 
  {
    // Calculate endurance time

    float enduranceTime = engine.Last.fuelRemain / engine.Last.gph;
    int   enduranceHours = int(enduranceTime);
    int   enduranceMinutes = fmodf(enduranceTime , (float)enduranceHours) * 60;

    tft.printNumI(enduranceHours, EDCTIMELOC2,3);
    tft.print(":" , EDCTIMELOCX2 + 55 , EDCTIMELOCY2);
    tft.printNumI(enduranceMinutes, EDCTIMELOCX2 + 65 , EDCTIMELOCY2,2);
    
    
    // Calculate endurance miles

    float enduranceMiles = enduranceTime * engine.Last.knots;
    tft.printNumF(enduranceMiles, 1, EDCDISTLOC2,'.',6);
  }

  else
  {
    tft.print("------", EDCTIMELOC2);      
    tft.print("------", EDCDISTLOC2);
  }
  


}

void  UpdatePage5()  // History
{


// Clear data
  //tft.setColor(BACKGND);


if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);
  

  // Update data
  tft.setColor(TEXT);


  
  //tft.setBackColor(BACKGND);



  if(WHITESCREEN == true)
    tft.setBackColor(BACKGND);
  else tft.setBackColor(VGA_YELLOW);
  tft.setFont(arial_bold);

  tft.printNumI(engine.Total.hours, TOTALHRSLOC,6);
  tft.printNumI(engine.Total.minutes, TOTALMINLOC,6);
  tft.printNumI(engine.Total.starts, TOTALSTARTSLOC,6);
  tft.printNumI(engine.Total.fuelUsed, TOTALFUELLOC,6);
  tft.printNumI(round((float)engine.Last.maintenanceTimer / 60.), MAINTTIMELOC,6);

  // No easy way...
  tft.setFont(SmallFont);
  tft.printNumI(engine.Total.speed_time_array[0], RPMMINLOC1,7);
  tft.printNumI(engine.Total.speed_time_array[1], RPMMINLOC2,7);
  tft.printNumI(engine.Total.speed_time_array[2], RPMMINLOC3,7);
  tft.printNumI(engine.Total.speed_time_array[3], RPMMINLOC4,7);
  tft.printNumI(engine.Total.speed_time_array[4], RPMMINLOC5,7);
  tft.printNumI(engine.Total.speed_time_array[5], RPMMINLOC6,7);
  tft.printNumI(engine.Total.speed_time_array[6], RPMMINLOC7,7);


}



//*********************************************************************
//
//            Button Response Routines
//
//********************************************************************

void  Button1()  // Instruments
{
  currentDisplayPage  = Instruments;
  
  // blabla 5/11/23 DrawButtons not necessary in 
  // button response routines. Put in for 
  // screen color test.
  
  DrawButtons();
  DrawPage1();
  engine.lastPage = Instruments;
  
}

void  Button2()  // Navigation
{
  currentDisplayPage  = Navigation;
  DrawButtons();
  DrawPage2();
  engine.lastPage = 2;
}

void  Button3()  // Trip A
{
  currentDisplayPage  = TripA;
  DrawButtons();
  DrawPage3();
  engine.lastPage = 3;
}

void  Button4()  // Trip B
{
  currentDisplayPage  = TripB;
  DrawButtons();
  DrawPage4();
  engine.lastPage = 4;
}

void  Button5()  // History
{
  currentDisplayPage  = History;
  if((WHITESCREEN == true)&&(histButtonLatch == false)) WHITESCREEN = false;
   else WHITESCREEN = true;

  if( histButtonState == UP) {
    histButtonState = DOWN;
    
    // Set the latch
    if(!histButtonLatch) histButtonLatch = true;
    if (progResetTimer == 0) {
      Serial.println("Program Reset");
      resetProgStart = millis();
      progResetTimer = 10;
      }
    }
   else {

    // increment program reset timer
    progResetTimer = millis() - resetProgStart;
    }


    if (progResetTimer > MAINT_RESET_TIME)  {
    Serial.println("RESET PROGRAM");
    
    RESETFLAG = true;   
    }
  
  DrawButtons();
  DrawPage5();
  engine.lastPage = 5;
}

void  TripAStartButton()
{
  Serial.println("Trip A Start");
  engine.Last.tripAState = RUN;
  DrawTripButtons();
  // reset starting lat lon
  engine.Last.tripALat = gps->location.lat();
  engine.Last.tripALon = gps->location.lng();
  tripAResetTimer = 0;

  tft.setColor(VGA_LIME);
  tft.setBackColor(VGA_LIME);
  tft.fillRect(TRIPLABELRECT);

}

void  TripAStopButton()
{

  
  if (engine.Last.tripAState == RUN)  { // Initial stop/reset button press
    // Just freeze it
    engine.Last.tripAState = STOP;
    DrawTripButtons();
    // Set latch
    if (!tripAButtonLatch) tripAButtonLatch = true;
    Serial.println("Trip A Stop");
    buttonUpTimer = 0;
    buttonUpStart = millis();
    tripAButtonState = DOWN;
  }

  else if ((engine.Last.tripAState == STOP) && (!tripAButtonLatch)) { //in reset mode
    tripAButtonState = DOWN;
    if (tripAResetTimer == 0) {
      resetAStart = millis();
      tripAResetTimer = 10;
    }
    else {

      // increment trip reset timer
      tripAResetTimer = millis() - resetAStart;

    }

    if (tripAResetTimer > RESET_TIME)  {
      Serial.println("RESET A");
      tripAResetTimer = 0;
      engine.Last.tripAFuel = 0;
      engine.Last.tripAMiles = 0;
      engine.Last.tripATime = 0;
      
      UpdatePage3();
    }
  } // end else if


  tft.setColor(VGA_RED);
  tft.setBackColor(VGA_RED);
  tft.fillRect(TRIPLABELRECT);

  //Save trip state
  UpdateFlash();
  

}

void  TripBStartButton()
{
  Serial.println("Trip B Start");
  engine.Last.tripBState = RUN;
  DrawTripButtons();
  // reset starting lat lon
  engine.Last.tripBLat = gps->location.lat();
  engine.Last.tripBLon = gps->location.lng();
  tripBResetTimer = 0;

  tft.setColor(VGA_LIME);
  tft.setBackColor(VGA_LIME);
  tft.fillRect(TRIPLABELRECT);

}

void  TripBStopButton()
{

  if (engine.Last.tripBState == RUN)  { // Initial stop/reset button press
    // Just freeze it
    engine.Last.tripBState = STOP;
    DrawTripButtons();
    // Set latch
    if (!tripBButtonLatch) tripBButtonLatch = true;
    Serial.println("Trip B Stop");
    buttonUpTimer = 0;
    buttonUpStart = millis();
    tripBButtonState = DOWN;
  }


  else if ((engine.Last.tripBState == STOP) && (!tripBButtonLatch)) { //in reset mode
    tripBButtonState = DOWN;
    if (tripBResetTimer == 0) {
      resetBStart = millis();
      tripBResetTimer = 10;
    }
    else {

      // increment trip reset timer
      tripBResetTimer = millis() - resetBStart;

    }

    if (tripBResetTimer > RESET_TIME)  {
      Serial.println("RESET B");
      tripBResetTimer = 0;
      engine.Last.tripBFuel = 0;
      engine.Last.tripBMiles = 0;
      engine.Last.tripBTime = 0;
      UpdatePage4();
    }
  } // end else if

  tft.setColor(VGA_RED);
  tft.setBackColor(VGA_RED);
  tft.fillRect(TRIPLABELRECT);
  
  //Save trip state
  UpdateFlash();


}

void  MaintResetButton()
{

  if( maintButtonState == UP) {
    maintButtonState = DOWN;
    // Gray the button so the user knows
    tft.setColor(VGA_GRAY);
    tft.setBackColor(VGA_WHITE);
    tft.fillRect(maintButtonRect.left,maintButtonRect.top,maintButtonRect.right,maintButtonRect.bottom);  
    tft.setColor(TEXT);
    tft.setFont(SmallFont);
    tft.print("RESET", maintButtonRect.left+10,maintButtonRect.top+15);
    // Set the latch
    if(!maintButtonLatch) maintButtonLatch = true;
    if (maintResetTimer == 0) {
      Serial.println("Maintenance Reset");
      resetMaintStart = millis();
      maintResetTimer = 10;
      }
    }
   else {

    // increment maintenance reset timer
    maintResetTimer = millis() - resetMaintStart;
    }


    if (maintResetTimer > MAINT_RESET_TIME)  {
    Serial.println("RESET MAINTENANCE");
    maintResetTimer = 0;
    engine.Last.maintenanceTimer = 0;      
    //Save state
    UpdateFlash();
    }
  
}


//*****************************************************************
//
//       Touch Controller Routines
//
//****************************************************************

uint8_t readFT6236TouchRegister( uint8_t reg )
{
  Wire.beginTransmission(addr);
  Wire.write( reg );  // register 0
  uint8_t retVal = Wire.endTransmission();

  uint8_t returned = Wire.requestFrom(addr, uint8_t(1) );    // request 6 uint8_ts from slave device #2

  if (Wire.available())
  {
    retVal = Wire.read();
  }

  return retVal;
}

uint8_t readFT6236TouchAddr( uint8_t regAddr, uint8_t * pBuf, uint8_t len )
{
  Wire.beginTransmission(addr);
  Wire.write( regAddr );  // register 0
  uint8_t retVal = Wire.endTransmission();

  uint8_t returned = Wire.requestFrom(addr, len);    // request 1 bytes from slave device #2

  uint8_t i;
  for (i = 0; (i < len) && Wire.available(); i++)
  {
    pBuf[i] = Wire.read();
  }

  return i;
}

uint8_t readFT6236TouchLocation( TouchLocation * pLoc, uint8_t num )
{
  uint8_t retVal = 0;
  uint8_t i;
  uint8_t k;

  do
  {
    if (!pLoc) break; // must have a buffer
    if (!num)  break; // must be able to take at least one

    uint8_t status = readFT6236TouchRegister(2);

    static uint8_t tbuf[40];

    if ((status & 0x0f) == 0) break; // no points detected

    uint8_t hitPoints = status & 0x0f;

    //Serial.print("number of hit points = ");
    //Serial.println( hitPoints );

    readFT6236TouchAddr( 0x03, tbuf, hitPoints * 6);

    for (k = 0, i = 0; (i < hitPoints * 6) && (k < num); k++, i += 6)
    {
      pLoc[k].x = (tbuf[i + 0] & 0x0f) << 8 | tbuf[i + 1];
      pLoc[k].y = (tbuf[i + 2] & 0x0f) << 8 | tbuf[i + 3];
    }

    retVal = k;

  } while (0);

  return retVal;
}

void writeFT6236TouchRegister( uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(addr);
  Wire.write( reg );  // register 0
  Wire.write( val );  // value

  uint8_t retVal = Wire.endTransmission();
}

//********************************************************************
//
//       Touch Utility Routines
//
//********************************************************************

uint32_t dist(const TouchLocation & loc)
{
  uint32_t retVal = 0;

  uint32_t x = loc.x;
  uint32_t y = loc.y;

  retVal = x * x + y * y;

  return retVal;
}
uint32_t dist(const TouchLocation & loc1, const TouchLocation & loc2)
{
  uint32_t retVal = 0;

  uint32_t x = loc1.x - loc2.x;
  uint32_t y = loc1.y - loc2.y;

  retVal = sqrt(x * x + y * y);

  return retVal;
}

bool sameLoc( const TouchLocation & loc, const TouchLocation & loc2 )
{
  return dist(loc, loc2) < 50;
}


bool  IsInRect(TouchLocation point, Rect rect)
{

  
  if (((point.x >= rect.left) && (point.x <= rect.right)) &&
      ((point.y >= rect.top) && (point.y <= rect.bottom)))
    return true;

  else return false;


}

//**************************************************************
//
//   Paints LCD black
//
//*************************************************************


void  screenOff()
{
  Serial.println("screen off");
  engine.screenStatus = OFF;
  
}

void  screenOn()
{
  Serial.println("screen on");
  
  engine.screenStatus = ON;
  
  // Clear the screen and draw the frame
  tft.clrScr();
  //tft.setColor(BACKGND);

if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);

  
  tft.fillRect(SCREEN);
  currentDisplayPage = Instruments;
  SetupButtons();
  DrawButtons();
  DrawPage1();
  
}


//****************************************************************
//
//   GetBattVoltage measures the current battery voltage applied
//    to the Arduino analog pin.
//
//***************************************************************

float GetBattVoltage()
{

  return (analogRead(battPin) * 5.0  ) / ( BATTVOLTAGEDIV * 1023);

}



//****************************************************************
// GetSpeedandFuel monitors the analog signal from one of the
// engine's fuel injectors waiting for the drop in voltage that
// signals the start of fuel injection. The duration of the signal
//  directly corresponds to the fuel injected into the cylinder.
// The period between the end of one injection signal and the end
// of the next can be used to calculate engine speed.
//****************************************************************

void GetSpeedandFuel()
{
    // increment count of pulses received in this one second interval
   intervalPulseCount++;
   
  // Pulse width in microseconds
  unsigned long fuelPulseWidth;

  // Current time, time of last pulse start, current pulse and pulse duration

  fuelPulseWidth = finish - start;

  // Correct for micros() overflow
  if (fuelPulseWidth < 0) {
    fuelPulseWidth = finish + (maxMicros - start);
  }


  // calculate rpm
  thisPulse = finish;  //revolutions counted from end of fuel injection signal

  // Correct for micros() overflow
  if (thisPulse > lastPulse) pulsePeriod = thisPulse - lastPulse;
  else  // overflow
  {
    pulsePeriod = thisPulse + (maxMicros - lastPulse);
  }


  // Signal generator sine wave input produces some crazy numbers
  if(fuelPulseWidth > pulsePeriod) // bad signal
  {
   Serial.println("BAD INJECTION SIGNAL");
   return; 
  }
    

  
  //restart rpm calculation period
  lastPulse = thisPulse;


  // Calculate engine speed and injected fuel quantity per cylinder
  rpm =  (120. * pow(10, 6)) / pulsePeriod;
  // clamp for display
  if (abs(rpm) > 9999.) rpm = 9999.;

  // Fuel quantity is in terms of microseconds per millicc (nominally 285).
  // so, millicc = microseconds / 285
  fuelQty = fuelPulseWidth / MICROSECPERMILLICC ;

  //  total fuel (gallons)
  cycleFuel += (4.0 * fuelQty * GALLONSPERCC) / 1000; // (4 cyl.* mCC * (gal / cc)) /1000
  
  // clamp
  //if(cycleFuel > .1) cycleFuel = .1;
 
  if (PULSEDIAGNOSTICS == true)
  {
    Serial.print("Pulse Period:  "); Serial.println(pulsePeriod);
    Serial.print("Pulse width:  "); Serial.println(fuelPulseWidth);
    Serial.print("RPM:  "); Serial.println(rpm);
  }


  if (REALTIMEENGINE == true)
  {
    Serial.print("RPM:  ");
    Serial.println(rpm);
    Serial.print("Fuel:  ");
    Serial.println(fuelQty);
    Serial.print("Cycle Fuel: ");
    Serial.println(cycleFuel,8);
  }


}


//********************************************************************
//
// GetFuelLevel reads the tank sender voltage and calculates the
//   current level.
//
//********************************************************************

void  GetFuelLevel()
{
  int level = analogRead(fuelPin);
  float vFuel = (float)(analogRead(fuelPin)) / 1023. * 5.0 / FUELVOLTAGEDIV;

  // Old Level Sender
  //fuelLevel = (round((1 - ((vFuel - 2.0) / 3.7)) * 8.0) / 8.0 ) * TANKCAPACITY;

  // New Level Sender 5/30/23
   //fuelLevel = 1.2 *((1 - ((vFuel - 1.98) / 3.79)))  * TANKCAPACITY;

  // Function calculated from tests with current meter and sender 6/1/23
  fuelLevel = TANKCAPACITY * (1.106132+(0.03121199*vFuel)-(0.04507456*vFuel*vFuel)+(0.001367488*pow(vFuel,3))); 
 
  if (fuelLevel < 0) fuelLevel = 0;
  if (fuelLevel > TANKCAPACITY) fuelLevel = TANKCAPACITY;

  if (REALTIMEENGINE == true)
  {
    Serial.print("Battery Voltage: ");
    Serial.println(engine.Last.battVoltage);
    Serial.print("Fuel Pin Voltage: ");
    Serial.println((analogRead(fuelPin) * 5.0) / 1023., 3);
    Serial.print("Fuel Level Voltage: ");
    Serial.println(vFuel, 3);
    Serial.print("Fuel Level: ");
    Serial.println(fuelLevel);
  }


}

//*********************************************************************
//
//  SmoothFuel averages the fuel level reading over the last 10
//   seconds to compensate for sloshing
//
//********************************************************************

unsigned long SmoothFuel()
{

  // shift the history
  for (int i = 0; i < 10; i++) {
    fuelTankLevel[i] = fuelTankLevel[i + 1];

  }

  // add the latest
  fuelTankLevel[9] = fuelLevel;

  // calculate the average speed
  unsigned long sum = 0;
  for (int i = 0; i < 10; i++)
    sum += fuelTankLevel[i];

  return sum / 10;
}





//*********************************************************************
// SmoothSpeeds using moving average tecchnique to smooth the rpm data.
//*********************************************************************

unsigned long SmoothSpeeds()
{

// When the one second or one minute timers interrupt to perform the 
// required methods, fuel injection interrupts are ignored and several
// fuel injection pulses are missed. The way we can see this is in the 
// rpms calculated using the very long pulse intervals in GetSpeedandFuel().
// This results in the latest rpm being reduced multiples of the last rpm.
// This multiple is the number of missed injections, and we can correct the
// speed calculations.
// 



  int missedPulses;

  // shift the history down
  for (int i = 0; i < SPEEDARRAYSIZE-1; i++) {
    if (abs(speeds[i]) > 9999) speeds[i] = 9999; 
    speeds[i] = speeds[i + 1];    
  }

  // add the latest
  if((speeds[SPEEDARRAYSIZE - 1] / rpm < 2)||(missedISR == true)) // All OK
  {
    speeds[SPEEDARRAYSIZE - 1] = rpm;
    missedISR= false;
  }
 
    else {  // missed interrupt(s)
      missedISR = true;
      missedPulses = speeds[SPEEDARRAYSIZE - 1] / rpm;

      // Use previous rpm for average
      speeds[SPEEDARRAYSIZE - 1] = speeds[SPEEDARRAYSIZE - 2]; 
    }

  // calculate the average speed
  unsigned long sum = 0;
  for (int i = 0; i < SPEEDARRAYSIZE; i++)
  {
    sum += speeds[i];
    
  }

  return sum / SPEEDARRAYSIZE;
}


//********************************************************************
//
//     UpdateSpeedRanges determines which of 6 speed ranges the
//      current engine speed falls into. These are the data used
//      in the permanent memory
//
//*********************************************************************
void UpdateSpeedRanges(int speed)
{

  int index = speed / 1000;
  if (speed > 6) speed = 6;
  speed_time_accum[index]++;


}

//*******************************************************************
//
//  UpdateSpeedArray adds the accumulated speed range data from the
//   last minute interval to the engine total parameters before
//   writing to memory
//
//******************************************************************

void  UpdateSpeedArray()
{
  int index, minutes;

  for (index = 0; index < 7; index++)  {
    minutes = speed_time_accum[index] / 60;
    if (minutes >= 1) {
      engine.Total.speed_time_array[index] += minutes;
      speed_time_accum[index] -= 60 * minutes;
    }

  }

  if (SPEEDARRAYDIAGNOSTICS == true)
  {
    for (index = 0; index < 7; index++)
    {
      Serial.println("Speed Times: ");
      Serial.println(engine.Total.speed_time_array[index]);
    }

  }


}


//*********************************************************************
//
// CalcGPH caluclates gph
//
//********************************************************************
void  CalcGPH()
{


  // If engine not running, don't risk crazy calculations
  if(intervalPulseCount == 0) {
    engine.Last.gph = 0;
    return;
  }
 
  engine.Last.gph = (cycleFuel /  intervalPulseCount) * engine.Last.rpm * 30;
  // clamp
  if (engine.Last.gph > 25.) engine.Last.gph = 25.;


  // Update fuel usage totals
  engine.Total.fuelUsed += engine.Last.gph/3600.;
  if (engine.Last.tripAState == RUN) engine.Last.tripAFuel += engine.Last.gph/3600.;
  if (engine.Last.tripBState == RUN) engine.Last.tripBFuel += engine.Last.gph/3600.;
  

  lastFuelQty = engine.Total.fuelUsed;
  engine.Last.range = (engine.Last.fuelRemain * engine.Last.knots) / engine.Last.gph;

  engine.Last.nmpg = engine.Last.knots / engine.Last.gph;
  if(engine.Last.nmpg > 1000.) engine.Last.nmpg = 1000.;
  
  if (REALTIMEENGINEFUEL == true)
  {

    Serial.print("GPH: ");
    Serial.println(engine.Last.gph, 8);
    Serial.print("Total Fuel Used: ");
    Serial.println(engine.Total.fuelUsed);
    Serial.print("Range: ");
    Serial.println(engine.Last.range);
  }


}


//*********************************************************************
//
//          GPSUpdate reads the data from the gps and updates position
//              speed, and course data
//
//********************************************************************

void  GPSUpdate()
{


  // We need to watch out for stale data - mostly course and speed.
  // We'll set flags for display purposes.
  // 10/13/23 Let's play with this.

  COURSESTALE = false;
  SPEEDSTALE = false;
  
  if(gps->course.age() > 5000) COURSESTALE = true;
  else COURSESTALE = false;
  
  if(gps->speed.age() > 5000) SPEEDSTALE = true;
  else SPEEDSTALE = false;

  if(gps->location.age() > 5000) LATLONSTALE = true;
  else LATLONSTALE = false;


  //We need to start the stale timer if any of the data 
  // becomes stale. At the timeout, we'll destroy the 
  //  gps object and recreate it.
  if((STALEFLAG == false)&&((COURSESTALE==true) || (SPEEDSTALE==true)
  || (LATLONSTALE==true) || (gps->satellites.value() == 0)))
    {
      STALEFLAG = true;
      staleTimer = 0;
      Serial.println(gps->satellites.value());
    }

  
  
  if ((gps->location.isValid())||(gps->speed.isValid()))
  {
      
    engine.Last.knots = gps->speed.knots();
    engine.Last.lat = gps->location.lat();
    engine.Last.lon = gps->location.lng();
    engine.Last.course = gps->course.deg();

  }

 

 if (GPSDIAGNOSTICS == true)
    { 
      
      if(LATLONSTALE) Serial.println("LatLon Stale");
      if(COURSESTALE) Serial.println("Course Stale");
      if(SPEEDSTALE) Serial.println("Speed Stale");
      if(STALEFLAG) Serial.println("STALEFLAG ON");
      Serial.print("Stale Timer: ");
      Serial.println(staleTimer);
      
      Serial.print("Sentences with Fix: ");
      Serial.println(gps->sentencesWithFix());
      Serial.print("Satellite Chars Processed: ");
      Serial.println(gps->charsProcessed());
      Serial.print("# of satellites: ");
      Serial.println(gps->satellites.value());
      Serial.print("Location Age: ");
      Serial.println(gps->location.age());
      Serial.print("Course Age: ");
      Serial.println(gps->course.age());
      Serial.print("Speed Age: ");
      Serial.println(gps->speed.age());
      Serial.print("Latitude = ");
      Serial.println(gps->location.lat(), 6);
      Serial.print("Longitude = ");
      Serial.println(gps->location.lng(),6);
      Serial.print("Speed = ");
      Serial.println(engine.Last.knots);
      Serial.print("Course = ");
      Serial.println(engine.Last.course);
    }

    //if(gps->satellites.value() == 0) Serial.println("NO SATELLITES");

}

//********************************************************************
//
//  UpdateTrips uses time, fuel and speed information to update
//   trip data
//
//*******************************************************************

void  UpdateTrips()
{


  double  distance = 0;
 
  if (engine.Last.tripAState == RUN)
  {
    engine.Last.tripATime++;

    // distance for first gps fix can be enormous since last position is 0,0
    if((abs(gps->location.lat())>0)&&(abs(gps->location.lng())>0)&&(!LATLONSTALE))
    {
      distance = TinyGPSPlus::distanceBetween(engine.Last.tripALat, engine.Last.tripALon, gps->location.lat(), gps->location.lng());
  
      //clamp
      if(distance > 5000) distance = 0.;
      distance *= 0.00062137; // miles per meter
      engine.Last.tripAMiles += distance;
      engine.Last.tripALat = gps->location.lat();
      engine.Last.tripALon = gps->location.lng();
    }
    

  }

  distance = 0;
  
  if (engine.Last.tripBState == RUN)
  {
    engine.Last.tripBTime++;
    if((abs(gps->location.lat())>0)&&(abs(gps->location.lng())>0)&&(!LATLONSTALE))
    {
       distance = TinyGPSPlus::distanceBetween(engine.Last.tripBLat, engine.Last.tripBLon, gps->location.lat(), gps->location.lng());
  
      //clamp
      if(distance > 5000) distance = 0.;
      distance *= 0.00062137; // miles per meter
      engine.Last.tripBMiles += distance;
      engine.Last.tripBLat = gps->location.lat();
      engine.Last.tripBLon = gps->location.lng();
    }

   
  }

}



//*********************************************************************
// CheckRunning looks athe the interval since the last injection pulse
// was observed and determines if the engine is running or not.
//*********************************************************************

bool  CheckRunning()
{
  unsigned long interval, now = micros();

  // Correct for micros() overflow
  if (now > lastPulse) interval = now - lastPulse;
  else  // overflow
  {
    interval = now + (maxMicros - lastPulse);
  }
  if (interval < ENGINESTOPDELAY) return true;
  else return false;

}



//***********************************************************
// serialEvent alerts the Arduino
// whenever a serial event occurs. In our case, we check
// for available input data and concatenate a command
// string, setting a boolean used by the loop() routine
// as a dispatch trigger.
//***********************************************************

void serialEvent()
{
  char c;
  while (Serial.available()) {
    c = (char)Serial.read();
    if (c == ';') {
      g_command_ready = true;
    }
    else {
      g_command += c;
    }
  }
}

//**************************************************************
//
//     ProccessCommand takes a string passed from the serial
//      connection and executes the user's commands
//
//**************************************************************

void  ProcessCommand(String command)
{

  Serial.println(command);
  if (command == "memorydiagnostics_on")
    MEMORYDIAGNOSTICS = true;
  if (command == "memorydiagnostics_off")
    MEMORYDIAGNOSTICS = false;

  if (command == "realtimeengine_on")
    REALTIMEENGINE = true;
  if (command == "realtimeengine_off")
    REALTIMEENGINE = false;

  if (command == "pulsediagnostics_on")
    PULSEDIAGNOSTICS = true;
  if (command == "pulsediagnostics_off")
    PULSEDIAGNOSTICS = false;

  if (command == "speedarraydiagnostics_on")
    SPEEDARRAYDIAGNOSTICS = true;
  if (command == "speedarraydiagnostics_off")
    SPEEDARRAYDIAGNOSTICS = false;

  if (command == "realtimeenginefuel_on")
    REALTIMEENGINEFUEL = true;
  if (command == "realtimeenginefuel_off")
    REALTIMEENGINEFUEL = false;

  if (command == "gpsdiagnostics_on")
    GPSDIAGNOSTICS = true;
  if (command == "gpsdiagnostics_off")
    GPSDIAGNOSTICS = false;

  if (command == "listparameters")
    ListParameters();

  g_command = "";
  g_command_ready = false;
}

//**************************************************************
//
//  OnSecondperforms some timer functions
//   in the main loop
//
//*************************************************************
void  OnSecond()
{
  secondFlag = true;
}


//**************************************************************
//
//  OnMinute performs memory updating and other time functions
//   in the main loop
//
//*************************************************************
void  OnMinute()
{

  //Serial.println("OnMinute");
  if (CheckRunning()) {
    engine.Last.maintenanceTimer++;
    engine.Total.minutes++;
    if (engine.Total.minutes % 60 == 0)
    {
      engine.Total.hours++;
      engine.Total.minutes = 0;
    }
  }

  // Time to write memory values
  minuteFlag = true;


}




//************************************************************
//
//   ListParameters sends the current engine struct to the
//    serial port.
//
//************************************************************
void  ListParameters()
{

  Serial.println(engine.Total.hours);
  Serial.println(engine.Total.minutes);
  Serial.println(engine.Total.starts);
  for (int i = 0; i < 7; i++) {
    Serial.print(engine.Total.speed_time_array[i]); //0-1k,1-2k,2-3k,3-4k,4-5k,5-6k,6+k
    Serial.print(", ");
  }
  Serial.println();
  Serial.println(engine.Total.fuelUsed);

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println(engine.Last.rpm);
  Serial.println(engine.Last.battVoltage);
  Serial.println(engine.Last.lat);
  Serial.println(engine.Last.lon);
  Serial.println(engine.Last.knots);
  Serial.println(engine.Last.range);  // nautical miles
  Serial.println(engine.Last.course);  // degrees
  Serial.println(engine.Last.fuelRemain);  // gallons
  Serial.println(engine.Last.fuelAmount);
  Serial.println(engine.Last.gph);
  Serial.println(engine.Last.tripAFuel);
  Serial.println(engine.Last.tripBFuel);
  Serial.println(engine.Last.tripAMiles); // nautical miles
  Serial.println(engine.Last.tripBMiles);
  Serial.println(engine.Last.tripAState);
  Serial.println(engine.Last.tripBState);
  Serial.println(engine.Last.tripATime);  // minuutes
  Serial.println(engine.Last.tripBTime);
  Serial.println(engine.Last.progTime);
  Serial.println(engine.Last.maintenanceTimer);
  Serial.println(engine.Last.pulseWidth);



}

//************************************************************
//
//  ResetMemoryData is used when main flash memory bank is
//    corrupted.
//************************************************************

void  ResetMemoryData()
{
  int   i;

  // Try backup
  ReadMemory(BACKUPMEMORYADDRESS);
  if (!ERASEDDATA)  {
    Serial.println("Succcessful restore");
    return;
  }
  else  // Start all over
  {
    Serial.println("RESTORE FAILED - resetting values");
    ZeroData();

  }


}

//*************************************************************
//
//  ZeroData zeroes the flash memory parameters
//
//************************************************************

void  ZeroData()
{
  engine.Total.hours = 0;
  engine.Total.minutes = 0;
  engine.Total.starts = 0;
  for (int i = 0; i < 7; i++ )
    engine.Total.speed_time_array[i] = 0;
  engine.Total.fuelUsed = 0;


  engine.Last.rpm = 0;
  engine.Last.battVoltage = 0.0;
  engine.Last.fuelRemain = 0;
  engine.Last.fuelAmount = 0;
  engine.Last.gph = 0.0;
  engine.Last.rpm = 0.0;
  engine.Last.tripAFuel = 0.0;
  engine.Last.tripBFuel = 0.0;
  engine.Last.tripAMiles = 0;
  engine.Last.tripBMiles = 0;
  engine.Last.tripAState = STOP;
  engine.Last.tripBState = STOP;
  engine.Last.tripATime = 0;
  engine.Last.tripATime = 0;
  engine.Last.progTime = 0;
  engine.Last.maintenanceTimer = 0;
  engine.Last.pulseWidth = 0;

}


//**************************************************************
//
//
//                           Memory Routines
//
//
//*************************************************************

//*************************************************************
//
//  Write Memory writes the array of engine permament
//  parameters to flash memory
//
//************************************************************
bool  WriteMemory(uint32_t addr)
{

  // Before we write memory, we need to make sure some
  // parameters that can really mess us up are OK
  MemoryParameterCheck();


  // We must erase memory before writing to it
  if (MEMORYDIAGNOSTICS == true)
  {
    Serial.print("Erasing: "); Serial.println(addr);
  }
  if (!flash.eraseSection((uint32_t)addr, sizeof(Engine))) {

    Serial.println("Memory erase failure");
    return false;
  }


  // Write the array
  if (MEMORYDIAGNOSTICS == true)
    Serial.print("Writing: ");
  //Serial.println(addr);
  if (flash.writeAnything((uint32_t)addr, engine)) {
    if (MEMORYDIAGNOSTICS == true)
    {
      Serial.println("WRITE:");
      Serial.println(engine.Total.hours);
      Serial.println(engine.Total.minutes);
      Serial.println(engine.Total.starts);
      Serial.println(engine.Total.fuelUsed, 8);
      Serial.println(engine.Last.rpm);
    }

  }
  else {
    Serial.println("Write Memory ERROR");
    return false;
  }

  if (MEMORYDIAGNOSTICS == true)
    Serial.println("Write Memory SUCCESS");

  return true;

}


//*********************************************************************************
//
//  ReadMemory reads the permanent parameters from flash memory
//
//********************************************************************************
bool  ReadMemory(uint32_t addr)
{


  if (flash.readAnything((uint32_t)addr, engine))  {
    if (MEMORYDIAGNOSTICS == true)
    {
      Serial.println("READ:");
      Serial.println(engine.Total.hours);
      Serial.println(engine.Total.minutes);
      Serial.println("Speed Time Array 0-1k, 1-2k, 2-3k, 3-4k, 4-5k, 5-6k, >6k");
      for(int i = 0; i < 6; i++){
        Serial.print(engine.Total.speed_time_array[i]);
        Serial.print(", ");
      }
      Serial.println(engine.Total.speed_time_array[6]);
      Serial.println(engine.Total.starts);
      Serial.println(engine.Total.fuelUsed, 8);
      Serial.println(engine.Last.rpm);
    }

  }
  else {
    Serial.println("Read Memory ERROR");

    return false;
  }
  if (MEMORYDIAGNOSTICS == true)
    Serial.println("Read Memory SUCCESS");

}

//**************************************************************
//
//    UpdateFlash writes the stored parameter blocks to flash
//     memory and its backup location
//
//*************************************************************

void  UpdateFlash()
{


  // Don't write erased data
  if (!ERASEDDATA) {
    if (WriteMemory(ENGINEMEMORYADDRESS)) {
      if (MEMORYDIAGNOSTICS == true)
        Serial.println("Backing up");
      if (WriteMemory(BACKUPMEMORYADDRESS) && (MEMORYDIAGNOSTICS == true))
        Serial.println("Backup SUCCESS");
    }
    else ResetMemoryData();
  }

  ReadMemory(ENGINEMEMORYADDRESS);



}

void  MemoryParameterCheck()
{
  //Some sanity checks
  if(engine.Last.tripAState > RUN) engine.Last.tripAState = RUN;
  if(engine.Last.tripBState > RUN) engine.Last.tripBState = RUN;


  
}


//**************************************************************
//
//
//                           Interrupt Service Routines
//
//
//*************************************************************

void  StartPulse()
{

  {
    pulseReceived = false;
    start = micros();
    pulseStart = true;
  }

}

void  EndPulse()
{

  finish = micros();
  pulseReceived = true;

}

//************************************************************
//  One second timer
//
//************************************************************
ISR(TIMER1_COMPA_vect)
{

  programSeconds++;

  OnSecond();

  if (programSeconds % 60 == 0)  {
    programMinutes++;
    OnMinute();

  }
}



//************************************************************
//                 resetFunc
//  Restarts the program. Used for some detected errors
//
//*************************************************************

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0







//************************************************************
//
//                   Standard Arduio Routines
//
//************************************************************


void setup()
{
  int   i;

  Serial.begin(115200);
  Serial2.begin(9600); // connect gps sensor

  Serial.println("Program Start");

  //Our stuff

  // GPS
  gps = new TinyGPSPlus;
  
  pinMode(injPinA, INPUT_PULLUP);
  pinMode(injPinB, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(fuelPin, INPUT);
  pinMode(battPin, INPUT);
  pinMode(screenPinOff, INPUT);
  pinMode(screenPinOn, INPUT);
  digitalWrite(ledPin, LOW);

  //  Display setup
  Wire.begin();        // join i2c bus (address optional for master)

  // Setup the LCD
  tft.InitLCD();
  // -------------------------------------------------------------
  pinMode(8, OUTPUT);  //backlight
  digitalWrite(8, HIGH);//on
  // -------------------------------------------------------------

  writeFT6236TouchRegister(0, eNORMAL); // device mode = Normal

  uint8_t  lenLibVersion = readFT6236TouchAddr(0x0a1, buf, 2 );
  if (lenLibVersion)
  {
    uint16_t libVersion = (buf[0] << 8) | buf[1];

    Serial.print("lib version = ");
    Serial.println( libVersion, HEX);
  }
  else
  {
    Serial.println("lib version length is zero");
  }

  uint8_t firmwareId = readFT6236TouchRegister( 0xa6 );

  Serial.print("firmware ID = ");
  Serial.println( firmwareId);

  pinMode(FT6236_INT, INPUT);

  currentDisplayPage  = Instruments;
  tripAButtonState = UP;
  tripBButtonState = UP;
  maintButtonState = UP;
  histButtonState = UP;
  
  tripAButtonLatch = false;
  tripBButtonLatch = false;
  maintButtonLatch = false;
  histButtonLatch = false;
  buttonUpTimer = 0;
  buttonUpStart = 0;


  engine.Last.tripAState = STOP;
  engine.Last.tripBState = STOP;
  tripAResetTimer = 0;
  tripBResetTimer = 0;
  maintResetTimer = 0;
  progResetTimer = 0;
  

  engine.Last.course = 0;
  engine.Last.knots = 0;
  engine.Last.tripATime = 0;
  engine.Last.tripBTime = 0;
  engine.Last.tripAMiles = 0;
  engine.Last.tripBMiles = 0;
  engine.Last.tripAFuel = 0;
  engine.Last.tripBFuel = 0;
  engine.Last.tripALat = 0;
  engine.Last.tripALon = 0;
  engine.Last.tripBLat = 0;
  engine.Last.tripBLon = 0;
  engine.lastPage = 1;
  engine.RESTART = false;
  engine.screenStatus = OFF;

  
  WHITESCREEN = true;

  // Clear the screen and draw the first page
  tft.clrScr();
  if(WHITESCREEN == true)
    tft.setColor(BACKGND);
  else tft.setColor(VGA_YELLOW);
  tft.fillRect(SCREEN);
  SetupButtons();
  DrawButtons();
  DrawPage1();

  // Accumulated ignition pulses
  intervalPulseCount = 0;
  


  // Clear the memory and parameters
  ZeroData();
  for (i = 0; i < 7; i++ )
    speed_time_accum[i] = 0;
  lastFuelQty = 0;

  

  // Set flags
  pulseReceived = false;
  pulseStart = false;
  running = true;
  lastPulse = micros();
  secondFlag = false;
  minuteFlag = false;
  STALEFLAG = false;
  COURSESTALE = false;
  SPEEDSTALE = false;

  programTime = 0;
  motor_timer = 0;

  RESETFLAG = false;
  


  // initiallize smoothing arrays
  for (i = 0; i < SPEEDARRAYSIZE; i++)
    speeds[i] = 0;
  for (int j = 0; j < 10; j++)
    fuelTankLevel[j] = engine.Last.fuelRemain;

  // initialize for engine running detection
  lastPulse = micros();

  // ISR setup
  missedISR = false;  // flag for rpm and fuel calculation
  
  attachInterrupt(digitalPinToInterrupt(injPinA), StartPulse, FALLING);
  attachInterrupt(digitalPinToInterrupt(injPinB), EndPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(screenPinOn), screenOn, RISING);
  attachInterrupt(digitalPinToInterrupt(screenPinOff), screenOff, FALLING);

  // Timers
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = secondReload;
  TCCR1B = (1 << WGM12) | (1 << CS12);
  TIMSK1 = (1 << OCIE1A);
  sei();


  // Flash memory
  flash.begin();
  ReadMemory(ENGINEMEMORYADDRESS);
  if (ERASEDDATA) {
    Serial.println("Bad data - trying restore");
    ResetMemoryData();
  }

  if(engine.RESTART)
  {
    // First, let's clamp the stored page number.
    if((engine.lastPage < Instruments) ||(engine.lastPage > History))
      engine.lastPage = Instruments;

    // Now lets restore the page displayed before the restart
    currentDisplayPage = engine.lastPage;
    switch(engine.lastPage) {
      case Instruments:
       DrawPage1();
       break;
  
     case Navigation:
      DrawPage2();
      break;
  
     case TripA:
      DrawPage3();
      break;
  
     case TripB:
      DrawPage4();
      break;
  
     case History:
      DrawPage5();
      break;
    }  // end switch
      
   engine.RESTART = false;
  }  // end if engine.restart




  
  Serial.println("SETUP COMPLETE OK");

  // Memory Synchronization with engine 05/19/24
//  engine.Total.hours = 477;
//  engine.Total.minutes = 14;
//  engine.Total.fuelUsed = 706.5;
//  engine.Total.speed_time_array[0] = 4540;
//  engine.Total.speed_time_array[1] = 7584;
//  engine.Total.speed_time_array[2] = 4294;
//  engine.Total.speed_time_array[3] = 10302;
//  engine.Total.speed_time_array[4] = 1578;
//  engine.Total.speed_time_array[5] = 336;
//  engine.Total.speed_time_array[6] = 0;
//  engine.Last.maintenanceTimer = 0;
  


}

//*****************************************************
//
//
//              MAIN LOOP
//
//
//*****************************************************

void loop()
{

  // 2/5/24 - Let's move the GPS encoding to the top of the loop
  // GPS Query must be outside of time interrupt routines
  while (Serial2.available() > 0)
    gps->encode(Serial2.read());


  // Button Logic:

  // The FT6236 interrupt line provides alert to new touches, but does not refresh fast
  // enough to cover each loop. We can, however, query the chip each time.
  // For the trip stop / reset and maintenance reset buttons we need some special logic.
  // For those buttons, we need to detect continuous "press"ing for a fixed period of time.
  // For the trip buttons we need to detect a lift from the stop button so we can
  // change the function to reset.
  // We can detect continuous "press" ing in the loop by querrying the FT6236 for count
  // and location. If the count remains 1 and the current location is close to the
  // original button press location, we're still down. A lift will cause the count to
  // go to zero for some period of time.


  //  Touch screen loop
  uint8_t attention = digitalRead(FT6236_INT);
  static uint8_t oldAttention = 1;
  uint16_t xy = 0;
  uint8_t  count = 0;

  /* Wait around for touch events */
  if (!attention && oldAttention ) // A new touch somewhere
  {
    //Serial.println("Touch: ");

    count = readFT6236TouchLocation( touchLocations, 2 );
    if (count)  // == 1 Allow only one touch
    {
      for (int i = 0; i < count; i++)
      {
        xy = touchLocations[i].x;
        touchLocations[i].x = 480 - touchLocations[i].y;
        touchLocations[i].y = xy;
     }  // end for i

      // check for button push
      if (IsInRect(touchLocations[0], button1Rect))
        Button1();

      if (IsInRect(touchLocations[0], button2Rect))
        Button2();

      if (IsInRect(touchLocations[0], button3Rect))
        Button3();

      if (IsInRect(touchLocations[0], button4Rect))
        Button4();

      if (IsInRect(touchLocations[0], button5Rect))
        Button5();

      if ((currentDisplayPage  == TripA) && (IsInRect(touchLocations[0], startButtonRect))
          && (engine.Last.tripAState == STOP))
        TripAStartButton();

      if ((currentDisplayPage  == TripB) && (IsInRect(touchLocations[0], startButtonRect))
          && (engine.Last.tripBState == STOP))
        TripBStartButton();

        if(IsInRect(touchLocations[0], startButtonRect)){
          
        }

      if ((currentDisplayPage  == TripA) && (IsInRect(touchLocations[0], stopButtonRect)))
        TripAStopButton();

      if ((currentDisplayPage  == TripB) && (IsInRect(touchLocations[0], stopButtonRect)))
        TripBStopButton();

      if ((currentDisplayPage  == History) && (IsInRect(touchLocations[0], maintButtonRect)))
        MaintResetButton();


    } // end if count


  }  // end a touch somewhere

  oldAttention = attention;



  // See if the touch is gone
  // If trip button, maintenance reset, or history latchs are set and the touch count has been zero
  // for a timer amount greater than the limit, change the button states to UP
  
  if((tripAButtonLatch == true)||(tripBButtonLatch == true)||(maintButtonLatch == true)||(histButtonLatch == true))
  {
    
    count = readFT6236TouchLocation( touchLocations, 2 );
    if(count == 0)
      buttonUpTimer += millis() - buttonUpStart;

    if(buttonUpTimer > BUTTON_UP_TIME)  // button released
      {
        tripAButtonLatch = false;
        tripBButtonLatch = false;
        maintButtonLatch = false;
        histButtonLatch = false;
        tripAButtonState = UP; 
        tripBButtonState = UP;
        maintButtonState = UP;
        histButtonState = UP;
        buttonUpTimer = 0;
        if(currentDisplayPage == History) DrawMaintButton();
      }
    
    
  }
  
  // If no buttons pressed, clear timers
  if (count == 0)  {
    if (tripAButtonState == UP) tripAResetTimer = 0;
    if (tripBButtonState == UP) tripBResetTimer = 0;
    if (maintButtonState == UP) maintResetTimer = 0;
    if (histButtonState == UP) progResetTimer = 0;
  }


  unsigned long now = micros();
  programTime = millis();

  // 2/5/24 GPS encoding was here. Moved to top of loop
  // GPS Query must be outside of time interrupt routines
  //while (Serial2.available() > 0)
    //gps->encode(Serial2.read());
    
    
  // Let's check for user commands
  if (g_command_ready) 
  {
    ProcessCommand(g_command);
  }

 //**********************************************************
//
//         One Second timer routines
//
//
//********************************************************* 
  // Every Second...
  if (secondFlag)  {
    // check for a start
    if (CheckRunning()){
      if (!running) {
        running = true;
        engine.Total.starts++;
      }
    }
    
    else {  // not running
      running = false;
      rpm = 0.;
      engine.Last.rpm = 0;
      engine.Last.fuelAmount = 0;
      engine.Last.pulseWidth = 0;
      motor_timer = 0;
      for (int i = 0; i < SPEEDARRAYSIZE; i++)
        speeds[i] = 0.;
    }

    // Update parameters
    UpdateSpeedRanges(engine.Last.rpm);

    GetFuelLevel();
    CalcGPH();

    // GPS routines
    GPSUpdate();

    // If GPS data is stale bump the kill timer.
    if((STALEFLAG == true)||(engine.Last.course > 1000)||(engine.Last.course < 0)) 
    {
      if((COURSESTALE==true) || (SPEEDSTALE==true)
         || (LATLONSTALE==true) || (gps->satellites.value() == 0))      
      staleTimer++;
      else {
        staleTimer = 0;
        STALEFLAG = false;
       }
        
      if(staleTimer >= MAXSTALETIME){ // stale too long. Kill the gps instance
//        Serial.println("***************GPS RENEWED**************");
//        delete gps;
//
//        // and reconstitute it.
//        TinyGPSPlus *gps = new TinyGPSPlus();
//        
//        staleTimer = 0;
//        STALEFLAG = false;

          // Let's restart the whole shebang
          // Set reset
          engine.RESTART = true;
          engine.lastPage = currentDisplayPage;
          RESETFLAG = true;
         
      }
    }

    engine.Last.fuelAmount = engine.Total.fuelUsed;
    engine.Last.fuelRemain = SmoothFuel();
    engine.Last.battVoltage = GetBattVoltage();
    if (REALTIMEENGINE == true)
    {
      Serial.print("Battery Voltage: ");
      Serial.println(engine.Last.battVoltage, 2);
    }
    
    // Display
    if(engine.screenStatus == ON)
    {
      switch (currentDisplayPage ) {
        case Instruments:
          UpdatePage1();
          break;
  
        case Navigation:
          UpdatePage2();
          break;
  
        case TripA:
          UpdatePage3();
          break;
  
        case TripB:
          UpdatePage4();
          break;
  
        case History:
          UpdatePage5();
          break;
      }
    }  //end if screen on

    else tft.fillScr(VGA_BLACK);

    // clear pulse count and fuel accumulator
    intervalPulseCount = 0;
    cycleFuel = 0;
    
    secondFlag = false;
  }   // end of every second

//**********************************************************
//
//         One Minute timer routines
//
//
//*********************************************************

  // Every Minute...
  if (minuteFlag)    {
    // Update parameters
    UpdateSpeedArray();

    // Update flash memory
    UpdateFlash();

    // Update trip data
    UpdateTrips();

    minuteFlag = false;
  }  // end of every minute


  if (pulseReceived)
  {
    int   speed;
    GetSpeedandFuel();
    speed = SmoothSpeeds();
    engine.Last.rpm = speed;
    pulseReceived = false;

  }


  // Need to restart the program
  if(RESETFLAG == true)
  {
  // Update flash memory
    UpdateFlash();
    resetFunc(); //call reset
  }


}  // End Main Loop
