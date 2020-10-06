//datasheet at https://github.com/Atlantis-Specialist-Technologies/CAN485/blob/master/Documentation/Datasheet%20AT90CANXX.pdf
#include <ASTCanLib.h>
#include <math.h>
#include <AutoCAN.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <TimeLib.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <EEPROM.h>

#define DEBUG_PROD true
#define DEBUG_MPH false
#define DEBUG_CAN false
#define DEBUG_VSS false
#define DEBUG_DAC false
#define DEBUG_GPS false
#define DEBUG_ACCEL false
#define DEBUG_COMPASS false

void(* reset) (void) = 0;

// Pins used on board //////////////////////////////////////////////////////////

//pins 2-4 used for I2C
byte const TIMEZONE_PIN = 5;              //each button press cycles through different time zone offsets
byte const VSS_PIN = 9;                   //pin 9 on the board corresponds to interrupt 7 on the chip

// Variables for timing ////////////////////////////////////////////////////////

byte const GPS_CALC_INTERVAL = 100;       //how long to wait between asking GPS for data
byte const ACCEL_INTERVAL = 50;           //time between reading accelerometer
byte const SPEED_CALC_INTERVAL = 125;     //read number of pulses approx every 1/8 second
byte const MPH_BUFFER_LENGTH = 4;         //length of MPH buffer
byte const COMPASS_INTERVAL = 200;        //how long to wait between compass checks
uint16_t const PROD_INTERVAL = 1000;      //prod debug message interval

unsigned long currentMillis = 0;          //now
unsigned long lastMphMillis = 0;          //used to cut time into slices of SPEED_CALC_INTERVAL
unsigned long lastGpsMillis = 0;          //used with GPS_CALC_INTERVAL
unsigned long lastAccelMillis = 0;        //used with ACCEL_INTERVAL
unsigned long lastCompassMillis = 0;      //used with COMPASS_INTERVAL
unsigned long lastProdMillis = 0;         //used with PROD_INTERVAL

// Variables for calculating MPH ///////////////////////////////////////////////

volatile unsigned long vssCounter = 0;    //increment pulses in the interrupt function
unsigned long vssCounterSafe = 0;         //vss pulses that won't be corrupted by an interrupt
unsigned long vssCounterPrevious = 0;     //used to calculate speed
float mphBuffer[MPH_BUFFER_LENGTH];       //keep buffer of mph readings (approx .5 second)
byte mphBufferIndex = 0;
float mph = 0.0;

// Generic CAN bus variables ///////////////////////////////////////////////////

uint8_t canBuffer[8] = {};

#define MESSAGE_PROTOCOL  0     // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8     // Data length: 8 bytes
#define MESSAGE_RTR       0     // rtr bit

volatile unsigned long canCount = 0;
volatile unsigned long canUnhandledCount = 0;

volatile st_cmd_t canMsg;

typedef struct {
  int16_t id;
  unsigned long counter;
  uint8_t* data;
} canData;

// Variables for MegaSquirt CAN messages ///////////////////////////////////////

volatile canData* allCanMessages[5];
volatile canData canBase;
volatile canData canPlus1;
volatile canData canPlus2;
volatile canData canPlus3;
volatile canData canPlus4;
uint8_t canBufferBase[8] = {};
uint8_t canBufferPlus1[8] = {};
uint8_t canBufferPlus2[8] = {};
uint8_t canBufferPlus3[8] = {};
uint8_t canBufferPlus4[8] = {};
volatile canData canTemp;
uint8_t canBufferTemp[8] = {};

// CAN objects for sending messages ////////////////////////////////////////////

st_cmd_t txMsg;
uint8_t txBuffer[8] = {0,0,0,0,0,0,0,0};

// DAC variables ///////////////////////////////////////////////////////////////

Adafruit_MCP4728 dac;
uint16_t analogAfrOutput = 0;

// GPS variables ///////////////////////////////////////////////////////////////

SFE_UBLOX_GPS gps;
int8_t timeZoneOffset[5] = {-4, -5, -6, -7, -8};
uint8_t timeZoneIndex = 1; // change this value with a button, load/save from eeprom
const uint8_t eepromAddressTimezone = 0;
datetime gpsDatetime;
time_t previousTime = 0; // when the digital clock was displayed
long longitude = 0;
long latitude = 0;
long altitude = 0;

// Button press variables //////////////////////////////////////////////////////

bool currentButtonValue = 1;
bool previousButtonValue = 1;
unsigned long buttonMillis = 0;
const byte DEBOUNCE_DELAY = 250;

// Accelerometer & compass variables ///////////////////////////////////////////

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(11223);
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(22334);
float compassHeading = 0.0;
char* compassDirection = "";

// END GLOBAL VARIABLES ////////////////////////////////////////////////////////

//interrupt routine for interrupt 7 (pin 9) - vss sensor
ISR(INT7_vect) {
  vssCounter++;
}

//interrupt routine for receiving CAN bus messages
ISR(CANIT_vect) {
  canCount++;

  unsigned i;   
  char save_canpage=CANPAGE;   
  
  unsigned mob = CANHPMOB; // get highest prio mob   
  CANPAGE = mob & 0xf0;   
  mob >>= 4; // -> mob number 0..15   
  //ASSERT( (CANSTMOB & ~0xa0) ==0); // allow only RX ready and DLC warning   
    
  canTemp.id = (CANIDT2>>5) | (CANIDT1 <<3);
  
  register char length; 
  length = CANCDMOB & 0x0f;
  for (i = 0; i <length; ++i)   
  {
    canTemp.data[i] = CANMSG;
  }   
  
  CANSTMOB = 0;           // reset INT reason   
  CANCDMOB = 0x80;        // re-enable RX on this channel   
  CANPAGE = save_canpage; // restore CANPAGE   

  if(true) 
  {
    switch(canTemp.id)
    {
      case MS_BASE_ID:
        allCanMessages[MSG_MS_BASE]->counter++;
        fillCanDataBuffer(MSG_MS_BASE, &canTemp);
        break;
      case MS_BASE_ID + 1:
        allCanMessages[MSG_MS_PLUS1]->counter++;
        fillCanDataBuffer(MSG_MS_PLUS1, &canTemp);
        break;
      case MS_BASE_ID + 2:
        allCanMessages[MSG_MS_PLUS2]->counter++;
        fillCanDataBuffer(MSG_MS_PLUS2, &canTemp);
        break;
      case MS_BASE_ID + 3:
        allCanMessages[MSG_MS_PLUS3]->counter++;
        fillCanDataBuffer(MSG_MS_PLUS3, &canTemp);
        break;
      case MS_BASE_ID + 4:
        allCanMessages[MSG_MS_PLUS4]->counter++;
        fillCanDataBuffer(MSG_MS_PLUS4, &canTemp);
        break;
      case SH_BASE_ID:
        vssCounter++;
        break;
      default:
        canUnhandledCount++;
        break;
    }
  }
}

void fillCanDataBuffer(int index, canData* canTemp)
{
  for(int i = 0; i < 8; i++)
  {
    allCanMessages[index]->data[i] = canTemp->data[i];
  }
}

void setup() {
  canInit(500000);
  Serial.begin(1000000);
  pinMode(TIMEZONE_PIN, INPUT_PULLUP);
  pinMode(VSS_PIN, INPUT_PULLUP);
  
  //set trigger for interrupt 7 (pin 9) to be falling edge (see datasheet)
  EICRB |= ( 1 << ISC71);
  EICRB |= ( 0 << ISC70);

  //enable interrupt 7 (pin 9) (see datasheet)
  EIMSK |= ( 1 << INT7);
  
  timeZoneIndex = EEPROM.read(eepromAddressTimezone);

  Wire.begin();
  delay(200); //give time for gps to wake up to prevent program hangs
  if (gps.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("GPS not found."));
    reset();
  }

  gps.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  gps.setNavigationFrequency(20);
  gps.saveConfiguration();

  if (!dac.begin()) 
  {
    Serial.println("MCP4728 (DAC) not found.");
    delay(1000);
    reset();
  }

  if (!accel.begin()) 
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("LSM303 (accelerometer) not found.");
    delay(1000);
    reset();
  }

  accel.setRange(LSM303_RANGE_2G);
  accel.setMode(LSM303_MODE_HIGH_RESOLUTION);

  if (!mag.begin()) 
  {
    Serial.println("LSM303 (compass) not found.");
    delay(1000);
    reset();
  }

  #pragma region setup can bus

  txMsg.pt_data = &txBuffer[0];      // reference message data to transmit buffer

  clearBuffer(&canBufferTemp[0]);
  canTemp.data = &canBufferTemp[0];

  canBase.id = MS_BASE_ID;
  canBase.counter = 0;
  clearBuffer(&canBufferBase[0]);
  canBase.data = &canBufferBase[0];
  allCanMessages[0] = &canBase;

  canPlus1.id = MS_BASE_ID + 1;
  canPlus1.counter = 0;
  clearBuffer(&canBufferPlus1[0]);
  canPlus1.data = &canBufferPlus1[0];
  allCanMessages[1] = &canPlus1;
  
  canPlus2.id = MS_BASE_ID + 2;
  canPlus2.counter = 0;
  clearBuffer(&canBufferPlus2[0]);
  canPlus2.data = &canBufferPlus2[0];
  allCanMessages[2] = &canPlus2;
  
  canPlus3.id = MS_BASE_ID + 3;
  canPlus3.counter = 0;
  clearBuffer(&canBufferPlus3[0]);
  canPlus3.data = &canBufferPlus3[0];
  allCanMessages[3] = &canPlus3;

  canPlus4.id = MS_BASE_ID + 4;
  canPlus4.counter = 0;
  clearBuffer(&canBufferPlus4[0]);
  canPlus4.data = &canBufferPlus4[0];
  allCanMessages[4] = &canPlus4;

  CANSTMOB |= (1 << RXOK);
  CANGIE |= (1 << ENRX);

  CANIE1 |= (1 << IEMOB14);
  CANIE1 |= (1 << IEMOB13);
  CANIE1 |= (1 << IEMOB12);
  CANIE1 |= (1 << IEMOB11);
  CANIE1 |= (1 << IEMOB10);
  CANIE1 |= (1 << IEMOB9);
  CANIE1 |= (1 << IEMOB8);

  CANIE2 |= (1 << IEMOB7);
  CANIE2 |= (1 << IEMOB6);
  CANIE2 |= (1 << IEMOB5);
  CANIE2 |= (1 << IEMOB4);
  CANIE2 |= (1 << IEMOB3);
  CANIE2 |= (1 << IEMOB2);
  CANIE2 |= (1 << IEMOB1);
  CANIE2 |= (1 << IEMOB0);

  CANGIE |= (1 << ENIT);

  clearBuffer(&canBuffer[0]);
  canMsg.cmd      = CMD_RX_DATA;
  canMsg.pt_data  = &canBuffer[0];
  canMsg.ctrl.ide = MESSAGE_PROTOCOL; 
  canMsg.id.std   = 0;
  canMsg.id.ext   = 0;
  canMsg.dlc      = MESSAGE_LENGTH;
  canMsg.ctrl.rtr = MESSAGE_RTR;

  while(can_cmd(&canMsg) != CAN_CMD_ACCEPTED);

  #pragma endregion

  Serial.println("Finished initialization");
}

void loop() {
  currentMillis = millis();

  //////////////////////////////////////////////////////////////////////////////
  noInterrupts();
  processCanMessages(); //take remote data from CAN bus interrupt and put it in variables
  interrupts();
  //////////////////////////////////////////////////////////////////////////////

  outputAFR(engine_afr.currentValue);

  previousButtonValue = currentButtonValue;
  currentButtonValue = digitalRead(TIMEZONE_PIN);
  if(currentButtonValue != previousButtonValue) 
  {
    if(currentButtonValue == 0) 
    {
      buttonMillis = currentMillis;
      Serial.println("BUTTON PRESSED======================");
      timeZoneIndex++;
      if(timeZoneIndex > 4)
      {
        timeZoneIndex = 0;
      }
      EEPROM.write(eepromAddressTimezone, timeZoneIndex);
    }
    else 
    {
       if((currentMillis - buttonMillis) < DEBOUNCE_DELAY) 
       {
         currentButtonValue = 0;
       }
    }
  }

  if(currentMillis - lastGpsMillis >= GPS_CALC_INTERVAL && currentMillis > 500)
  {
    getGpsData();
    sendGpsDatetimeToCan();
    lastGpsMillis = currentMillis;
  }

  if(currentMillis - lastMphMillis >= SPEED_CALC_INTERVAL && currentMillis > 500)
  {
    mph = calculateSpeed();
    sendVssToCan(mph); //mph
    lastMphMillis = currentMillis;
  }

  if(currentMillis - lastAccelMillis >= ACCEL_INTERVAL && currentMillis > 500)
  {
    sensors_event_t event;
    accel.getEvent(&event);

    accelX = event.acceleration.x;
    accelY = event.acceleration.y;
    accelZ = event.acceleration.z;

    if(DEBUG_ACCEL)
    {
      /* Display the results (acceleration is measured in m/s^2) */
      Serial.print("X: ");
      Serial.print(accelX);
      Serial.print("  ");
      Serial.print("Y: ");
      Serial.print(accelY);
      Serial.print("  ");
      Serial.print("Z: ");
      Serial.print(accelZ);
      Serial.print("  ");
      Serial.println("m/s^2");
    }
    lastAccelMillis = currentMillis;
  }

  if(currentMillis - lastCompassMillis >= COMPASS_INTERVAL && currentMillis > 500)
  {
    sensors_event_t event;
    mag.getEvent(&event);

    float Pi = 3.14159;

    // Calculate the angle of the vector y,x
    compassHeading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

    // Normalize to 0-360
    if (compassHeading < 0) {
      compassHeading = 360 + compassHeading;
    }

    if(DEBUG_COMPASS)
    {
      Serial.print("Compass Heading: ");
      Serial.print(compassHeading);
      Serial.print(" (");
    }

    compassDirection = getCompassDirection(compassHeading);
    
    if(DEBUG_COMPASS)
    {
      Serial.print(compassDirection);
      Serial.println(")");
    }

    lastCompassMillis = currentMillis;
  }

  
  if(DEBUG_PROD && currentMillis - lastProdMillis >= PROD_INTERVAL && currentMillis > 500)
  {
    char formattedTime[9];
    sprintf(formattedTime, "%02d:%02d:%02d", gpsDatetime.hour, gpsDatetime.minute, gpsDatetime.second);
    Serial.print(formattedTime);

    char formattedDate[11];
    sprintf(formattedDate, "%02d/%02d/%04d", gpsDatetime.month, gpsDatetime.day, gpsDatetime.year);
    Serial.print(" ");
    Serial.println(formattedDate);

    Serial.print("Accel X: ");
    Serial.println(accelX);

    Serial.print("Accel Y: ");
    Serial.println(accelY);

    Serial.print("Accel Z: ");
    Serial.println(accelZ);

    Serial.print("MPH: ");
    Serial.println(mph);

    Serial.print("AFR analog output: ");
    Serial.println(analogAfrOutput);

    Serial.print("Longitude: ");
    Serial.println(longitude);

    Serial.print("Latitude: ");
    Serial.println(latitude);

    Serial.print("Altitude: ");
    Serial.println(altitude);





    Serial.println("====================");
    lastProdMillis = currentMillis;
  }

}

char* getCompassDirection(float heading)
{
    char* headingText;

    if(heading >= 330.0 || heading <= 30.0)
    {
      headingText = "N";
    }
    else if(heading >= 30.1 && heading <= 59.9)
    {
      headingText = "NE";
    }
    else if(heading >= 60.0 && heading <= 120.0)
    {
      headingText = "E";
    }
    else if(heading >= 120.1 && heading <= 149.9)
    {
      headingText = "SE";
    }
    else if(heading >= 150.0 && heading <= 210.0)
    {
      headingText = "S";
    }
    else if(heading >= 210.1 && heading <= 239.9)
    {
      headingText = "SW";
    }
    else if(heading >= 240.0 && heading <= 300.0)
    {
      headingText = "W";
    }
    else if(heading >= 300.1 && heading <= 329.9)
    {
      headingText = "NW";
    }
    return headingText;
}

void getGpsData()
{
  if(true)
  {
    setTime(gps.getHour(), gps.getMinute(), gps.getSecond(), gps.getDay(), gps.getMonth(), gps.getYear());
    adjustTime(timeZoneOffset[timeZoneIndex] * SECS_PER_HOUR);


    if (timeStatus()!= timeNotSet) {
      if (now() != previousTime) { //update the display only if the time has changed
        previousTime = now();

        gpsDatetime.hour = hour();
        gpsDatetime.minute = minute();
        gpsDatetime.second = second();
        gpsDatetime.month = month();
        gpsDatetime.day = day();
        gpsDatetime.year = year();

      }
    }



    latitude = gps.getLatitude();
    longitude = gps.getLongitude();
    altitude = gps.getAltitude();
    byte SIV = gps.getSIV();

    if(DEBUG_GPS)
    {
      Serial.print(F("Lat: "));
      Serial.print(latitude);

      Serial.print(F(" Long: "));
      Serial.print(longitude);
      Serial.print(F(" (degrees * 10^-7)"));

      Serial.print(F(" Alt: "));
      Serial.print(altitude);
      Serial.print(F(" (mm)"));

      Serial.print(F(" SIV: "));
      Serial.print(SIV);

      Serial.println();

      Serial.print(hour());
      Serial.print(":");
      Serial.print(minute());
      Serial.print(":");
      Serial.print(second());
      Serial.print(" ");
      Serial.print(month());
      Serial.print(" ");
      Serial.print(day());
      Serial.print(" ");
      Serial.print(year()); 
      Serial.println();
    }
  }
}

void outputAFR(float afr)
{
  //Configured for a gauge that expects linear output with 0v = 10afr, 5v = 18afr

  uint8_t afrTimesTen = afr * 10;
  if (afrTimesTen > 180)
  {
    afrTimesTen = 180;
  }
  if(afrTimesTen < 100)
  {
    afrTimesTen = 100;
  }
  analogAfrOutput = map(afrTimesTen, 100, 180, 0, 4095);
  dac.setChannelValue(MCP4728_CHANNEL_A, analogAfrOutput);
  if(DEBUG_DAC)
  {
    Serial.println(analogAfrOutput);
  }
}

void processCanMessages()
{
  vssCounterSafe = vssCounter;  //need this because ISR function may need 2 cycles to write vssCounter (?)
  engine_afr.currentValue = (double)allCanMessages[MSG_MS_PLUS2]->data[1] / 10.0;
}

float calculateSpeed() {
  long pulses = vssCounter - vssCounterPrevious;
  vssCounterPrevious = vssCounter;
  float pulsesPerSecond = (float)pulses * ((float)1000 / ((float)currentMillis - (float)lastMphMillis));
  float pulsesPerMinute = pulsesPerSecond * 60.0;
  float pulsesPerHour = pulsesPerMinute * 60.0;
  float milesPerHour = pulsesPerHour / (float)VSS_PULSE_PER_MILE;
  if(mphBufferIndex >= MPH_BUFFER_LENGTH - 1) {
    mphBufferIndex = 0;
  }
  else {
    mphBufferIndex++;
  }
  mphBuffer[mphBufferIndex] = milesPerHour;
  float mphSum = 0.0;
  for(byte i = 0; i < MPH_BUFFER_LENGTH; i++) {
    mphSum += mphBuffer[i];
  }
  float smoothedMPH = mphSum / (float)MPH_BUFFER_LENGTH;
  if(DEBUG_MPH && smoothedMPH > 0.0) {
    Serial.print("MPH: ");
    Serial.println(smoothedMPH);
  }
  return smoothedMPH;
}

void sendGpsDatetimeToCan()
{
  clearBuffer(&txBuffer[0]);
  
  txBuffer[0] = gpsDatetime.hour;
  txBuffer[1] = gpsDatetime.minute;
  txBuffer[2] = gpsDatetime.second;
  txBuffer[3] = gpsDatetime.month;
  txBuffer[4] = gpsDatetime.day;

  union
  {
    uint16_t year;
    byte buf[2];
  } yearUnion;
  yearUnion.year = gpsDatetime.year;
  
  txBuffer[5] = yearUnion.buf[0];
  txBuffer[6] = yearUnion.buf[1];

  // Serial.print(txBuffer[0]);
  // Serial.print(" ");
  // Serial.print(txBuffer[1]);
  // Serial.print(" ");
  // Serial.print(txBuffer[2]);
  // Serial.print(" ");
  // Serial.print(txBuffer[3]);
  // Serial.print(" ");
  // Serial.print(txBuffer[4]);
  // Serial.print(" ");
  // Serial.print(txBuffer[5]);
  // Serial.print(" ");
  // Serial.print(txBuffer[6]);
  // Serial.print(" ");
  // Serial.println(txBuffer[7]);

  // Setup CAN packet.
  txMsg.ctrl.ide = MESSAGE_PROTOCOL;    // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
  txMsg.id.std   = CAN_SH_CLK_MSG_ID;   // Set message ID
  txMsg.dlc      = MESSAGE_LENGTH;      // Data length: 8 bytes
  txMsg.ctrl.rtr = MESSAGE_RTR;         // Set rtr bit
  txMsg.pt_data = &txBuffer[0];         // reference message data to transmit buffer

  // Send command to the CAN port controller
  txMsg.cmd = CMD_TX_DATA;       // send message
  // Wait for the command to be accepted by the controller
  while(can_cmd(&txMsg) != CAN_CMD_ACCEPTED);
  // Wait for command to finish executing
  while(can_get_status(&txMsg) == CAN_STATUS_NOT_COMPLETED);
}

void sendVssToCan(float mph) 
{
  //send vssCounter on the CAN bus to be interpreted as an odometer reading
  //send calculated speed on the CAN bus

  clearBuffer(&txBuffer[0]);

  uint16_t mphTimesTen = (uint16_t)(mph * 10.0);
  
  union
  {
    uint16_t mphTimesTen;
    byte buf[2];
  } mphUnion;

  mphUnion.mphTimesTen = mphTimesTen;
  txBuffer[0] = mphUnion.buf[0];
  txBuffer[1] = mphUnion.buf[1];
  
  union
  {
    long vss;
    byte buf[4];
  } vssUnion;

  vssUnion.vss = vssCounterSafe;

  txBuffer[2] = vssUnion.buf[0];
  txBuffer[3] = vssUnion.buf[1];
  txBuffer[4] = vssUnion.buf[2];
  txBuffer[5] = vssUnion.buf[3];

  if(DEBUG_VSS)
  {
    Serial.print("VSS Pulses: ");
    Serial.println(vssCounterSafe);
    Serial.print(txBuffer[0]);
    Serial.print(" ");
    Serial.print(txBuffer[1]);
    Serial.print(" | ");
    Serial.print(txBuffer[2]);
    Serial.print(" ");
    Serial.print(txBuffer[3]);
    Serial.print(" ");
    Serial.print(txBuffer[4]);
    Serial.print(" ");
    Serial.print(txBuffer[5]);
    Serial.print(" ");
    Serial.print(txBuffer[6]);
    Serial.print(" ");
    Serial.println(txBuffer[7]);
  }
  
  // Setup CAN packet.
  txMsg.ctrl.ide = MESSAGE_PROTOCOL;    // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
  txMsg.id.std   = CAN_SH_VSS_MSG_ID;   // Set message ID
  txMsg.dlc      = MESSAGE_LENGTH;      // Data length: 8 bytes
  txMsg.ctrl.rtr = MESSAGE_RTR;         // Set rtr bit
  txMsg.pt_data = &txBuffer[0];         // reference message data to transmit buffer

  // Send command to the CAN port controller
  txMsg.cmd = CMD_TX_DATA;       // send message
  // Wait for the command to be accepted by the controller
  while(can_cmd(&txMsg) != CAN_CMD_ACCEPTED);
  // Wait for command to finish executing
  while(can_get_status(&txMsg) == CAN_STATUS_NOT_COMPLETED);
}
