//datasheet at https://github.com/Atlantis-Specialist-Technologies/CAN485/blob/master/Documentation/Datasheet%20AT90CANXX.pdf
#include <ASTCanLib.h>
#include <math.h>
#include <AutoCAN.h>

#define DEBUG_MPH true
#define DEBUG_CAN true

//pins used on board
byte const VSS_PIN = 9;                                     //pin 9 on the board corresponds to interrupt 7 on the chip

//other constants
unsigned const int PULSES_PER_MILE = 8000;                  //typical early Ford speed sensor
byte const SPEED_CALC_INTERVAL = 125;                       //read number of pulses approx every 1/8 second
byte const BUFFER_LENGTH = 4;                               //length of MPH buffer

volatile unsigned long vssCounter = 0;                      //increment pulses in the interrupt function
unsigned long vssCounterPrevious = 0;                       //used to calculate speed
unsigned long currentMillis = 0;                            //now
unsigned long lastMillis = 0;                               //used to cut time into slices of SPEED_CALC_INTERVAL
float mphBuffer[BUFFER_LENGTH];                             //keep buffer of mph readings (approx .5 second)
byte mphBufferIndex = 0;

byte oldAssistValue = 0;
byte newAssistValue = 0;

//can bus variables

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

#define MS_BASE_ID    1512  // set this to match the MegaSquirt setting, default is 1512
#define MSG_MS_BASE   0     // array index of allCanMessages[] to find the base id
#define MSG_MS_PLUS1  1     // array index 
#define MSG_MS_PLUS2  2     // etc...
#define MSG_MS_PLUS3  3     // ...
#define MSG_MS_PLUS4  4     // last element of array

volatile canData* allCanMessages[5];  //array of all CAN messages we are interested in receiving

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

//interrupt routine for interrupt 7 (pin 9) - vss sensor
ISR(INT7_vect) {
  vssCounter++;
}

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
  pinMode(VSS_PIN, INPUT_PULLUP);
  
  //set trigger for interrupt 7 (pin 9) to be falling edge (see datasheet)
  EICRB |= ( 1 << ISC71);
  EICRB |= ( 0 << ISC70);

  //enable interrupt 7 (pin 9) (see datasheet)
  EIMSK |= ( 1 << INT7);
  

  #pragma region setup can bus

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

  if(DEBUG_CAN)
  {
    Serial.println("CAN bus initialized");
  }

  #pragma endregion




  Serial.println("Finished initialization");
}

void loop() {
  currentMillis = millis();
  
  //perform speed calculation on an interval of SPEED_CALC_INTERVAL
  if(currentMillis - lastMillis >= SPEED_CALC_INTERVAL && currentMillis > 500) {
    
    float mph = calculateSpeed();
    sendToCan(mph);

    if(DEBUG_CAN)
    {
      Serial.println(canCount);
    }
    
    lastMillis = currentMillis;
  }
}

float calculateSpeed() {
  long pulses = vssCounter - vssCounterPrevious;
  vssCounterPrevious = vssCounter;
  float pulsesPerSecond = (float)pulses * ((float)1000 / ((float)currentMillis - (float)lastMillis));
  float pulsesPerMinute = pulsesPerSecond * 60.0;
  float pulsesPerHour = pulsesPerMinute * 60.0;
  float milesPerHour = pulsesPerHour / (float)PULSES_PER_MILE;
  if(mphBufferIndex >= BUFFER_LENGTH - 1) {
    mphBufferIndex = 0;
  }
  else {
    mphBufferIndex++;
  }
  mphBuffer[mphBufferIndex] = milesPerHour;
  float mphSum = 0.0;
  for(byte i = 0; i < BUFFER_LENGTH; i++) {
    mphSum += mphBuffer[i];
  }
  float smoothedMPH = mphSum / (float)BUFFER_LENGTH;
  if(DEBUG_MPH) {
    Serial.print("MPH: ");
    Serial.println(smoothedMPH);
  }
  return smoothedMPH;
}

void sendToCan(float mph) {
  //send vssCounter on the CAN bus to be interpreted as an odometer reading
  //send calculated speed on the CAN bus
  //send steering mode to the CAN bus in case anyone needs to read the status

  
}
