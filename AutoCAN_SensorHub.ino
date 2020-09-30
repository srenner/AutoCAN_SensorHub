//datasheet at https://github.com/Atlantis-Specialist-Technologies/CAN485/blob/master/Documentation/Datasheet%20AT90CANXX.pdf
#include <ASTCanLib.h>
#include <math.h>
#include <AutoCAN.h>

#define DEBUG_MPH true
#define DEBUG_CAN false
#define DEBUG_VSS false

//pins used on board
byte const AFR_PIN = 5;                                     //analog (pwm) afr output for traditional gauge
byte const VSS_PIN = 9;                                     //pin 9 on the board corresponds to interrupt 7 on the chip

//other constants
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

//can bus variables ////////////////////////////////////////////////////////////

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

// can objects for sending messages
st_cmd_t txMsg;
uint8_t txBuffer[8] = {0,0,0,0,0,0,0,0};

unsigned long vssCanTest = 0;


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
      case SH_BASE_ID:
        vssCounter++;
        break;
      default:
        vssCanTest++;
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
  pinMode(AFR_PIN, OUTPUT);
  pinMode(VSS_PIN, INPUT_PULLUP);
  
  //set trigger for interrupt 7 (pin 9) to be falling edge (see datasheet)
  EICRB |= ( 1 << ISC71);
  EICRB |= ( 0 << ISC70);

  //enable interrupt 7 (pin 9) (see datasheet)
  EIMSK |= ( 1 << INT7);
  

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

  if(DEBUG_CAN)
  {
    Serial.println("CAN bus initialized");
  }

  #pragma endregion




  Serial.println("Finished initialization");
}

void loop() {
  currentMillis = millis();
  
  //Serial.println(vssCanTest);

  noInterrupts();
  processCanMessages();
  interrupts();

  //Serial.println(engine_afr.currentValue);

  outputAFR();

  //perform speed calculation on an interval of SPEED_CALC_INTERVAL
  if(currentMillis - lastMillis >= SPEED_CALC_INTERVAL && currentMillis > 500) {
    
    float mph = calculateSpeed();
    sendVssToCan(mph); //mph

    if(DEBUG_CAN)
    {
      Serial.println(canCount);
    }
    
    lastMillis = currentMillis;
  }
}

void outputAFR()
{
  //using a 14.7 Spartan2 wideband controller, which has a linear output
  //0v = 10afr, 5v = 20afr
  //many other controllers are the same

  //send AFR output to DAC over I2C

  //comment out analog output - pwm does not work with my gauge
  // uint8_t afrTimesTen = engine_afr.currentValue * 10;
  // if(afrTimesTen < 100)
  // {
  //   afrTimesTen = 100;
  // }
  // else if(afrTimesTen > 180)
  // {
  //   afrTimesTen = 180;  //use 18.0 as the max value here because many traditional gauges are 10-18
  // }
  // uint16_t analogOutputValue = 0;
  // analogOutputValue = map(afrTimesTen, 100, 180, 0, 255);
  // analogWrite(AFR_PIN, analogOutputValue);
  // Serial.println(analogOutputValue);
}

void processCanMessages()
{
    engine_afr.currentValue = (double)allCanMessages[MSG_MS_PLUS2]->data[1] / 10.0;
}

float calculateSpeed() {
  long pulses = vssCounter - vssCounterPrevious;
  vssCounterPrevious = vssCounter;
  float pulsesPerSecond = (float)pulses * ((float)1000 / ((float)currentMillis - (float)lastMillis));
  float pulsesPerMinute = pulsesPerSecond * 60.0;
  float pulsesPerHour = pulsesPerMinute * 60.0;
  float milesPerHour = pulsesPerHour / (float)VSS_PULSE_PER_MILE;
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
  if(DEBUG_MPH && smoothedMPH > 0.0) {
    Serial.print("MPH: ");
    Serial.println(smoothedMPH);
  }
  return smoothedMPH;
}

void sendVssToCan(float mph) {
  //send vssCounter on the CAN bus to be interpreted as an odometer reading
  //send calculated speed on the CAN bus
  //send steering mode to the CAN bus in case anyone needs to read the status

  //engine_vss.currentValue = (allCanMessages[MSG_MS_PLUS4]->data[0] * 256 + allCanMessages[MSG_MS_PLUS4]->data[1]) / 10.0;

  uint16_t mphTimesTen = (uint16_t)(mph * 10.0);
  byte byte1 = mphTimesTen / 256;
  byte byte2 = mphTimesTen % 256;

  if(DEBUG_VSS)
  {
    Serial.println(mphTimesTen);
    Serial.println(byte1);
    Serial.println(byte2);

    Serial.println("=====");
  }

  txBuffer[0] = byte1;
  txBuffer[1] = byte2;
  
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
  //Serial.println("command accepted?");
  // Wait for command to finish executing
  while(can_get_status(&txMsg) == CAN_STATUS_NOT_COMPLETED);
  //Serial.println("send complete?");
}
