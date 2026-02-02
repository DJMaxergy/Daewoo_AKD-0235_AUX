/*
        
DAEWOO AKD-0235 to Arduino connections:
Arduino                LC72366 (µC)            LC75371 (EVC)
D11 (MOSI)             EVR-SO                  -
D10 (SS)               EVR-CE                  -
D13 (SCK)              EVR-CLK                 -
D3 (DO)                -                       DI
D4 (CE)                -                       CE
D5 (CL)                -                       CL
GND                    GND                     GND

*/

#include <inttypes.h>
#include <SPI.h>
#include <SanyoCCB.h>
#include <avr/sleep.h>

// LC75371 Address = 10000001 = 0x81 = 129
#define ADDR          129

// LC75371 Parameters, byte 0
#define PAR0_FAD      4
#define PAR0_BASS     0

// LC75371 Parameters, byte 1
#define PAR1_TREB     4
#define PAR1_1DB      0

// LC75371 Parameters, byte 2
#define PAR2_10DB     5
#define PAR2_TEST     4
#define PAR2_INPUT    2
#define PAR2_GAIN     0

// LC75371 Parameters, byte 3
#define PAR3_CHAN     6
#define PAR3_LOUD     5
#define PAR3_FAD_RF   4
#define PAR3_TEST     0

// transmission types
#define TRANS16       0
#define TRANS4        1

// Backlight LED Color control pins for AUX recognition (Green = Normal, Blue = AUX):
#define LED_COLOR_G   8
#define LED_COLOR_B   9

// Interrupt for Arduino to wake up when ACC is active:
#define INT_ACC       2

// Input modes
#define IN_TUNER      0
#define IN_CD         1
#define IN_AUX        2

// Gain levels
#define GAIN_0        0
#define GAIN_6        1
#define GAIN_12       2
#define GAIN_18       3

SanyoCCB ccb(3, 5, 6, 4); // Pins: DO CL DI CE

byte buff [16];
volatile byte indx;
volatile boolean process;

byte recData14[4];     //Right channel select / MUTE data / INPUT change data
byte recData24[4];     //Right channel select
byte recData34[4];     //Left channel select
byte recData44[4];     //Left channel select
byte recData14Prev[4]; //Right channel select / MUTE data / INPUT change data
byte recData24Prev[4]; //Right channel select
byte recData34Prev[4]; //Left channel select
byte recData44Prev[4]; //Left channel select
byte muteData[4];      //Generated data for muting during switching to AUX Input

volatile byte transmissionType; //normal 16 bytes transmission or 4 bytes only transmission

boolean loudnessOnlyActivated; //detection variable for loudness button pressed only
boolean auxInActive;  //AUX input toggle variable
byte inputMode;       //Input parameter storage: 0 = TUNER, 1 = CD, 2 = AUX

boolean awake;        //Indicator for woken up from sleep mode -> radio switched on

void subArray(byte srcArray[], byte subArray[], int newLength, int offset)
{
    for (int i = offset; i < (newLength + offset); i++) {
        subArray[i - offset] = srcArray[i];
    }
}

void changeInputParam(byte *data, byte input)
{
  bitWrite(data[2], PAR2_INPUT, (input >> 1) & 0x01);
  bitWrite(data[2], PAR2_INPUT + 1, input & 0x01);
  //AUX in:
  //bitWrite(data[2], PAR2_INPUT, 1);
  //bitWrite(data[2], PAR2_INPUT + 1, 0);
}

void changeGainParam(byte *data, byte gain)
{
  bitWrite(data[2], PAR2_GAIN, (gain >> 1) & 0x01);
  bitWrite(data[2], PAR2_GAIN + 1, gain & 0x01);
}

void enter_sleep(void)
{
  attachInterrupt(digitalPinToInterrupt(INT_ACC), INT_ACCisr, RISING);
  // go to sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable();
  awake = true;
}

/*******************\
 * Arduino setup() *
\*******************/ 
void setup() {
  //Serial.begin(115200);
  pinMode(LED_COLOR_G, OUTPUT);
  pinMode(LED_COLOR_B, OUTPUT);
  digitalWrite(LED_COLOR_G, HIGH);
  digitalWrite(LED_COLOR_B, LOW);

  pinMode(INT_ACC, INPUT);
  
  // CCB Init for sending data to EVC:
  ccb.init();

  // SPI Init for receiving data from µC:
  pinMode(MISO, OUTPUT); // have to send on master in so it set as output
  SPCR |= _BV(SPE); // turn on SPI in slave mode
  indx = 0; // buffer empty
  process = false;
  SPI.attachInterrupt(); // turn on interrupt
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);

  //Fill transmission data arrays with zeros:
  memset(recData14, 0, 4);
  memset(recData24, 0, 4);
  memset(recData34, 0, 4);
  memset(recData44, 0, 4);
  memset(recData14Prev, 0, 4);
  memset(recData24Prev, 0, 4);
  memset(recData34Prev, 0, 4);
  memset(recData44Prev, 0, 4);
  memset(muteData, 0, 4);

  transmissionType = TRANS16;
  loudnessOnlyActivated = true;
  auxInActive = false;
  inputMode = IN_TUNER;
  awake = false;
}

void INT_ACCisr(void)
{
  // Detach Interrupt, as it should occur only once:
  detachInterrupt(digitalPinToInterrupt(INT_ACC));
}

ISR (SPI_STC_vect) // SPI interrupt routine 
{ 
   byte c = SPDR; // read byte from SPI Data Register
   if (indx < sizeof buff) {
      buff[indx] = c; // save data in the next index in the array buff
      indx++;
      if (indx == sizeof buff) //normally 16 bytes transmission
      {
        transmissionType = TRANS16;
        process = true;
      }
      if (((c & 0xCF) == 0xC0) && (indx == 4)) //check for MUTE state -> only 4 bytes transmission
      {
        transmissionType = TRANS4;
        indx = sizeof buff;
        process = true;
      }
   }
}

/******************\
 * Arduino loop() *
\******************/ 
void loop() {
  if (digitalRead(INT_ACC) == LOW) {
    enter_sleep();
  }
  
  if (process) {
      process = false; //reset the process

      // Divide received data (buff) into 4 byte arrays, 
      // as most of the time 4 data frames will be received at once:
      subArray(buff, recData14, 4, 0);
      subArray(buff, recData24, 4, 4);
      subArray(buff, recData34, 4, 8);
      subArray(buff, recData44, 4, 12);

      // Print received data to serial output:
      int i = 0;
//      for (i=0; i < sizeof recData14; i++)
//      {
//        Serial.println (recData14[i],HEX);
//      }
//      if (transmissionType == TRANS16)
//      {
//        for (i=0; i < sizeof recData24; i++)
//        {
//          Serial.println (recData24[i],HEX);
//        }
//        for (i=0; i < sizeof recData34; i++)
//        {
//          Serial.println (recData34[i],HEX);
//        }
//        for (i=0; i < sizeof recData44; i++)
//        {
//          Serial.println (recData44[i],HEX);
//        }
//      }
//      Serial.println("--");

      if (awake) {
        awake = false;
        // TODO: initialization?
      }

      // Handle Loudness activation -> used for toggling Input 3 (AUX input):
      for (i=0; i < sizeof recData14; i++) {
        // Is third byte reached? -> contains info about Loudness On/Off
        if (i == 3) {
          // Is previously received data and actual data the same, except PAR3_LOUD?
          if ((recData14[i] & 0xDF) == (recData14Prev[i] & 0xDF)) {
            // Has Loudness been activated or not?
            if (bitRead(recData14[i], PAR3_LOUD) == 1) {
              loudnessOnlyActivated &= true;
            } else {
              loudnessOnlyActivated &= false;
            }
          } else {
            loudnessOnlyActivated &= false;
          }
        }
        // Is every other byte the same as previously received? -> precondition for Loudness only activation:
        else if ((recData14[i] == recData14Prev[i]) && (recData24[i] == recData24Prev[i]) && (recData34[i] == recData34Prev[i]) && (recData44[i] == recData44Prev[i])) {
          loudnessOnlyActivated &= true;
        }
        else {
          loudnessOnlyActivated &= false;
        }
      }

      // Toggle AUX Input:
      if (loudnessOnlyActivated == true) //LOUDNESS only is activated
      {
        //Serial.println("LOUDNESS only activated");
        auxInActive = !auxInActive; // Toggle

        // Create muteData from received data, needed for muting Output in case of switching to AUX and back:
        memcpy(muteData, recData14, sizeof(recData14));
        muteData[1] = muteData[1] & 0xF0;
        muteData[2] = muteData[2] & 0x1F;
        bitWrite(muteData[3], PAR3_CHAN, 1);
        bitWrite(muteData[3], PAR3_CHAN + 1, 1);
        
        // Send muting data with previous input mode:
        changeInputParam(muteData, inputMode);
        ccb.write(ADDR, muteData, 4);
        
        if (auxInActive) {
          inputMode = IN_AUX;
          digitalWrite(LED_COLOR_G, LOW);
          digitalWrite(LED_COLOR_B, HIGH);
          changeGainParam(muteData, GAIN_0); // Change Gain to 0 in AUX Mode
        } else {
          inputMode = (bitRead(recData14[2], PAR2_INPUT) << 1) + bitRead(recData14[2], PAR2_INPUT + 1);
          digitalWrite(LED_COLOR_G, HIGH);
          digitalWrite(LED_COLOR_B, LOW);
        }
        
        // Send muting data with toggled input mode:
        changeInputParam(muteData, inputMode);
        ccb.write(ADDR, muteData, 4);
      }

      // Store received data for next iteration here, before being manipulated by auxInActive:
      memcpy(recData14Prev, recData14, sizeof(recData14));
      memcpy(recData24Prev, recData24, sizeof(recData24));
      memcpy(recData34Prev, recData34, sizeof(recData34));
      memcpy(recData44Prev, recData44, sizeof(recData44));

      // Manipulate received data to switch to L3 Input (AUX In), only in case of AUX should be activated:
      if (auxInActive) {
        changeInputParam(recData14, IN_AUX);
        changeInputParam(recData24, IN_AUX);
        changeInputParam(recData34, IN_AUX);
        changeInputParam(recData44, IN_AUX);
        changeGainParam(recData14, GAIN_0);
        changeGainParam(recData24, GAIN_0);
        changeGainParam(recData34, GAIN_0);
        changeGainParam(recData44, GAIN_0);
      }

      // Final write of received/manipulated data to EVC:
      if (transmissionType == TRANS4) {
        ccb.write(ADDR, recData14, 4);
      } else {
        ccb.write(ADDR, recData14, 4);
        ccb.write(ADDR, recData24, 4);
        ccb.write(ADDR, recData34, 4);
        ccb.write(ADDR, recData44, 4);
      }

      // Reset variables and data arrays:
      indx= 0; //reset buffer index to zero
      memset(buff, 0, 16);
      memset(recData14, 0, 4);
      memset(recData24, 0, 4);
      memset(recData34, 0, 4);
      memset(recData44, 0, 4);
      loudnessOnlyActivated = true;
  }
}
  
