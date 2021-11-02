#include "myEncoder.h"

#define ENCODER_ARGLIST_SIZE 4+1

static Encoder_internal_state_t *interruptArgs[ENCODER_ARGLIST_SIZE];
void IRAM_ATTR update(Encoder_internal_state_t *arg);

myEncoder::myEncoder(double rate, double wheelDiameter, unsigned long countsPerRevolution, myEncoder_pins enc1pin, myEncoder_pins enc2pin)
{
  updateRate = rate;

  encoderLeft.enc.encA = enc1pin.encA;
  encoderLeft.enc.encB = enc1pin.encB;
  encoderLeft.cntPerRev = countsPerRevolution;
  encoderLeft.radius = ((wheelDiameter) / 2.0) / 1000.0; // diamter in mm to radius in m

  encoderRight.enc.encA = enc2pin.encA;
  encoderRight.enc.encB = enc2pin.encB;
  encoderRight.cntPerRev = countsPerRevolution;
  encoderRight.radius = ((wheelDiameter) / 2.0) / 1000.0; // diamter in mm to radius in m
}

myEncoder::~myEncoder(){
    
}

// interrupt service routine is volatile and can not call members of a class
// volatile long counter1;
// volatile int cntDir1 = 1;
static void IRAM_ATTR ISR_enc1()
{
  //counter1 += cntDir1;
  update(interruptArgs[1]);
}

// interrupt service routine is volatile and can not call members of a class
// volatile long counter2;
// volatile int cntDir2 = 1;
static void IRAM_ATTR ISR_enc2()
{
  //counter2 += cntDir2;
  update(interruptArgs[2]);
}

// interrupt service routine is volatile and can not call members of a class
// volatile long counter3;
// int cntDir3 = 1;
static void IRAM_ATTR ISR_enc3()
{
  //counter3 += cntDir3;
  update(interruptArgs[3]);
}

// interrupt service routine is volatile and can not call members of a class
// volatile long counter4;
// int cntDir4 = 1;
static void IRAM_ATTR ISR_enc4()
{
  //counter4 += cntDir4;
  update(interruptArgs[4]);
}

void myEncoder::setup(){
  // set pins to output
  pinMode(encoderLeft.enc.encA, INPUT_PULLUP);
  pinMode(encoderLeft.enc.encB, INPUT_PULLUP);
  pinMode(encoderRight.enc.encA, INPUT_PULLUP);
  pinMode(encoderRight.enc.encB, INPUT_PULLUP);


  libencoderLeft.pin1_register = (portInputRegister(digitalPinToPort(encoderLeft.enc.encA)));
  libencoderLeft.pin1_bitmask = (digitalPinToBitMask(encoderLeft.enc.encA));
  libencoderLeft.pin2_register = (portInputRegister(digitalPinToPort(encoderLeft.enc.encB)));
  libencoderLeft.pin2_bitmask = (digitalPinToBitMask(encoderLeft.enc.encB));
  libencoderLeft.position = 0;

  libencoderRight.pin1_register = (portInputRegister(digitalPinToPort(encoderRight.enc.encA)));
  libencoderRight.pin1_bitmask = (digitalPinToBitMask(encoderRight.enc.encA));
  libencoderRight.pin2_register = (portInputRegister(digitalPinToPort(encoderRight.enc.encB)));
  libencoderRight.pin2_bitmask = (digitalPinToBitMask(encoderRight.enc.encB));
  libencoderRight.position = 0;

  // allow time for a passive R-C filter to charge
  // through the pullup resistors, before reading
  // the initial state
  delayMicroseconds(2000);
  
  uint8_t statelibencoder1 = 0;
  if ((((*(libencoderLeft.pin1_register)) & (libencoderLeft.pin1_bitmask)) ? 1 : 0)) statelibencoder1 |= 1;
  if ((((*(libencoderLeft.pin2_register)) & (libencoderLeft.pin2_bitmask)) ? 1 : 0)) statelibencoder1 |= 2;
  libencoderLeft.state = statelibencoder1;

  uint8_t statelibencoder2 = 0;
  if ((((*(libencoderRight.pin1_register)) & (libencoderRight.pin1_bitmask)) ? 1 : 0)) statelibencoder1 |= 1;
  if ((((*(libencoderRight.pin2_register)) & (libencoderRight.pin2_bitmask)) ? 1 : 0)) statelibencoder1 |= 2;
  libencoderRight.state = statelibencoder2;

  // attatch an interrupt detector to 2 pins threough the hardware abstraction matrix
  interruptArgs[1] = &libencoderLeft;
  attachInterrupt(encoderLeft.enc.encA, ISR_enc1, CHANGE);
  interruptArgs[2] = &libencoderLeft;
  attachInterrupt(encoderLeft.enc.encB, ISR_enc2, CHANGE);
  interruptArgs[3] = &libencoderRight;
  attachInterrupt(encoderRight.enc.encA, ISR_enc3, CHANGE);
  interruptArgs[4] = &libencoderRight;
  attachInterrupt(encoderRight.enc.encB, ISR_enc4, CHANGE);
  encoderLeft.counter = 0;
  //enclib1.write(0);
  encoderRight.counter = 0;
  //enclib2.write(0);
}

void myEncoder::myupdate(){

  // measure elapsed time
  double dT = (millis() - previousMillis)/1000.0;
  if(dT > 1/updateRate){

    // update class member counter values
    encoderLeft.counter = readAndResetLEFT();
    encoderRight.counter = readAndResetRIGHT();

    // calculate w, v, distance
    calculateValues(encoderLeft, dT);
    calculateValues(encoderRight, dT);

    previousMillis = millis();
  }

}

void myEncoder::myupdates()
{

  // measure elapsed time
  double dT = (millis() - previousMillis) / 1000.0;
  if (dT > 1 / updateRate)
  {

    // update class member counter values
    encoderLeft.counter = readAndResetRIGHT();
    encoderRight.counter = readAndResetLEFT();

    // calculate w, v, distance
    calculateValues(encoderLeft, dT);
    calculateValues(encoderRight, dT);

    previousMillis = millis();
  }
}

void myEncoder::calculateValues(myEncoder_struct &encoder, double dT){

    // counter * angle resolution * radius
    encoder.distance = encoder.counter * ( (2*3.14) / encoder.cntPerRev) * encoder.radius;

    double dPhi = (encoder.counter - encoder.lastCounter) * ( (2*3.14) / encoder.cntPerRev);

    // put in array and push by 1
    for(int i = mvAvg_length-1; i >= 1; i--){
        encoder.mvAvgOmega[i] = encoder.mvAvgOmega[i-1];
    }
    encoder.mvAvgOmega[0] = dPhi/dT;

    // calculate moving avg
    encoder.omega=0;
    for(int i=0; i<mvAvg_length; i++){
        encoder.omega += encoder.mvAvgOmega[i];
    }
    encoder.omega /= mvAvg_length;

    // tangential linear velocity
    encoder.vel = encoder.omega * encoder.radius;

    encoder.lastCounter = encoder.counter;

}

// this is only usefull if you cannot determin the direction from encoders
// void myEncoder::setCntDir(unsigned int encoderNum){

//   // this sensor can detect rotation direction by itself
//   if(encoderNum == 1){

//     if (encoderLeft.counter > 0){
//       cntDir1 = 1;
//     }
//     else if (encoderLeft.counter == 0)
//     {
//       cntDir1 = 0;
//     }
//     else
//     {
//       cntDir1 = -1;
//     }
//   } else {

//     if (encoderRight.counter > 0)
//     {
//       cntDir2 = 1;
//     }
//     else if (encoderRight.counter == 0)
//     {
//       cntDir2 = 0;
//     }
//     else
//     {
//       cntDir2 = -1;
//     }
//   }

// }

void myEncoder::printDistance(){

  Serial.print("\ndsit 1 ");
  Serial.print(encoderLeft.distance);
  Serial.print("\n   dsit 2 ");
  Serial.println(encoderRight.distance);
}

inline int32_t myEncoder::readRIGHT()
{
  noInterrupts();
  update(&libencoderLeft);
  int32_t ret = libencoderLeft.position;
  interrupts();
  return ret;
}

inline int32_t myEncoder::readLEFT()
{
  noInterrupts();
  update(&libencoderRight);
  int32_t ret = libencoderRight.position;
  interrupts();
  return ret;
}

int32_t myEncoder::readAndResetRIGHT()
{
  noInterrupts();
  update(&libencoderRight);
  int32_t ret = libencoderLeft.position;
  libencoderLeft.position = 0;
  interrupts();
  return ret;
}

int32_t myEncoder::readAndResetLEFT()
{
  noInterrupts();
  update(&libencoderLeft);
  int32_t ret = libencoderRight.position;
  libencoderRight.position = 0;
  interrupts();
  return ret;
}

inline void myEncoder::writeRIGHT(int32_t p){
  noInterrupts();
  libencoderLeft.position = p;
  interrupts();
}

inline void myEncoder::writeLEFT(int32_t p)
{
  noInterrupts();
  libencoderRight.position = p;
  interrupts();
}

//                           _______         _______
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2

//	new	new	old	old
//	pin2	pin1	pin2	pin1	Result
//	----	----	----	----	------
//	0	0	0	0	no movement
//	0	0	0	1	+1
//	0	0	1	0	-1
//	0	0	1	1	+2  (assume pin1 edges only)
//	0	1	0	0	-1
//	0	1	0	1	no movement
//	0	1	1	0	-2  (assume pin1 edges only)
//	0	1	1	1	+1
//	1	0	0	0	+1
//	1	0	0	1	-2  (assume pin1 edges only)
//	1	0	1	0	no movement
//	1	0	1	1	-1
//	1	1	0	0	+2  (assume pin1 edges only)
//	1	1	0	1	-1
//	1	1	1	0	+1
//	1	1	1	1	no movement
void update(Encoder_internal_state_t *arg)
{
  uint8_t p1val = (((*(arg->pin1_register)) & (arg->pin1_bitmask)) ? 1 : 0);
  uint8_t p2val = (((*(arg->pin2_register)) & (arg->pin2_bitmask)) ? 1 : 0);
  uint8_t state = arg->state & 3;
  if (p1val)
    state |= 4;
  if (p2val)
    state |= 8;
  arg->state = (state >> 2);
  // Do **not** use switch-case statement here 
  // The compiler emits a jump-table which is somehow (?!) still located
  // in flash and thus can't be used in an ISR since everything has to be in IRAM or DRAM.
  // this is actually bug-worthy to report.
  // the code below does the same and does not crash.
  if(state == 1 || state == 7 || state == 8 || state == 14) {
      arg->position++;
  } else if(state == 2 || state == 4 || state == 11 || state == 13){
      arg->position--;
  } else if(state == 3 || state == 12) {
      arg->position += 2;
  } else if(state == 6 || state == 9) {
      arg->position -= 2;
  }
  /*switch (state)
  {
    case 1: case 7: case 8: case 14:
      arg->position++;
      //Serial.printf("position = %d\n", arg->position);
      return;
    case 2: case 4: case 11: case 13:
      arg->position--;
      //Serial.printf("position = %d\n", arg->position);
      return;
    case 3: case 12:
      arg->position += 2;
      //Serial.printf("position = %d\n", arg->position);
      return;
    case 6: case 9:
      arg->position -= 2;
      //Serial.printf("position = %d\n", arg->position);
      return;
  }*/
}