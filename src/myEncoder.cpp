#include "myEncoder.h"

#define ENCODER_ARGLIST_SIZE 4+1

static Encoder_internal_state_t *interruptArgs[ENCODER_ARGLIST_SIZE];
void update(Encoder_internal_state_t *arg);

myEncoder::myEncoder(double rate, double wheelDiameter, unsigned long countsPerRevolution, myEncoder_pins enc1pin, myEncoder_pins enc2pin)
{
  updateRate = rate;

  encoder1.enc.encA = enc1pin.encA;
  encoder1.enc.encB = enc1pin.encB;
  encoder1.cntPerRev = countsPerRevolution;
  encoder1.radius = ((wheelDiameter) / 2.0) / 1000.0; // diamter in mm to radius in m

  encoder2.enc.encA = enc2pin.encA;
  encoder2.enc.encB = enc2pin.encB;
  encoder2.cntPerRev = countsPerRevolution;
  encoder2.radius = ((wheelDiameter) / 2.0) / 1000.0; // diamter in mm to radius in m
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
  pinMode(encoder1.enc.encA, INPUT_PULLUP);
  pinMode(encoder1.enc.encB, INPUT_PULLUP);
  pinMode(encoder2.enc.encA, INPUT_PULLUP);
  pinMode(encoder2.enc.encB, INPUT_PULLUP);


  libencoder1.pin1_register = (portInputRegister(digitalPinToPort(encoder1.enc.encA)));
  libencoder1.pin1_bitmask = (digitalPinToBitMask(encoder1.enc.encA));
  libencoder1.pin2_register = (portInputRegister(digitalPinToPort(encoder1.enc.encB)));
  libencoder1.pin2_bitmask = (digitalPinToBitMask(encoder1.enc.encB));
  libencoder1.position = 0;

  libencoder2.pin1_register = (portInputRegister(digitalPinToPort(encoder2.enc.encA)));
  libencoder2.pin1_bitmask = (digitalPinToBitMask(encoder2.enc.encA));
  libencoder2.pin2_register = (portInputRegister(digitalPinToPort(encoder2.enc.encB)));
  libencoder2.pin2_bitmask = (digitalPinToBitMask(encoder2.enc.encB));
  libencoder2.position = 0;

  // allow time for a passive R-C filter to charge
  // through the pullup resistors, before reading
  // the initial state
  delayMicroseconds(2000);
  
  uint8_t statelibencoder1 = 0;
  if ((((*(libencoder1.pin1_register)) & (libencoder1.pin1_bitmask)) ? 1 : 0)) statelibencoder1 |= 1;
  if ((((*(libencoder1.pin2_register)) & (libencoder1.pin2_bitmask)) ? 1 : 0)) statelibencoder1 |= 2;
  libencoder1.state = statelibencoder1;

  uint8_t statelibencoder2 = 0;
  if ((((*(libencoder2.pin1_register)) & (libencoder2.pin1_bitmask)) ? 1 : 0)) statelibencoder1 |= 1;
  if ((((*(libencoder2.pin2_register)) & (libencoder2.pin2_bitmask)) ? 1 : 0)) statelibencoder1 |= 2;
  libencoder2.state = statelibencoder2;

  // attatch an interrupt detector to 2 pins threough the hardware abstraction matrix
  interruptArgs[1] = &libencoder1;
  attachInterrupt(encoder1.enc.encA, ISR_enc1, CHANGE);
  interruptArgs[2] = &libencoder1;
  attachInterrupt(encoder1.enc.encB, ISR_enc2, CHANGE);
  interruptArgs[3] = &libencoder2;
  attachInterrupt(encoder2.enc.encA, ISR_enc3, CHANGE);
  interruptArgs[4] = &libencoder2;
  attachInterrupt(encoder2.enc.encB, ISR_enc4, CHANGE);
  encoder1.counter = 0;
  //enclib1.write(0);
  encoder2.counter = 0;
  //enclib2.write(0);
}

void myEncoder::myupdate(){

  // measure elapsed time
  double dT = (millis() - previousMillis)/1000.0;
  if(dT > 1/updateRate){

    // update class member counter values
    encoder1.counter = readAndResetRIGHT();
    encoder2.counter = readAndResetLEFT();

    // calculate w, v, distance
    calculateValues(encoder1, dT);
    calculateValues(encoder2, dT);

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

//     if (encoder1.counter > 0){
//       cntDir1 = 1;
//     }
//     else if (encoder1.counter == 0)
//     {
//       cntDir1 = 0;
//     }
//     else
//     {
//       cntDir1 = -1;
//     }
//   } else {

//     if (encoder2.counter > 0)
//     {
//       cntDir2 = 1;
//     }
//     else if (encoder2.counter == 0)
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
  Serial.print(encoder1.distance);
  Serial.print("\n   dsit 2 ");
  Serial.println(encoder2.distance);
}

inline int32_t myEncoder::readRIGHT()
{
  noInterrupts();
  update(&libencoder1);
  int32_t ret = libencoder1.position;
  interrupts();
  return ret;
}

inline int32_t myEncoder::readLEFT()
{
  noInterrupts();
  update(&libencoder2);
  int32_t ret = libencoder2.position;
  interrupts();
  return ret;
}

int32_t myEncoder::readAndResetRIGHT(){
  noInterrupts();
  update(&libencoder1);
  int32_t ret = libencoder1.position;
  libencoder1.position = 0;
  interrupts();
  return ret;
}

int32_t myEncoder::readAndResetLEFT()
{
  noInterrupts();
  update(&libencoder2);
  int32_t ret = libencoder2.position;
  libencoder2.position = 0;
  interrupts();
  return ret;
}

inline void myEncoder::writeRIGHT(int32_t p){
  noInterrupts();
  libencoder1.position = p;
  interrupts();
}

inline void myEncoder::writeLEFT(int32_t p)
{
  noInterrupts();
  libencoder2.position = p;
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
  switch (state)
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
  }
}