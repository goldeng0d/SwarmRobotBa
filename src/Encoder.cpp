#include "Encoder.h"

Encoder::Encoder(double rate, double wheelDiameter, unsigned long countsPerRevolution, Encoder_pins enc1pin, Encoder_pins enc2pin)
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

Encoder::~Encoder(){
    
}

// interrupt service routine is volatile and can not call members of a class
long counter1;
int cntDir1 = 1;
void IRAM_ATTR ISR_enc1() {
    counter1 += cntDir1;
}

// interrupt service routine is volatile and can not call members of a class
long counter2;
int cntDir2 = 1;
void IRAM_ATTR ISR_enc2() {
    counter2 += cntDir2;
}

// interrupt service routine is volatile and can not call members of a class
long counter3;
int cntDir3 = 1;
void IRAM_ATTR ISR_enc3()
{
  counter3 += cntDir3;
}

// interrupt service routine is volatile and can not call members of a class
long counter4;
int cntDir4 = 1;
void IRAM_ATTR ISR_enc4()
{
  counter4 += cntDir4;
}

void Encoder::setup(){
  // set pins to output
  pinMode(encoder1.enc.encA, INPUT_PULLUP);
  pinMode(encoder1.enc.encB, INPUT_PULLUP);
  pinMode(encoder2.enc.encA, INPUT_PULLUP);
  pinMode(encoder2.enc.encB, INPUT_PULLUP);

  // attatch an interrupt detector to 2 pins threough the hardware abstraction matrix
  attachInterrupt(encoder1.enc.encA, ISR_enc1, CHANGE);
  attachInterrupt(encoder1.enc.encB, ISR_enc2, CHANGE);
  attachInterrupt(encoder2.enc.encA, ISR_enc3, CHANGE);
  attachInterrupt(encoder2.enc.encB, ISR_enc4, CHANGE);
}

void Encoder::update(){

  // measure elapsed time
  double dT = (millis() - previousMillis)/1000.0;
  if(dT > 1/updateRate){

    // update class member counter values
    encoder1.counter = counter1;
    encoder2.counter = counter2;

    // calculate w, v, distance
    calculateValues(encoder1, dT);
    calculateValues(encoder2, dT);

    previousMillis = millis();
  }

}


void Encoder::calculateValues(Encoder_struct &encoder, double dT){

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

void Encoder::setCntDir(unsigned int encoderNum, bool forward){

  // this sensor cant detect rotation direction by itself
  if(encoderNum == 1){

    if(forward){
      cntDir1 = 1;
    }else{
      cntDir1 = -1;
    }

  } else {

    if(forward){
      cntDir2 = 1;
    }else{
      cntDir2 = -1;
    }

  }

}

void Encoder::printDistance(){

  Serial.print("dsit 1 ");
  Serial.print(encoder1.distance);
  Serial.print("   dsit 2 ");
  Serial.println(encoder2.distance);
}
