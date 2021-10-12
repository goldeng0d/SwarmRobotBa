#include "Wheels.h"

Wheels::Wheels(double rate, double wheelDiameter, unsigned long countsPerRevolution, int enc1pin, int enc2pin){
    updateRate = rate;

    wheel1.pin = enc1pin;
    wheel1.cntPerRev = countsPerRevolution;
    wheel1.radius = ((wheelDiameter)/2.0)/1000.0; // diamter in mm to radius in m

    wheel2.pin = enc2pin;
    wheel2.cntPerRev = countsPerRevolution;
    wheel2.radius = ((wheelDiameter)/2.0)/1000.0; // diamter in mm to radius in m

}

Wheels::~Wheels(){
    
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

void Wheels::setup(){
  // set pins to output
  pinMode(wheel1.pin, INPUT_PULLUP);
  pinMode(wheel2.pin, INPUT_PULLUP);

  // attatch an interrupt detector to 2 pins threough the hardware abstraction matrix
  attachInterrupt(wheel1.pin, ISR_enc1, CHANGE);
  attachInterrupt(wheel2.pin, ISR_enc2, CHANGE);

}

void Wheels::update(){

  // measure elapsed time
  double dT = (millis() - previousMillis)/1000.0;
  if(dT > 1/updateRate){

    // update class member counter values
    wheel1.counter = counter1;
    wheel2.counter = counter2;

    // calculate w, v, distance
    calculateValues(wheel1, dT);
    calculateValues(wheel2, dT);

    previousMillis = millis();
  }

}


void Wheels::calculateValues(Wheel_struct &wheel, double dT){

    // counter * angle resolution * radius
    wheel.distance = wheel.counter * ( (2*3.14) / wheel.cntPerRev) * wheel.radius;

    double dPhi = (wheel.counter - wheel.lastCounter) * ( (2*3.14) / wheel.cntPerRev);

    // put in array and push by 1
    for(int i = mvAvg_length-1; i >= 1; i--){
        wheel.mvAvgOmega[i] = wheel.mvAvgOmega[i-1];
    }
    wheel.mvAvgOmega[0] = dPhi/dT;

    // calculate moving avg
    wheel.omega=0;
    for(int i=0; i<mvAvg_length; i++){
        wheel.omega += wheel.mvAvgOmega[i];
    }
    wheel.omega /= mvAvg_length;

    // tangential linear velocity
    wheel.vel = wheel.omega * wheel.radius;

    wheel.lastCounter = wheel.counter;

}

void Wheels::setCntDir(unsigned int wheelNum, bool forward){

  // this sensor cant detect rotation direction by itself
  if(wheelNum == 1){

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

void Wheels::printDistance(){

  Serial.print("dsit 1 ");
  Serial.print(wheel1.distance);
  Serial.print("   dsit 2 ");
  Serial.println(wheel2.distance);
}
