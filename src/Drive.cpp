
#include "Drive.h"

// settings made here -> later create a configuration header with defines
myEncoder roboEncoder(50.0, 66.5, 1200, {4, 5}, {35, 34});
//myEncoder roboEncoder(50.0, 66.5, 1200, {4, 5}, {34, 35});
L298N roboMotors(4000, 8, {17, 18, 16, 2} , {33, 32, 19, 1});
//L298N roboMotors(4000, 8, {17, 18, 16, 2}, {32, 33, 19, 1});

Drive::Drive(){

}

Drive::~Drive(){

}

void Drive::setup(){
  roboEncoder.setup();
  roboMotors.setup();
}

float Drive::PIRegler(float error, float dt, float Kp, float Ki, float high, float low, float &iVal){

    float Tr = 0.2 * Kp / Ki;

    float Pval = Kp * error;
    float Vval = Pval + iVal;

    float Uval = Vval;

    if (Uval < low) Uval = low;
    if (Uval > high) Uval = high;

    iVal = iVal + dt * Ki * error + dt * (1 / Tr) * (Uval - Vval);

    return Uval;

}

void Drive::setspeed(const int velocity){
  
  // output motor speeds
  roboMotors.changeSpeed(MOTORLEFT, velocity);
  roboMotors.changeSpeed(MOTORRIGHT, velocity);
}

void Drive::setspeedMotorLEFT(const int velocity)
{
  // output motor speed
  roboMotors.changeSpeed(MOTORLEFT, velocity);
}

void Drive::setspeedMotorRIGHT(const int velocity)
{
  // output motor speed
  roboMotors.changeSpeed(MOTORRIGHT, velocity);
}

void Drive::move(const int direction){

  // set forward and backwardmovement or stop
  switch (direction)
  {

  case UP:
    roboMotors.changeDirection(MOTORLEFT, motDirection::FORWARD);
    roboMotors.changeDirection(MOTORRIGHT, motDirection::FORWARD);
    break;

  case DOWN:
    roboMotors.changeDirection(MOTORRIGHT, motDirection::BACKWARD);
    roboMotors.changeDirection(MOTORLEFT, motDirection::BACKWARD);
    break;

  case LEFT:
    roboMotors.changeDirection(MOTORRIGHT, motDirection::FORWARD);
    roboMotors.changeDirection(MOTORLEFT, motDirection::BACKWARD);
    break;

  case RIGHT:
    roboMotors.changeDirection(MOTORRIGHT, motDirection::BACKWARD);
    roboMotors.changeDirection(MOTORLEFT, motDirection::FORWARD);
    break;

  case STOP:
    roboMotors.changeDirection(MOTORRIGHT, motDirection::BREAKING);
    roboMotors.changeDirection(MOTORLEFT, motDirection::BREAKING);
    break;

  default:
    roboMotors.changeDirection(MOTORRIGHT, motDirection::BREAKING);
    roboMotors.changeDirection(MOTORLEFT, motDirection::BREAKING);
    break;
  }
  
}

void Drive::update(double dT, const double vel, const double omega){

    // update encoder
  roboEncoder.myupdate();

  // calculate singular wheel speeds
  double v_r = (2 * vel + omega * WHEEL_DISTANCE_L) / 2;
  double v_l = (2 * vel - omega * WHEEL_DISTANCE_L) / 2;

  double vl_error;
  double vr_error;

  // handle pos and neg rotation seperatly
  // left motor
  if(v_l > 0){
    //roboEncoder.setCntDir(1);
    roboMotors.changeDirection(MOTORLEFT, motDirection::FORWARD);
    vl_error = v_l - roboEncoder.encoder1.vel;

  } else if(v_l < 0){
    //roboEncoder.setCntDir(1);
    roboMotors.changeDirection(MOTORLEFT, motDirection::BACKWARD);
    vl_error = roboEncoder.encoder1.vel - v_l;

  } else {
    roboMotors.changeDirection(MOTORLEFT, motDirection::BREAKING);
    vl_error = 0;
  }

  // left motor
  if(v_r > 0){
    //roboEncoder.setCntDir(2);
    roboMotors.changeDirection(MOTORRIGHT, motDirection::FORWARD);
    vr_error = v_r - roboEncoder.encoder1.vel;

  } else if(v_r < 0){
    //roboEncoder.setCntDir(2);
    roboMotors.changeDirection(MOTORRIGHT, motDirection::BACKWARD);
    vr_error = roboEncoder.encoder1.vel - v_r;
    
  } else {
    roboMotors.changeDirection(MOTORRIGHT, motDirection::BREAKING);
    vr_error = 0;
  }

  // PI - Wheel-speed-controller
  double outR = PIRegler(vr_error, dT, 200, 50, 100.0, 45, iVal_rightWheel);
  double outL = PIRegler(vl_error, dT, 200, 50, 100.0, 45, iVal_leftWheel);

  
  #ifdef DEBUG_WHEEL_CONTROLLER_LEFT
    Serial.print(" v_l: ");
    Serial.print(roboEncoder.wheel1.vel);
    Serial.print(" v_l_soll: ");
    Serial.print(v_l);
    Serial.print(" error: ");
    Serial.print((v_l - roboEncoder.wheel1.vel));
    Serial.print(" out: ");
    Serial.println(outL);
  #endif
  

  // output motor speeds
  roboMotors.changeSpeed(MOTORLEFT, outL);
  roboMotors.changeSpeed(MOTORRIGHT, outR);

}

int32_t Drive::getEncoderValueLEFT(void)
{
  int32_t value = roboEncoder.readAndResetLEFT();
  Serial.printf("readdriveleft = %d\n", value);
  return (value);
}

int32_t Drive::getEncoderValueRIGHT(void)
{
  int32_t value = roboEncoder.readAndResetRIGHT();
  Serial.printf("readdriveright = %d\n", value);
  return (value);
}

void Drive::rpmcontrol(int rpm){
  int32_t leftEncoderValue = roboEncoder.readAndResetLEFT();
  int32_t rightEncoderValue = roboEncoder.readAndResetRIGHT();
  int32_t difference = leftEncoderValue - rightEncoderValue;
  if (difference > 2){
    setspeedMotorRIGHT(+2);
  }
  else if (difference < -2){  
    setspeedMotorLEFT(-2);
  }
  else{

  }
  return;

}