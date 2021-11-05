
#include "Drive.h"

// settings made here -> later create a configuration header with defines
myEncoder roboEncoder(50.0, 66.5, 12, {4, 5}, {35, 34});
//myEncoder roboEncoder(50.0, 66.5, 1200, {4, 5}, {34, 35});
L298N roboMotors(PWM_FREQUENCY, PWM_RESOLUTION, {17, 18, 16, 2}, {33, 32, 19, 1});
//L298N roboMotors(4000, 8, {17, 18, 16, 2}, {32, 33, 19, 1});

unsigned long lastMillisecafterBoot = 0;

Drive::Drive()
{
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

void Drive::setspeed(const unsigned int motoNum, const int velocity)
{

  // output motor speeds
  roboMotors.changeSpeed(MOTORLEFT, velocity);
  roboMotors.changeSpeed(MOTORRIGHT, velocity);
}

void IRAM_ATTR Drive::setDutyMotor(const unsigned int motoNum, const int duty)
{
  // output motor speed
  roboMotors.changeDuty(motoNum, duty);
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
    vl_error = v_l - roboEncoder.encoderLeft.vel;

  } else if(v_l < 0){
    //roboEncoder.setCntDir(1);
    roboMotors.changeDirection(MOTORLEFT, motDirection::BACKWARD);
    vl_error = roboEncoder.encoderLeft.vel - v_l;

  } else {
    roboMotors.changeDirection(MOTORLEFT, motDirection::BREAKING);
    vl_error = 0;
  }

  // left motor
  if(v_r > 0){
    //roboEncoder.setCntDir(2);
    roboMotors.changeDirection(MOTORRIGHT, motDirection::FORWARD);
    vr_error = v_r - roboEncoder.encoderLeft.vel;

  } else if(v_r < 0){
    //roboEncoder.setCntDir(2);
    roboMotors.changeDirection(MOTORRIGHT, motDirection::BACKWARD);
    vr_error = roboEncoder.encoderLeft.vel - v_r;
    
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

int32_t IRAM_ATTR Drive::getEncoderValueLEFT(void)
{
  return (roboEncoder.readAndResetLEFT());
}

int32_t IRAM_ATTR Drive::getEncoderValueRIGHT(void)
{
  return (roboEncoder.readAndResetRIGHT());
}

// void IRAM_ATTR Drive::rpmcontrol(int rpmVorgabe, int32_t &encoderValueleft, int32_t &encoderValueright)
// {

//    return;
// }

int32_t IRAM_ATTR Drive::IRegler(const int32_t Sollwertdrehzahl, const float dT, volatile int32_t Stellwert, const int32_t Drehzahlvalue, const int32_t resolution, const float lowfraction)
{
   // roboEncoder.myupdates();
   int32_t high = 0;
   int32_t Stellschritt = 0;
   // Although more comfortable DO NOT USE SWITCH CASE HERE OR THE PROGRAM CRASHES
   // This Look-up Table is a Lot faster than the power function.
   if (resolution == 8)
   {
      high = 255;
   }
   else if (resolution == 10)
   {
      high = 1023;
   }
   else if (resolution == 12)
   {
      high = 4095;
   }
   else if (resolution == 14)
   {
      high = 16383;
   }
   else if (resolution == 16)
   {
      high = 65535;
   }
   else if (resolution == 1) //very uncommon to use uneven resolutions but possible
   {
      high = 1;
   }
   else if (resolution == 2)
   {
      high = 3;
   }
   else if (resolution == 3)
   {
      high = 7;
   }
   else if (resolution == 4)
   {
      high = 15;
   }
   else if (resolution == 5)
   {
      high = 31;
   }
   else if (resolution == 6)
   {
      high = 63;
   }
   else if (resolution == 7)
   {
      high = 127;
   }
   else if (resolution == 9)
   {
      high = 511;
   }
   else if (resolution == 11)
   {
      high = 2047;
   }
   else if (resolution == 13)
   {
      high = 8191;
   }
   else if (resolution == 15)
   {
      high = 32767;
   }
   else{
      Serial.println("Error resolution not supported.");
   }

   Stellschritt = int32_t(high / 255);
   if (Sollwertdrehzahl > Drehzahlvalue && Stellwert + Stellschritt <= high)
   {
      if (Stellwert < high * lowfraction)
      {
         Stellwert = high * lowfraction;
      }
      Stellwert += Stellschritt;
   }
   // high * lowfraction to cap minimum Voltage, because if volatage to low on motor won't turn but 
   else if (Sollwertdrehzahl < Drehzahlvalue && Stellwert - Stellschritt > high * lowfraction) 
   {
      Stellwert -= Stellschritt;
   }
   return Stellwert;
}

// int Drive::mapInteger(int x, int in_min, int in_max, int out_min, int out_max)
// {
//    double result;
//    result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//    return (int)result;
// }

// void Drive::updateEncoderAndRPM(unsigned long dT){
//   roboEncoder.myupdates();
//   leftrpmValue = CalculateRPMfromEncoderValue(roboEncoder.encoderLeft.counter, dT);
//   rightrpmValue = CalculateRPMfromEncoderValue(roboEncoder.encoderRight.counter, dT);
//   return;
// }

// Calculate the RPM from the encoder value and the time difference
float IRAM_ATTR Drive::CalculateRPMfromEncoderValue(const int32_t encValue, const unsigned long dT)
{
   float ret = ((encValue / (float)ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE) / (dT / (float)MILLISEC_IN_SEC)) * SEC_IN_MIN;
   return ret;
}

// int8_t Drive::Zweipunktregler(int8_t Sollwert){
//   int8_t Stellwert;
//   if (Sollwert)
//   return Stellwert;
// }