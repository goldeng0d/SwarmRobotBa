
#include "Drive.h"

// settings made here -> later create a configuration header with defines
myEncoder roboEncoder(50.0, 66.5, 12, {4, 5}, {35, 34});
//myEncoder roboEncoder(50.0, 66.5, 1200, {4, 5}, {34, 35});
L298N roboMotors(4000, 8, {17, 18, 16, 2} , {33, 32, 19, 1});
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

void Drive::setDutyMotor(const unsigned int motoNum, const int duty)
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

int32_t Drive::getEncoderValueLEFT(void)
{
  return (roboEncoder.readAndResetLEFT());
}

int32_t Drive::getEncoderValueRIGHT(void)
{
  return (roboEncoder.readAndResetRIGHT());
}

void Drive::rpmcontrol(int rpmVorgabe){
   int differenzleft = 0;
   int differenzright = 0;
   int absolutdifferenzleft = 0;
   int absolutdifferenzright = 0;
   volatile uint32_t rpmvorgaberightold = rpmVorgabe;
   volatile uint32_t rpmvorgabeleftold = rpmVorgabe;
   double dT = 0;
   double percent = 0;
   int32_t leftEncoderValue = 0;
   int32_t rightEncoderValue = 0;
   int32_t leftrpmValue = 0;
   int32_t rightrpmValue = 0;
   int32_t differenzSchrittlinks = 0;
   int32_t differenzSchrittrechts = 0;
   uint8_t newDutycycle = 0;
   while (1)
   {
      leftEncoderValue = roboEncoder.readAndResetLEFT();
      rightEncoderValue = roboEncoder.readAndResetRIGHT();
      dT = (millis() - lastMillisecafterBoot) / 1000.0;
      lastMillisecafterBoot = millis();
      //volatile unsigned long ganzezeit = (dT * 1000);
      leftrpmValue = ((leftEncoderValue / ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE) / dT) * SEC_IN_MIN;
      if (leftrpmValue < 0){
         leftrpmValue = -leftrpmValue;
      }
      rightrpmValue = ((rightEncoderValue / ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE) / dT ) * SEC_IN_MIN;
      if (rightrpmValue < 0)
      {
         rightrpmValue = -rightrpmValue;
      }
      differenzleft = rpmVorgabe - leftrpmValue;
      differenzright = rpmVorgabe - rightrpmValue;

      differenzSchrittlinks = 100;
      differenzSchrittrechts = 100;

      // set Stepsize towards goal for left Motor
      // if (differenzleft < 0)
      // {
      //    absolutdifferenzleft = -differenzleft;
      // }
      // if (absolutdifferenzleft > 8000)
      // {
      //    differenzSchrittlinks = 4000;
      // }
      // else if (absolutdifferenzleft <= 8000 && absolutdifferenzleft > 2000)
      // {
      //    differenzSchrittlinks = 2000;
      // }
      // else if (absolutdifferenzleft <= 2000 && absolutdifferenzleft > 500)
      // {
      //    differenzSchrittlinks = 500;
      // }
      // else if (absolutdifferenzleft <= 500)
      // {
      //    differenzSchrittlinks = 100;
      // }
      // // set Stepsize towards goal for right Motor
      // if(differenzright < 0)
      // {
      //    absolutdifferenzright = -differenzright;
      // }
      // if (absolutdifferenzright > 8000)
      // {
      //    differenzSchrittrechts = 4000;
      // }
      // else if (absolutdifferenzright <= 8000 && absolutdifferenzright > 2000)
      // {
      //    differenzSchrittrechts = 2000;
      // }
      // else if (absolutdifferenzright <= 2000 && absolutdifferenzright > 500)
      // {
      //    differenzSchrittrechts = 500;
      // }
      // else if (absolutdifferenzright <= 500)
      // {
      //    differenzSchrittrechts = 100;
      // }

      // Serial.printf("Zeit in ms = %f \n", dT * 1000);
      // Serial.printf("Differenz left  = %d \n", differenzleft);
      // Serial.printf("Differenz Right = %d \n", differenzright);
      // Serial.printf("RPM Value vorgabe  = %d \n", rpmVorgabe);
      // Serial.printf("RPM Value Calc Left  = %d \n", leftrpmValue);
      // Serial.printf("RPM Value Calc Right = %d \n", rightrpmValue);
      //int32_t encoderValueDifference = leftEncoderValue - rightEncoderValue;
      newDutycycle = 0;
      if(rightrpmValue == 0)
      {
         rightrpmValue++;
      }
      percent = (leftrpmValue / rightrpmValue) * 100;

      roboMotors.changeSpeed(MOTORRIGHT, percent);

      if (percent < 5)
      {
         break;
      }

      //double newrpmTarget;
      // if (leftrpmValue < rpmVorgabe || rightrpmValue < rpmVorgabe)
      // {
      //    if (differenzleft > 0)
      //    {
      //       newDutycycle = mapInteger(rpmvorgabeleftold, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       // Serial.printf("rpmvorgaberightold more Left  = %d \n", rpmvorgabeleftold);
      //       // Serial.printf("newDutycycle vor more Left  = %d \n", newDutycycle);
      //       roboMotors.changeDuty(MOTORLEFT,newDutycycle);
      //       rpmvorgabeleftold += differenzSchrittlinks;
      //       // Serial.printf("rpmvorgaberightold nach more Left  = %d \n", rpmvorgaberightold);
      //    }

      //    if (differenzright > 0)
      //    {
      //       newDutycycle = mapInteger(rpmvorgaberightold, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       // Serial.printf("rpmvorgaberightold more Right  = %d \n", rpmvorgaberightold);
      //       Serial.printf("newDutycycle vor more Right  = %d \n", newDutycycle);
      //       roboMotors.changeDuty(MOTORRIGHT, newDutycycle);
      //       rpmvorgaberightold += differenzSchrittrechts;
      //       // Serial.printf("rpmvorgaberightold nach more Right  = %d \n", rpmvorgaberightold);
      //    }
      // }
      // if (leftrpmValue > rpmVorgabe || rightrpmValue > rpmVorgabe)
      // {
      //    if (differenzleft < 0)
      //    {
      //       newDutycycle = mapInteger(rpmvorgabeleftold, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       // Serial.printf("rpmvorgaberightold less Left  = %d \n", rpmvorgabeleftold);
      //       // Serial.printf("newDutycycle less Left  = %d \n", newDutycycle);
      //       //roboMotors.changeDuty(MOTORLEFT, newDutycycle);
      //       rpmvorgabeleftold -= differenzSchrittlinks;
      //       // Serial.printf("rpmvorgaberightold nach less Left  = %d \n", rpmvorgaberightold);
      //    }

      //    if (differenzright < 0)
      //    {
      //       newDutycycle = mapInteger(rpmvorgaberightold, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       // Serial.printf("rpmvorgaberightold vor less Right  = %d \n", rpmvorgaberightold);
      //        Serial.printf("newDutycycle less Right  = %d \n", newDutycycle);
      //       //roboMotors.changeDuty(MOTORRIGHT, newDutycycle);
      //       rpmvorgaberightold -= differenzSchrittrechts;
      //       // Serial.printf("rpmvorgaberightold nach less Right  = %d \n", rpmvorgaberightold);
      //    }
      // }

      // if ((differenzleft <= 1000 && differenzleft >= -1000) && (differenzright <= 1000 && differenzright >= -1000))
      // {
      //    break;
      // }

      // if (leftrpmValue || rightrpmValue < rpmVorgabe)
      // {
      //    if (rpmVorgabe - leftrpmValue > RPM_MAX * RAMP_FRACTION_SLOW)
      //    {
      //       newrpmTarget = leftrpmValue + schleifendurchlaeufe * (RPM_MAX * RAMP_FRACTION_SLOW);
      //       Serial.printf("newrpmTarget more MOTORLEFT  = %f \n", newrpmTarget);

      //       newDutycycle = mapInteger(newrpmTarget, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       roboMotors.changeDuty(MOTORLEFT,newDutycycle);
      //    }
      //    else{
      //       newrpmTarget = rpmVorgabe;
      //       Serial.printf("newrpmTarget more MOTORLEFT  = %f \n", newrpmTarget);

      //       newDutycycle = mapInteger(newrpmTarget, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       roboMotors.changeDuty(MOTORLEFT, newDutycycle);
      //    }

      //    if (rpmVorgabe - rightrpmValue > RPM_MAX * RAMP_FRACTION_SLOW)
      //    {
      //       newrpmTarget = rightrpmValue + schleifendurchlaeufe * (RPM_MAX * RAMP_FRACTION_SLOW);
      //       Serial.printf("newrpmTarget more MOTORRIGHT  = %f \n", newrpmTarget);
      //       newDutycycle = mapInteger(newrpmTarget, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       roboMotors.changeDuty(MOTORRIGHT, newDutycycle);
      //    }
      //    else
      //    {
      //       newrpmTarget = rpmVorgabe;
      //       Serial.printf("newrpmTarget more MOTORRIGHT  = %f \n", newrpmTarget);

      //       newDutycycle = mapInteger(newrpmTarget, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       roboMotors.changeDuty(MOTORLEFT, newDutycycle);
      //    }

      //    if (rpmVorgabe - leftrpmValue < RPM_MAX * RAMP_FRACTION_FAST)
      //    {
      //       newrpmTarget = leftrpmValue + (schleifendurchlaeufe - 3) * (RPM_MAX * RAMP_FRACTION_SLOW);
      //       Serial.printf("newrpmTarget more MOTORLEFT  = %f \n", newrpmTarget);
      //       newDutycycle = mapInteger(newrpmTarget, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       roboMotors.changeDuty(MOTORLEFT, newDutycycle);
      //    }
      //    else
      //    {
      //       newrpmTarget = rpmVorgabe;
      //       Serial.printf("newrpmTarget more MOTORLEFT  = %f \n", newrpmTarget);

      //       newDutycycle = mapInteger(newrpmTarget, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       roboMotors.changeDuty(MOTORLEFT, newDutycycle);
      //    }

      //    if (rpmVorgabe - rightrpmValue < RPM_MAX * RAMP_FRACTION_FAST)
      //    {
      //       newrpmTarget = rightrpmValue + (schleifendurchlaeufe - 3) * (RPM_MAX * RAMP_FRACTION_SLOW);
      //       Serial.printf("newrpmTarget more MOTORRIGHT  = %f \n", newrpmTarget);
      //       newDutycycle = mapInteger(newrpmTarget, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       roboMotors.changeDuty(MOTORRIGHT, newDutycycle);
      //    }
      //    else
      //    {
      //       newrpmTarget = rpmVorgabe;
      //       Serial.printf("newrpmTarget more MOTORRIGHT  = %f \n", newrpmTarget);

      //       newDutycycle = mapInteger(newrpmTarget, 0, RPM_MAX, 80, pow(2, roboMotors.getResolution()) - 1);
      //       roboMotors.changeDuty(MOTORLEFT, newDutycycle);
      //    }
      // }
      //schleifendurchlaeufe++;
      // loop until same speed
   
      // leftEncoderValue = roboEncoder.readAndResetLEFT();
      // rightEncoderValue = roboEncoder.readAndResetRIGHT();
      // encoderValueDifference = leftEncoderValue - rightEncoderValue;

      // if(encoderValueDifference <= 3)
      // {
      //    break;
      // }
   }

   return;

}

int Drive::mapInteger(int x, int in_min, int in_max, int out_min, int out_max)
{
   double result;
   result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
   return (int)result;
}

void Drive::updateEncoderAndRPM(unsigned long dT){
  roboEncoder.myupdates();
  leftrpmValue = CalculateRPMfromEncoderValue(roboEncoder.encoderLeft.counter, dT);
  rightrpmValue = CalculateRPMfromEncoderValue(roboEncoder.encoderRight.counter, dT);
  return;
}

int32_t Drive::CalculateRPMfromEncoderValue(int32_t encValue, unsigned long dT)
{
  return (((encValue / ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE) / (dT / MILLISEC_IN_SEC)) * SEC_IN_MIN);
}

// int8_t Drive::Zweipunktregler(int8_t Sollwert){
//   int8_t Stellwert;
//   if (Sollwert)
//   return Stellwert;
// }