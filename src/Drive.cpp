
#include "Drive.h"

// settings made here -> later create a configuration header with defines
myEncoder roboEncoder(50.0, 66.5, 12, {4, 5}, {35, 34});
L298N roboMotors(PWM_FREQUENCY, PWM_RESOLUTION, {17, 18, 16, 1}, {33, 32, 19, 2});

unsigned long lastMillisecafterBoot = 0;

Drive::Drive()
{
   
   // Although more comfortable DO NOT USE SWITCH CASE HERE OR THE PROGRAM CRASHES
   // This Look-up Table is a lot faster than the power function.
   if(PWM_RESOLUTION == 8)
   {
      maxPWMvalue = 255; // 2^8 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 10)
   {
      maxPWMvalue = 1023; // 2^10 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 12)
   {
      maxPWMvalue = 4095; // 2^12 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 14)
   {
      maxPWMvalue = 16383; // 2^14 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 16)
   {
      maxPWMvalue = 65535; // 2^16 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 1) //very uncommon to use uneven PWM_RESOLUTIONs but technically possible
   {
      maxPWMvalue = 1; // 2^1 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 2)
   {
      maxPWMvalue = 3; // 2^2 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 3)
   {
      maxPWMvalue = 7; // 2^3 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 4)
   {
      maxPWMvalue = 15; // 2^4 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 5)
   {
      maxPWMvalue = 31; // 2^5 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 6)
   {
      maxPWMvalue = 63; // 2^6 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 7)
   {
      maxPWMvalue = 127; // 2^7 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 9)
   {
      maxPWMvalue = 511; // 2^9 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 11)
   {
      maxPWMvalue = 2047; // 2^11 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 13)
   {
      maxPWMvalue = 8191; // 2^13 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else if (PWM_RESOLUTION == 15)
   {
      maxPWMvalue = 32767; // 2^15 - 1
      Adjustingstep = uint32_t(maxPWMvalue / 255); // set to fixed Adjustingstep for fast adjustinig
   }
   else
   {
      Serial.println("Error PWM_RESOLUTION not supported.");
   }
   minPWMvaluestartturn = (int32_t((float)maxPWMvalue * MOTOR_LOWFRACTION_STARTTURN));
   minPWMvalueturning = (int32_t((float)maxPWMvalue * MOTOR_LOWFRACTION_TURNING));
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

int32_t IRAM_ATTR Drive::IRegler(const uint32_t Sollwertdrehzahl, volatile uint32_t Stellwert, const uint32_t Drehzahlvalue)
{

   // maxPWMvalue = highest PWM value
   // minPWMvalueturning = lowest PWM value while turning

   // Increase Output DutyCycle if Sollwertdrehzahl > Drehzahlvalue but,
   // maxPWMvalue to cap maximum Voltage, because PWM has a limit where DutyCycle = 100%
   if (Sollwertdrehzahl > Drehzahlvalue && Stellwert + Adjustingstep <= maxPWMvalue)
   {
      Stellwert += Adjustingstep;
   }
   // Lower Output DutyCycle if Sollwertdrehzahl < Drehzahlvalue but,
   // minPWMvalueturning to cap minimum Voltage, because if volatage to low on motor won't turn
   else if (Sollwertdrehzahl < Drehzahlvalue && Stellwert - Adjustingstep > minPWMvalueturning)
   {
      Stellwert -= Adjustingstep;
   }
   return Stellwert;
}

int32_t IRAM_ATTR Drive::mapInteger(const float x, const float in_min, const float in_max, const float out_min, const float out_max)
{
   float result;
   result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
   return (int32_t)result;
}

// Calculate the RPM from the encoder value and the time difference
float IRAM_ATTR Drive::CalculateRPMfromEncoderValue(const int32_t encValue, const float dT)
{
   float ret = (((float)encValue / (float)ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE) / (dT / (float)MILLISEC_IN_SEC)) * (float)SEC_IN_MIN;
   return ret;
}
