
#ifndef drive_h__
#define drive_h__


#include <Arduino.h>
#include "myEncoder.h"
#include "L298N.h"


#define WHEEL_DISTANCE_L 0.140 // 140mm // our BARobot has ~110mm
#define ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE 12
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 4000
#define MOTOR_LOWFRACTION 0.25
#define MILLISEC_IN_SEC 1000
#define SEC_IN_MIN 60
#define RAMP_FRACTION_SLOW (1/5)
#define RAMP_FRACTION_FAST (1/15)
#define RPM_MAX 30000
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define STOP 0
#define MOTORLEFT 1
#define MOTORRIGHT 2

// #define DEBUG_WHEEL_CONTROLLER_LEFT


class Drive{

    public:
      // Constructor has no actions in it
      Drive();
      // Destructor has no actions in it
      ~Drive();

      /**
       * setup encoder
       * setop motor driver
      */
      void setup();

      /**
       * setup encoder
       * setop motor driver
       * @param[in] dT time elapsed since function was called the last time
       * @param[in] vel Angular Velocity which should be driven
       * @param[in] omega Linear Velocity which should be driven
      */
      void update(double dT, const double vel, const double omega);

      void move(const int direction);
      void setspeed(const int velocity);
      void setspeed(const unsigned int motoNum, const int velocity);
      void setDutyMotor(const unsigned int motoNum, const int duty);
      int32_t IRAM_ATTR getEncoderValueLEFT(void);
      int32_t IRAM_ATTR getEncoderValueRIGHT(void);
      void rpmcontrol(int rpmVorgabe);
      void updateEncoderAndRPM(unsigned long dT);
      int32_t CalculateRPMfromEncoderValue(int32_t encValue, unsigned long dT);

      /**
      * PI-Controller with mit control signal limitation and anti-windup
      * 
      * @param[in] Sollwertdrehzahl Error which should become 0
      * @param[in] dt time elapsed since controller was called the last time
      * @param[in] Stellwert Proportional factor
      * @param[in] Drehzahlvalue Integral Factor
      * @param[in] high maximum output limit
      * @param[in] low minimum input limit
      */
      int32_t IRAM_ATTR IRegler(const int32_t Sollwertdrehzahl, const float dT, int32_t Stellwert, const int32_t Drehzahlvalue, const int32_t resolution, const float lowfraction);

   private:

      // global variables which needs to be saved globally
      float iVal_rightWheel;
      float iVal_leftWheel;
      volatile int32_t Stellwertrechts;
      volatile int32_t Stellwertlinks;
      volatile int32_t rightrpmValue;
      volatile int32_t leftrpmValue;

      /**
      * PI-Controller with mit control signal limitation and anti-windup
      * 
      * @param[in] error Error which should become 0
      * @param[in] dt time elapsed since controller was called the last time
      * @param[in] Kp Proportional factor
      * @param[in] Ki Integral Factor
      * @param[in] high maximum output limit
      * @param[in] low minimum input limit
      * @param[out] iVal global variable which needs to be saved globally
      */
      float PIRegler(float error, float dt, float Kp, float Ki, float high, float low, float &iVal);

      int mapInteger(int x, int in_min, int in_max, int out_min, int out_max);
};

#endif