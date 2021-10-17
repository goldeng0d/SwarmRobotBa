
#ifndef drive_h__
#define drive_h__


#include <Arduino.h>
#include "Encoder.h"
#include "L298N.h"


#define WHEEL_DISTANCE_L 0.140 // 140mm
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

          private :

          // global variables which needs to be saved globally
          float iVal_rightWheel;
      float iVal_leftWheel;

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

};

#endif