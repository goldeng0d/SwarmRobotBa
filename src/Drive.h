
#ifndef drive_h__
#define drive_h__


#include <Arduino.h>
#include "myEncoder.h"
#include "L298N.h"


#define WHEEL_DISTANCE_L 0.140 // 140mm // our BARobot has ~110mm
#define ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE 12
#define PWM_RESOLUTION 10
#define PWM_FREQUENCY 4000
#define MOTOR_LOWFRACTION_STARTTURN 0.27 //  low voltage for motor that needs more voltage is 0.55 and 0.27 for motor that needs less voltage
#define MOTOR_LOWFRACTION_TURNING 0.07 // the motor can go as low as 7% of it's nominal RPM
#define ALLOWABLE_ERROR_RPM 100 // the motor rpm is allowd to be 100 RPM off the other Motor's RPM
#define MILLISEC_IN_SEC 1000
#define SEC_IN_MIN 60
#define RPM_MAX 30000
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define STOP 0
#define MOTORRIGHT 1
#define MOTORLEFT 2


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
        void IRAM_ATTR setDutyMotor(const unsigned int motoNum, const int duty);
        int32_t IRAM_ATTR getEncoderValueLEFT(void);
        int32_t IRAM_ATTR getEncoderValueRIGHT(void);
        // void rpmcontrol(int rpmVorgabe);
        // void updateEncoderAndRPM(unsigned long dT);

        float IRAM_ATTR CalculateRPMfromEncoderValue(const int32_t encValue, const float dT);

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
        int32_t IRAM_ATTR IRegler(const uint32_t Sollwertdrehzahl, volatile uint32_t Stellwert, const uint32_t Drehzahlvalue);

        int32_t IRAM_ATTR mapInteger(const float x, const float in_min, const float in_max, const float out_min, const float out_max);

        volatile int32_t Stellwertrechts = 0;
        volatile int32_t Stellwertlinks = 0;
        uint32_t minPWMvaluestartturn;
        uint32_t minPWMvalueturning;
        uint32_t maxPWMvalue;

    private:

        // global variables which needs to be saved globally
        float iVal_rightWheel;
        float iVal_leftWheel;
        volatile int32_t rightrpmValue;
        volatile int32_t leftrpmValue;
        uint32_t Adjustingstep;

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