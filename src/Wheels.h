
#ifndef wheels_h__
#define wheels_h__


#include <Arduino.h>

#define mvAvg_length 5

struct Wheel_struct{
  //sensor pin (can be any for the esp32 cause function is handled by abstraction matrix)
  int pin;

  // counts per revolution -> rising and falling edge is measured
  unsigned long cntPerRev;

  // amount of counts since start
  long counter;
  // amount of counts since last tie it was updated
  long lastCounter;

  // radius of the wheel
  double radius;

  // wheel rounds per second (moving average)
  double mvAvgOmega[mvAvg_length];
  // wheel rounds per second
  double omega;

  // tangential linearvel of one wheel
  double vel;

  // total distance travelled
  double distance;

};



class Wheels{

    public:
      /**
       * Configure PWM parameter
       * 
       * @param[in] rate rate on which counts are updated -> higher rate leads to larger errors on small speeds
       * @param[in] wheelDiameter Diameter of wheel in mm
       * @param[in] countsPerRevolution counts per revolution -> rising and falling edge is measured
       * @param[in] enc1pin sensor pin (can be any for the esp32 because function is handled by hardware abstraction matrix)
       * @param[in] enc2pin sensor pin (can be any for the esp32 because function is handled by hardware abstraction matrix)
      */
      Wheels(double rate, double wheelDiameter, unsigned long countsPerRevolution, int enc1pin, int enc2pin);

      /**
       * Destructor does not do anything
      */
      ~Wheels();

      // struct to store date of wheel 1
      Wheel_struct wheel1;
      // struct to store date of wheel 2
      Wheel_struct wheel2;

      /**
       * attatch rising and falling edge interrupts to pins
      */
      void setup();

      /**
       * update counter, omega, velocity and distance of both wheel structs
      */
      void update();

      /**
       * This encoder cant detect rotation direction by itself -> needs to be supplied
       * 
       * @param[in] wheelNum Wheel number 1 or 2
       * @param[in] forward is rotation direction forward? -> cant detect by itself
      */
      void setCntDir(unsigned int wheelNum, bool forward);

      /**
       * print total driven distance to SerialPort
      */
      void printDistance();



    private:
      double updateRate;
      unsigned long previousMillis = 0;

      /**
       * calculate omega, velocity and distance of both wheel structs
       * uses elapsed time, measured counters and wheel radius
       * 
       * @param[in] wheel strcut of the wheels which shall be updated
       * @param[in] dT elapsed time since function was called the last time
      */
      void calculateValues(Wheel_struct & wheel, double dT);
      

};

#endif