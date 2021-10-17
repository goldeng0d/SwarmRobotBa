
#ifndef Encoder_h__
#define Encoder_h__


#include <Arduino.h>

#define mvAvg_length 5

struct Encoder_pins
{

  // Encoder has EncA and EncB pins
  unsigned int encA;
  unsigned int encB;
};

struct Encoder_struct{
  //sensor pin (can be any for the esp32 cause function is handled by abstraction matrix)
  Encoder_pins enc;

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



class Encoder{

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
      Encoder(double rate, double wheelDiameter, unsigned long countsPerRevolution, Encoder_pins enc1, Encoder_pins enc2);

      /**
       * Destructor does not do anything
      */
      ~Encoder();

      // struct to store date of encoder 1
      Encoder_struct encoder1;
      // struct to store date of encoder 2
      Encoder_struct encoder2;

      /**
       * attatch rising and falling edge interrupts to pins
      */
      void setup();

      /**
       * update counter, omega, velocity and distance of both encoder structs
      */
      void update();

      /**
       * This encoder cant detect rotation direction by itself -> needs to be supplied
       * 
       * @param[in] encoderNum encoder number 1 or 2
       * @param[in] forward is rotation direction forward? -> cant detect by itself
      */
      void setCntDir(unsigned int encoderNum, bool forward);

      /**
       * print total driven distance to SerialPort
      */
      void printDistance();



    private:
      double updateRate;
      unsigned long previousMillis = 0;

      /**
       * calculate omega, velocity and distance of both encoder structs
       * uses elapsed time, measured counters and wheel radius
       * 
       * @param[in] encoder strcut of the encoders which shall be updated
       * @param[in] dT elapsed time since function was called the last time
      */
      void calculateValues(Encoder_struct & encoder, double dT);
      

};

#endif