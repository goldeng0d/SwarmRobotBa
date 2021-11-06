
#ifndef l298n_h__
#define l298n_h__


#include <Arduino.h>
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"
#define LEDC_CHAN(g, c) LEDC.channel_group[(g)].channel[(c)]

struct Motor_pins{

    // see L298N Datasheet
    unsigned int pin1;
    unsigned int pin2;
    unsigned int enable;

    // ESP32 uses channels 0-15 for PWM
    unsigned int channel;

};

// enum for motor direction
enum motDirection {FORWARD, BACKWARD, BREAKING};


class L298N{

    public:

      /**
       * Constructor
       * 
       * @param[in] frequency PWM Frequency - does not matter too much for L298N
       * @param[in] resolution resolution of dutycyle in bits 0 ... 2^(resolution)
       * @param[in] motright Motor 1 configuration
       * @param[in] motleft Motor 2 configuration
      */
      L298N(const unsigned int frequency, const unsigned int resolution, const Motor_pins motright, const Motor_pins motleft);

      /**
       * detatch PWM function from GPIO Pin matrix
      */
      ~L298N();

      /**
       * Setup PINS and PWM
      */
      void setup();

      /**
       * Change speed of motor by duty cycle in bits from 0 ... 2^(resolution)
       * 
       * @param[in] motoNum Motor number A = 1 | B = 2
       * @param[in] duty dutycyle in bits 0 ... 2^(resolution)
      */
      void IRAM_ATTR changeDuty(const unsigned int motoNum, const unsigned int duty);

      /**
       * Change speed of motor in percent from 0% to 100%
       * 
       * @param[in] motoNum Motor number A = 1 | B = 2
       * @param[in] percent motor speed percentage
      */
      int8_t changeSpeed(unsigned int motoNum, double percent);

      /**
       * Change rotational direction or disable motor by breaking
       * 
       * @param[in] motoNum Motor number A = 1 | B = 2
       * @param[in] direction motor rotation direction
      */
      void changeDirection(unsigned int motoNum, motDirection direction);

      unsigned int getResolution();

      /**
       * map one value in range A one to range B
       * 
       * @param[in] x input value which shall be mapped
       * @param[in] in_min minimum value of input range A
       * @param[in] in_max maximum value of input range A
       * @param[in] out_min minimum value of output range B
       * @param[in] out_max maximum value of output range B
      */
      double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);

    private:
      
      // frequency of PWM
      unsigned int frequency_;
      // resolution in bits of PWM
      unsigned int resolution_;

      // Motor 1 Config
      Motor_pins motorright;
      // Motor 2 Config
      Motor_pins motorleft;

      // Motor1 rotation direction
      motDirection motorright_direction;
      // Motor2 rotation direction
      motDirection motorleft_direction;

      void IRAM_ATTR myledcWrite(uint8_t chan, uint32_t duty);
};

#endif