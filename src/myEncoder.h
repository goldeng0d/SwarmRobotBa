
#ifndef myEncoder_h__
#define myEncoder_h__


#include <Arduino.h>

#define mvAvg_length 5

struct myEncoder_pins
{

  // myEncoder has EncA and EncB pins
  uint8_t encA;
  uint8_t encB;
};

// All the data needed by interrupts is consolidated into this ugly struct
// to facilitate assembly language optimizing of the speed critical update.
// The assembly code uses auto-incrementing addressing modes, so the struct
// must remain in exactly this order.
typedef struct
{
  volatile uint32_t *pin1_register;
  volatile uint32_t *pin2_register;
  uint32_t pin1_bitmask;
  uint32_t pin2_bitmask;
  uint8_t state;
  int32_t position;
} Encoder_internal_state_t;

struct myEncoder_struct{
  //sensor pin (can be any for the esp32 cause function is handled by abstraction matrix)
  myEncoder_pins enc;
  
  // counts per revolution -> rising and falling edge is measured
  unsigned long cntPerRev;

  // amount of counts since start
   volatile long counter;
  // amount of counts since last tie it was updated
  volatile long lastCounter;

  // radius of the wheel
  double radius;

  // wheel rounds per second (moving average)
  double mvAvgOmega[mvAvg_length];
  // wheel rounds per second
  double omega;

  // tangential linearvel of one wheel
  double vel;

  // total distance travelled
  volatile double distance;

};

//Encoder libenc(myEncoder_pins enc.encA, myEncoder_pins enc.encB);

class myEncoder{

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
      myEncoder(double rate, double wheelDiameter, unsigned long countsPerRevolution, myEncoder_pins enc1, myEncoder_pins enc2);
      //Encoder myEnc(myEncoder_pins enc1, myEncoder_pins enc2);
      /**
       * Destructor does not do anything
      */
      ~myEncoder();

      // void IRAM_ATTR ISR_enc1();
      // void IRAM_ATTR ISR_enc2();
      // void IRAM_ATTR ISR_enc3();
      // void IRAM_ATTR ISR_enc4();

      // struct to store date of encoderLeft
      myEncoder_struct encoderLeft;

      // struct to store internal libencoderLeft values
      Encoder_internal_state_t libencoderLeft;

      // struct to store date of encoderRight
      myEncoder_struct encoderRight;

      // struct to store internal libencoderRight values
      Encoder_internal_state_t libencoderRight;

      /**
       * attatch rising and falling edge interrupts to pins
      */
      void setup();

      /**
       * update counter, omega, velocity and distance of both encoder structs
      */
      void myupdate();

      /**
       * update counter, omega, velocity and distance of both encoder structs
      */
      void myupdates();

      /**
       * This encoder cant detect rotation direction by itself -> needs to be supplied
       * 
       * @param[in] encoderNum encoder number 1 or 2
      */
      //void setCntDir(unsigned int encoderNum);

      /**
       * print total driven distance to SerialPort
      */
      void printDistance();

      /**
       * update the counter variable
      */
      inline int32_t readRIGHT();
      inline int32_t readLEFT();

      /**
       * update the counter variable and reset the internal interrupt counter
      */
       int32_t readAndResetRIGHT();
       int32_t readAndResetLEFT();

       /**
       * write a certain variable p into the internal interrupt counter
       * @param[in] p variable that is written into the internal interrupt counter
      */
       inline void writeRIGHT(int32_t p);
       inline void writeLEFT(int32_t p);

       //static void update(Encoder_internal_state_t *arg);

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
       void calculateValues(myEncoder_struct &encoder, double dT);
      

};

#endif