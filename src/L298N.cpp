#include "L298N.h"

L298N::L298N(unsigned int frequency, unsigned int resolution, Motor_pins mot1, Motor_pins mot2){
    frequency_ = frequency;
    resolution_ = resolution;

    motor1 = mot1;
    motor2 = mot2;
}

L298N::~L298N(){
    ledcDetachPin(motor1.enable);
    ledcDetachPin(motor2.enable);
}

void L298N::setup(){

    // configre pins for output
    pinMode(motor1.enable, OUTPUT);
    pinMode(motor1.pin1, OUTPUT);
    pinMode(motor1.pin2, OUTPUT);

    pinMode(motor2.enable, OUTPUT);
    pinMode(motor2.pin1, OUTPUT);
    pinMode(motor2.pin2, OUTPUT);

    // set default rotation direction to forward
    changeDirection(1, motDirection::FORWARD);
    changeDirection(2, motDirection::FORWARD);

    // connect PWM output to pin on set channel through hardware abstraction amtrix
    ledcSetup(motor1.channel, frequency_, resolution_);
    ledcAttachPin(motor1.enable, motor1.channel);

    ledcSetup(motor2.channel, frequency_, resolution_);
    ledcAttachPin(motor2.enable, motor2.channel);

    // set duty cycle to 0
    myledcWrite(motor1.channel, 0);
    myledcWrite(motor2.channel, 0);
}

void IRAM_ATTR L298N::changeDuty(unsigned int motoNum, unsigned int duty)
{
    if(motoNum == 1){
        myledcWrite(1, duty);
    } else {
        myledcWrite(2, duty);
    }
}

// https://community.platformio.org/t/esp32-change-pwm-duty-cycle-in-interrupt-possible/24319
void IRAM_ATTR L298N::myledcWrite(uint8_t chan, uint32_t duty)
{
    if (chan > 15)
    {
        return;
    }
    uint8_t group = (chan / 8), channel = (chan % 8);
    LEDC_CHAN(group, channel).duty.duty = duty << 4; //25 bit (21.4)
    if (duty)
    {
        LEDC_CHAN(group, channel).conf0.sig_out_en = 1; //This is the output enable control bit for channel
        LEDC_CHAN(group, channel).conf1.duty_start = 1; //When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
        if (group)
        {
            LEDC_CHAN(group, channel).conf0.low_speed_update = 1;
        }
        else
        {
            LEDC_CHAN(group, channel).conf0.clk_en = 1;
        }
    }
    else
    {
        LEDC_CHAN(group, channel).conf0.sig_out_en = 0; //This is the output enable control bit for channel
        LEDC_CHAN(group, channel).conf1.duty_start = 0; //When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
        if (group)
        {
            LEDC_CHAN(group, channel).conf0.low_speed_update = 1;
        }
        else
        {
            LEDC_CHAN(group, channel).conf0.clk_en = 0;
        }
    }
}

int8_t L298N::changeSpeed(unsigned int motoNum, double percent){

    percent =  constrain(percent, 0.0, 100.0);
    unsigned int duty = mapDouble(percent, 0.0, 100.0, 60.0, pow(2,resolution_)-1);
    if (percent == 0){
        duty = 0;
    }
    changeDuty(motoNum, duty);
    //Uncomment if you want to see the duty cycle in serial Monitor
    Serial.printf("duty: %d\n", duty);
    return duty;
}


void L298N::changeDirection(unsigned int motoNum, motDirection direction){

    if(motoNum == 1){

        switch(direction){
            case motDirection::FORWARD:{
                digitalWrite(motor1.enable, LOW);
                digitalWrite(motor1.pin1, HIGH);
                digitalWrite(motor1.pin2, LOW);
                break;
            }
            case motDirection::BACKWARD:{
                digitalWrite(motor1.enable, LOW);
                digitalWrite(motor1.pin1, LOW);
                digitalWrite(motor1.pin2, HIGH);
                break;
            }
            case motDirection::BREAKING:{
                digitalWrite(motor1.enable, LOW);
                digitalWrite(motor1.pin1, LOW);
                digitalWrite(motor1.pin2, LOW);
                break;
            }
        }
    
    } else {
        
        switch(direction){
            case motDirection::FORWARD:{
                digitalWrite(motor2.enable, LOW);
                digitalWrite(motor2.pin1, HIGH);
                digitalWrite(motor2.pin2, LOW);
                break;
            }
            case motDirection::BACKWARD:{
                digitalWrite(motor2.enable, LOW);
                digitalWrite(motor2.pin1, LOW);
                digitalWrite(motor2.pin2, HIGH);
                break;
            }
            case motDirection::BREAKING:{
                digitalWrite(motor2.enable, LOW);
                digitalWrite(motor2.pin1, LOW);
                digitalWrite(motor2.pin2, LOW);
                break;
            }
        }

    }

}

double L298N::mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
    double result;
    result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return result;
}

unsigned int L298N::getResolution()
{
    return resolution_;
}