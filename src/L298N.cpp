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
    ledcWrite(motor1.channel, 0);
    ledcWrite(motor2.channel, 0);
}


void L298N::changeDuty(unsigned int motoNum, unsigned int duty){

    if(motoNum == 1){
        ledcWrite(1, duty);
    } else {
        ledcWrite(2, duty);
    }

}

void L298N::changeSpeed(unsigned int motoNum, double percent){

    percent =  constrain(percent, 0.0, 100.0);
    unsigned int duty = mapDouble(percent, 0.0, 100.0, 60.0, pow(2,resolution_)-1);
    if (percent == 0){
        duty = 0;
    }
    changeDuty(motoNum, duty);
    //Uncomment if you want to see the duty cycle in serial Monitor
    Serial.printf("duty: %d\n", duty);
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