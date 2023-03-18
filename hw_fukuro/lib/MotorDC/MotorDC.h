#ifndef MotorDC_H
#define MotorDC_H

#include <Arduino.h>
#include <PCF8574.h>

#define FREQUENCY 100
#define RESOLUTION 8

// PCF8574 expansion(0x20);

class Motor
{
private:
    uint8_t pin_en_;
    uint8_t pin_pwm_;
    uint8_t channel_;
    PCF8574 *expansion_;

public:
    Motor(uint8_t pin_en, uint8_t pin_pwm, uint8_t channel, PCF8574 *expansion);

    void speed(float speed);

    void period(float period);

    void operator=(float value)
    {
        float value_ = value;
        speed(value_);
    };
};

#endif