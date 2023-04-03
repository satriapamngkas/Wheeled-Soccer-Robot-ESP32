#include <MotorDC.h>

Motor::Motor(uint8_t pin_en, uint8_t pin_pwm, uint8_t channel, PCF8574 *expansion) : pin_en_(pin_en), pin_pwm_(pin_pwm), channel_(channel), expansion_(expansion)
{
    // channel_ = channel++;
    ledcSetup(channel_, FREQUENCY, RESOLUTION);
    ledcAttachPin(pin_en_, channel_);
    // pinMode(pin_en, OUTPUT);
    expansion_->pinMode(pin_pwm_, OUTPUT);

    ledcWrite(channel_, 0);
    // analogWrite(pin_en, 0);
    expansion_->digitalWrite(pin_pwm_, LOW);
}

void Motor::speed(float speed)
{
    if (speed > float(0.0))
    {
        expansion_->digitalWrite(pin_pwm_, LOW);
    }
    else
    {
        expansion_->digitalWrite(pin_pwm_, HIGH);
    }
    double pwm = fabs(speed) > 0.94 ? 0.94 : fabs(speed);
    pwm = pwm * 250;
    ledcWrite(channel_, pwm);
    // analogWrite(pin_en_, pwm);
}

void Motor::freq(float freq)
{
    ledcSetup(channel_, freq, RESOLUTION);
}