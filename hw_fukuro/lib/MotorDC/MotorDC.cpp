#include <MotorDC.h>

Motor::Motor(uint8_t pin_en, uint8_t pin_pwm, uint8_t channel, PCF8574 *expansion) : pin_en_(pin_en), pin_pwm_(pin_pwm), channel_(channel), expansion_(expansion)
{
    ledcSetup(channel, FREQUENCY, RESOLUTION);
    ledcAttachPin(pin_en, channel);
    // pinMode(pin_en, OUTPUT);
    expansion_->pinMode(pin_pwm, OUTPUT);

    ledcWrite(channel, 0);
    // analogWrite(pin_en, 0);
    expansion_->digitalWrite(pin_pwm, LOW);
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