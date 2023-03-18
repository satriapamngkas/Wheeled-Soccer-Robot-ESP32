#include <Arduino.h>
#include <MotorDC.h>
#include <Encoder_Fukuro.h>
#include <ros.h>
#include <fukuro_common/STMData.h>
#include <fukuro_common/SerialData.h>
#include <std_msgs/String.h>

#define LED_BUILTIN 2

PCF8574 expansion(0x20);
Motor motorKanan(23, P0, &expansion);
Motor motorKiri(5, P1, &expansion);
Motor motorBelakang(14, P2, &expansion);
Motor motorDribKanan(18, P3, &expansion);
Motor motorDribKiri(19, P4, &expansion);

Encoder encoderMotorKiri(36, 39, 269);
Encoder encoderMotorKanan(34, 35, 269);
Encoder encoderMotorBelakang(13, 4, 269);
Encoder encoderFreeKiri(25, 27, 269);
Encoder encoderFreeKanan(32, 15, 269);
Encoder encoderFreeDepan(26, 33, 269);

unsigned char sensor_data[2048],
    pwm_data[2048],
    kick_data[2048],
    brake_data[2048];

double velKiri = 0.0,
       velKanan = 0.0,
       velBelakang = 0.0,
       velFreeKiri = 0.0,
       velFreeKanan = 0.0,
       velFreeDepan = 0.0;

float motor1 = 0.0,
      motor2 = 0.0,
      motor3 = 0.0,
      drib1 = 0.0,
      drib2 = 0.0,
      sudut = 0.0;

uint8_t del_kick = 0;
bool outputrem = 0;

fukuro_common::STMData stmData;
ros::NodeHandle nh;
ros::Publisher stm("/fukuro_stm_pc", &stmData);

void updateData(const fukuro_common::SerialData &pwm)
{
    motor1 = pwm.motor.m1;
    motor2 = pwm.motor.m2;
    motor3 = pwm.motor.m3;
    drib1 = pwm.kecepatan.speed1;
    drib2 = pwm.kecepatan.speed2;
    del_kick = pwm.kick;
    outputrem = pwm.motor_brake;
}
bool led_state = 0;
void updateSData(const std_msgs::String &pwm)
{
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state);
    digitalWrite(16, led_state);
    // expansion.digitalWrite(P5, LOW);
}
ros::Subscriber<fukuro_common::SerialData> pc_data("/fukuro1/pwminfo", &updateData);
ros::Subscriber<std_msgs::String> string_data("/fukuro1/string", &updateSData);

void pwmOut()
{
    motorKiri.speed(motor1);
    motorKanan.speed(motor2);
    motorBelakang.speed(motor3);
    motorDribKiri.speed(drib1);
    motorDribKanan.speed(drib2);
}

void bacaEncoder()
{
    velKanan = (double)encoderMotorKanan.getPulses();
    velKiri = (double)encoderMotorKiri.getPulses();
    velBelakang = (double)encoderMotorBelakang.getPulses();
    velFreeKanan = (double)encoderFreeKanan.getPulses();
    velFreeKiri = (double)encoderFreeKiri.getPulses();
    velFreeDepan = (double)encoderFreeDepan.getPulses();
    stmData.encoder.m1 = velKiri;
    stmData.encoder.m2 = velKanan;
    stmData.encoder.m3 = velBelakang;
    stmData.freeenc.free1 = velFreeKanan;
    stmData.freeenc.free2 = velFreeKiri;
    stmData.freeenc.free3 = velFreeDepan;
}

void resetEncoder()
{
    encoderMotorKiri.reset();
    encoderMotorKanan.reset();
    encoderMotorBelakang.reset();
    encoderFreeKiri.reset();
    encoderFreeKanan.reset();
    encoderFreeDepan.reset();
}

void kickReady()
{
}

void bacaBola()
{
    // stmData.ir=
}

void IRAM_ATTR encoderMotorKiriISR()
{
    encoderMotorKiri.encode();
}

void IRAM_ATTR encoderMotorKananISR()
{
    encoderMotorKanan.encode();
}

void IRAM_ATTR encoderMotorBelakangISR()
{
    encoderMotorBelakang.encode();
}

void IRAM_ATTR encoderFreeKiriISR()
{
    encoderFreeKiri.encode();
}

void IRAM_ATTR encoderFreeKananISR()
{
    encoderFreeKanan.encode();
}

void IRAM_ATTR encoderFreeDepanISR()
{
    encoderFreeDepan.encode();
}

void setPinMode()
{
    pinMode(36, INPUT);
    pinMode(39, INPUT);
    attachInterrupt(digitalPinToInterrupt(36), encoderMotorKiriISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(39), encoderMotorKiriISR, CHANGE);
    pinMode(34, INPUT);
    pinMode(35, INPUT);
    attachInterrupt(digitalPinToInterrupt(34), encoderMotorKananISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(35), encoderMotorKananISR, CHANGE);
    pinMode(13, INPUT);
    pinMode(4, INPUT);
    attachInterrupt(digitalPinToInterrupt(13), encoderMotorBelakangISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(4), encoderMotorBelakangISR, CHANGE);
    pinMode(25, INPUT);
    pinMode(27, INPUT);
    attachInterrupt(digitalPinToInterrupt(25), encoderFreeKiriISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(27), encoderFreeKiriISR, CHANGE);
    pinMode(32, INPUT);
    pinMode(15, INPUT);
    attachInterrupt(digitalPinToInterrupt(32), encoderFreeKananISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(15), encoderFreeKananISR, CHANGE);
    pinMode(26, INPUT);
    pinMode(33, INPUT);
    attachInterrupt(digitalPinToInterrupt(26), encoderFreeDepanISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(33), encoderFreeDepanISR, CHANGE);
}

void sensorTask(void *parameters)
{
    for (;;)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        bacaEncoder();
        // Serial.println(velKanan);
        resetEncoder();
        bacaBola();
        kickReady();
        stm.publish(&stmData);
    }
}

void pwmTask(void *parameters)
{
    for (;;)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        // Serial.println("Task PWM");
        pwmOut();
    }
}

void kickTask(void *parameters)
{
    for (;;)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        // Serial.println("Task Kick");

        if (del_kick)
        {
            // digitalWrite(kick_pin, HIGH);
            vTaskDelay(del_kick / portTICK_PERIOD_MS);
            // digitalWrite(kick_pin, LOW);
            del_kick = 0;
        }
    }
}

void setup()
{
    setPinMode();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(16, OUTPUT);
    // expansion.pinMode(P5, OUTPUT);

    nh.initNode();
    // nh.subscribe(pc_data);
    nh.subscribe(string_data);

    nh.advertise(stm);

    xTaskCreate(sensorTask, "Sensor Task", 2048, (void *)sensor_data, 1, NULL);
    xTaskCreate(pwmTask, "PWM Task", 2048, (void *)pwm_data, 1, NULL);
    xTaskCreate(kickTask, "Kick Task", 2048, (void *)kick_data, 1, NULL);
    // Serial.begin(9600);
}

void loop()
{
    nh.spinOnce();
    // Serial.println(velKanan);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    // delay(20);
}