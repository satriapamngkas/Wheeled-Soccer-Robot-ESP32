#include <Arduino.h>
#include <MotorDC.h>
#include <Encoder_Fukuro.h>
#include <ros.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>
#include <fukuro_common/STMData.h>
#include <fukuro_common/SerialData.h>
#include <fukuro_common/MotorVel.h>
#include <fukuro_common/DribblerVel.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

#define debugLed 12
#define kickPin 16
#define irPin P5
#define servoPin 17

#define VL
#define BNO
bool led_state = 0, vl_state = 0;

PCF8574 expansion(0x20);
Servo servoKick;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_VL53L0X vl = Adafruit_VL53L0X();

Motor motorKiri(5, P0, 5, &expansion);
Motor motorKanan(23, P3, 3, &expansion);
Motor motorBelakang(14, P2, 2, &expansion);
Motor motorDribKanan(18, P1, 6, &expansion);
Motor motorDribKiri(19, P4, 4, &expansion);

Encoder encoderMotorKiri(36, 39, 269);
Encoder encoderMotorKanan(34, 35, 269);
Encoder encoderMotorBelakang(4, 15, 269);
Encoder encoderFreeKiri(26, 25, 269);
Encoder encoderFreeKanan(33, 32, 269);
Encoder encoderFreeDepan(27, 13, 269);

unsigned char sensor_data[2048],
    pwm_data[2048],
    kick_data[2048];

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
      sudut = 0.0,
      orientation_x = 0.0,
      ir_now = 0.0,
      ir_prev = 0.0,
      filtered = 0.0,
      gain = 0.25,
      distance = 0.0;

uint8_t del_kick = 0,
        servo_angle = 100;

fukuro_common::STMData stmData;
ros::NodeHandle nh;
ros::Publisher stm("/fukuro_stm_pc", &stmData);

// void updateData(const fukuro_common::SerialData &pwm)
// {
//     // led_state=!led_state;
//     // digitalWrite(debugLed, led_state);
//     // digitalWrite(16, led_state);
//     motor1 = pwm.motor.m1;
//     motor2 = pwm.motor.m2;
//     motor3 = pwm.motor.m3;
//     drib1 = pwm.dribbler.d1;
//     drib2 = pwm.dribbler.d2;
//     del_kick = pwm.kick;
//     outputrem = pwm.motor_brake;
// }

void updateKickData(const std_msgs::UInt8 &pwm)
{
    del_kick = pwm.data;
}

void updateServoData(const std_msgs::UInt8 &pwm)
{
    servo_angle = pwm.data;
}

void updateMotorData(const fukuro_common::MotorVel &pwm)
{
    motor1 = pwm.m1;
    motor2 = pwm.m2;
    motor3 = pwm.m3;
}

void updateDribbleData(const fukuro_common::DribblerVel &pwm)
{
    drib1 = pwm.d1;
    drib2 = pwm.d2;
}

ros::Subscriber<fukuro_common::MotorVel> motor_data("/fukuro_mcu/motor", &updateMotorData);
ros::Subscriber<fukuro_common::DribblerVel> dribble_data("/fukuro_mcu/dribbler", &updateDribbleData);
// ros::Subscriber<fukuro_common::SerialData> pc_data("/fukuro_mcu/pwminfo", &updateData);
ros::Subscriber<std_msgs::UInt8> kicker_data("/fukuro_mcu/kick", &updateKickData);
ros::Subscriber<std_msgs::UInt8> servo_data("/fukuro_mcu/servo", &updateServoData);

void pwmOut()
{
    motorKiri.speed(motor1);
    motorKanan.speed(motor2);
    motorBelakang.speed(motor3);
    motorDribKiri.speed(drib1);
    motorDribKanan.speed(drib2);
    // servoKick.write(servo_angle);
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

void bacaBNO()
{
    imu::Quaternion quat = bno.getQuat();
    double yy = quat.y() * quat.y();
    double yaw = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2 * (yy + quat.z() * quat.z()));
    float yawDeg = 57.2958 * yaw;
    // sensors_event_t event;
    // bno.getEvent(&event);
    // orientation_x = event.orientation.x;
    stmData.yaw = yawDeg;
}

void bacaBola()
{
    int ir_read = !expansion.digitalRead(irPin);
    filtered = filtered + gain * (ir_read - filtered);
    // ir_prev = ir_now;
    stmData.ir = filtered > 0.5 ? 1 : 0;

#ifdef VL
    if (ir_read)
    {
        vl_state = 1;
    }

    if (vl_state)
    {
        VL53L0X_RangingMeasurementData_t measure;
        vl.rangingTest(&measure, false);
        distance = measure.RangeMilliMeter;
        stmData.distance = distance;
        if (distance > 100)
        {
            vl_state = 0;
        };
    }
#endif
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

void PinInit()
{
    pinMode(36, INPUT);
    pinMode(39, INPUT);
    attachInterrupt(digitalPinToInterrupt(36), encoderMotorKiriISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(39), encoderMotorKiriISR, CHANGE);
    pinMode(34, INPUT);
    pinMode(35, INPUT);
    attachInterrupt(digitalPinToInterrupt(34), encoderMotorKananISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(35), encoderMotorKananISR, CHANGE);
    pinMode(4, INPUT);
    pinMode(15, INPUT);
    attachInterrupt(digitalPinToInterrupt(4), encoderMotorBelakangISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(15), encoderMotorBelakangISR, CHANGE);
    pinMode(25, INPUT);
    pinMode(26, INPUT);
    attachInterrupt(digitalPinToInterrupt(25), encoderFreeKiriISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(26), encoderFreeKiriISR, CHANGE);
    pinMode(33, INPUT);
    pinMode(32, INPUT);
        // bacaBola();
    attachInterrupt(digitalPinToInterrupt(33), encoderFreeKananISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(32), encoderFreeKananISR, CHANGE);
    pinMode(27, INPUT);
    pinMode(13, INPUT);
    attachInterrupt(digitalPinToInterrupt(27), encoderFreeDepanISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(13), encoderFreeDepanISR, CHANGE);
    pinMode(debugLed, OUTPUT);
    pinMode(kickPin, OUTPUT);
    expansion.pinMode(irPin, INPUT);
    // ESP32PWM::allocateTimer(0);
    // ESP32PWM::allocateTimer(1);
    // ESP32PWM::allocateTimer(2);
    // ESP32PWM::allocateTimer(3);
    servoKick.setPeriodHertz(50);
    servoKick.attach(servoPin, 1000, 2000);
}

void sensorTask(void *parameters)
{
    for (;;)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        bacaEncoder();
        resetEncoder();
        #ifdef BNO
        bacaBNO();
        #endif
        bacaBola();
        stm.publish(&stmData);
    }
}

void pwmTask(void *parameters)
{
    for (;;)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        pwmOut();
    }
}

void kickTask(void *parameters)
{
    for (;;)
    {
        vTaskDelay(20 / portTICK_PERIOD_MS);

        if (del_kick)
        {
            digitalWrite(kickPin, HIGH);
            vTaskDelay(del_kick / portTICK_PERIOD_MS);
            digitalWrite(kickPin, LOW);
            del_kick = 0;
        }
    }
}

void setup()
{
    PinInit();

#ifdef VL
    while (!vl.begin())
    {
        digitalWrite(debugLed, HIGH);
        delay(150);
        digitalWrite(debugLed, LOW);
        delay(150);
    }
#endif

#ifdef BNO
    while (!bno.begin())
    {
        digitalWrite(debugLed, HIGH);
        delay(150);
        digitalWrite(debugLed, LOW);
        delay(150);
    }
#endif

    while (!expansion.begin())
    {
        digitalWrite(debugLed, HIGH);
        delay(150);
        digitalWrite(debugLed, LOW);
        delay(150);
    }

    // motorKiri.freq(1000);
    // motorKanan.freq(1000);
    // motorBelakang.freq(1000);
    // motorDribKanan.freq(1000);
    // motorDribKiri.freq(1000);

    bno.setExtCrystalUse(true);

    nh.initNode();
    nh.subscribe(motor_data);
    nh.subscribe(dribble_data);
    nh.subscribe(kicker_data);
    nh.subscribe(servo_data);

    nh.advertise(stm);

    xTaskCreate(sensorTask, "Sensor Task", 2048, (void *)sensor_data, 1, NULL);
    xTaskCreate(pwmTask, "PWM Task", 2048, (void *)pwm_data, 1, NULL);
    xTaskCreate(kickTask, "Kick Task", 2048, (void *)kick_data, 1, NULL);
}

void loop()
{
    nh.spinOnce();
    vTaskDelay(20 / portTICK_PERIOD_MS);
}