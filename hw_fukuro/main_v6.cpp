#include <mbed.h>
#include <Motor2pin.h>
#include <ros.h>
#include <encoder.h>
#include <fukuro_common/STMData.h>
#include <fukuro_common/SerialData.h>
#include <rtos.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <imumaths.h>

// #define SDA PC_12
// #define SCL PB_10

unsigned char sensor_data[2048];
unsigned char pwm_data[2048];
unsigned char kick_data[2048];
unsigned char brake_data[2048];

DigitalOut kickpin(PA_6);
DigitalIn kickreadypin(PC_13);
DigitalOut outputrempin(PA_7);
DigitalIn g1(PA_1); // tadinya PC_3

// Serial pc(USBTX, USBRX);

//====PARAMETER (PWM, FWD)===
// Motor motorKanan    (PC_6, PC_0);
// Motor motorKiri     (PC_7, PC_1);
// Motor motorBel      (PC_8, PC_3);
// Motor motorDrib     (PC_9, PA_0);
// Parameter Lama
Motor motorKanan(PC_9, PA_0); // PC_7 PC_1
Motor motorKiri(PC_6, PC_0);
Motor motorBel(PC_8, PA_4);
Motor motorDrib(PC_7, PC_1);

double velKiri = 0.0,
       velKanan = 0.0,
       velBelakang = 0.0,
       velFreeKanan = 0.0,
       velFreeKiri = 0.0,
       velFreeDpn = 0.0;

encoderstm encoderMotorKiri(PB_6, PC_2, 269, encoderstm::X4_ENCODING);
encoderstm encoderMotorKanan(PB_14, PB_15, 269, encoderstm::X4_ENCODING);
encoderstm encoderMotorBelakang(PA_5, PA_11, 269, encoderstm::X4_ENCODING);
encoderstm encoderMotorFreeKanan(PB_3, PB_13, 269, encoderstm::X4_ENCODING);
encoderstm encoderMotorFreeKiri(PB_8, PB_9, 269, encoderstm::X4_ENCODING);
encoderstm encoderMotorFreeDpn(PB_7, PB_0, 269, encoderstm::X4_ENCODING);

fukuro_common::STMData stmData;
ros::NodeHandle nh;
ros::Publisher stm("/fukuro_stm_pc", &stmData);
float motor1 = 0.0, motor2 = 0.0, motor3 = 0.0, motor4 = 0.0, sudut = 0.0;
uint8_t del_kick = 0;
bool outputrem = 0;

void updateData(const fukuro_common::SerialData &pwm)
{
    motor1 = pwm.motor.m1;
    motor2 = pwm.motor.m2;
    motor3 = pwm.motor.m3;
    motor4 = pwm.kecepatan.speed;
    del_kick = pwm.kick;
    outputrem = pwm.motor_brake;
}

void PWMOut()
{
    motorKiri.speed(motor1);
    motorKanan.speed(motor2);
    motorBel.speed(motor3);
    motorDrib.speed(motor4);
}

void BacaEncoder()
{
    velKanan = (double)encoderMotorKanan.getPulses();
    velKiri = (double)encoderMotorKiri.getPulses();
    velBelakang = (double)encoderMotorBelakang.getPulses();
    velFreeKanan = (double)encoderMotorFreeKanan.getPulses();
    velFreeKiri = (double)encoderMotorFreeKiri.getPulses();
    velFreeDpn = (double)encoderMotorFreeDpn.getPulses();
    stmData.encoder.m1 = velKiri;
    stmData.encoder.m2 = velKanan;
    stmData.encoder.m3 = velBelakang;
    stmData.freeenc.free1 = velFreeKanan;
    stmData.freeenc.free2 = velFreeKiri;
    stmData.freeenc.free3 = velFreeDpn;
}
void kick_ready()
{
    stmData.ready_kick = kickreadypin;
}

void BacaBola()
{
    stmData.ir = g1;
}

/*void I2C_SudutBatt(){
    pc.baud(115200);
    bno.setExtCrystalUse(true);
    while(1){
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        float SudutBNO=euler.x();
        wait(0.1);
    }
}*/

void ResetEncoder()
{
    encoderMotorKiri.reset();
    encoderMotorKanan.reset();
    encoderMotorBelakang.reset();
    encoderMotorFreeKanan.reset();
    encoderMotorFreeKiri.reset();
    encoderMotorFreeDpn.reset();
}

ros::Subscriber<fukuro_common::SerialData> pc_data("/fukuro1/pwminfo", &updateData);

void Sensor_Thread(void const *pvArgs)
{
    while (1)
    {
        ThisThread::sleep_for(20);
        BacaEncoder();
        ResetEncoder();
        BacaBola();
        kick_ready();
        //      I2C_SudutBatt();
        stm.publish(&stmData);
    }
}

void PWM_Thread(void const *pvArgs)
{
    while (1)
    {
        ThisThread::sleep_for(20);
        PWMOut();
    }
}

void Kick_Thread(void const *pvArgs)
{
    while (1)
    {
        ThisThread::sleep_for(20);
        if (del_kick)
        {
            kickpin = 1;
            wait_ms(del_kick);
            kickpin = 0;
            del_kick = 0;
        }
    }
}

void Brake_Thread(void const *pvArgs)
{
    while (1)
    {
        ThisThread::sleep_for(20);
        outputrempin = outputrem;
    }
}

int main()
{
    // 0.01 0.0217
    // fukuro2
    motorKanan.period(0.01);
    motorKiri.period(0.01);
    motorBel.period(0.01);
    // fukuro3
    // motorKanan.period(0.0055);
    //        motorKiri.period(0.005); //tadinya 0.055
    //        motorBel.period(0.006);

    motorDrib.period(0.01);

    nh.initNode();
    nh.subscribe(pc_data);
    nh.advertise(stm);
    Thread thSensor(Sensor_Thread, NULL, osPriorityNormal, 2048, &sensor_data[0]);
    Thread thPWM(PWM_Thread, NULL, osPriorityNormal, 2048, &pwm_data[0]);
    Thread thkick(Kick_Thread, NULL, osPriorityNormal, 2048, &kick_data[0]);
    Thread thBrake(Brake_Thread, NULL, osPriorityNormal, 2048, &brake_data[0]);
    while (1)
    {
        nh.spinOnce();
        ThisThread::sleep_for(20);
    }
}