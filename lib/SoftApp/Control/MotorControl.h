#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "pid.h"
#include "../../Hardware/MOTOR/motor.h"
#include "Communication/IIC/MPU6050_IIC/mpu6050_iic.h"
#include "Network/WiFi/WiFiReceiver.h"

struct PidLoopSettings
{
    float kp;
    float ki;
    float kd;

    PidLoopSettings(float p = 0.0f, float i = 0.0f, float d = 0.0f)
        : kp(p), ki(i), kd(d)
    {
    }
};

struct PidSettings
{
    PidLoopSettings rollAngle;
    PidLoopSettings pitchAngle;
    PidLoopSettings yawRate;
    PidLoopSettings rollRate;
    PidLoopSettings pitchRate;
};

void configureMotorPid();
void resetMotorControl();
bool applyMotorPidSettings(const PidSettings &settings);
bool MotorControl(
    const softapp::wifi::RemoteCommand &command,
    const softapp::mpu6050::Data &imu,
    float dt,
    hardware::motor::motor &output
);

#endif
