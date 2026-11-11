#include "MotorControl.h"

#include <math.h>

namespace
{

PID_t RollPID;
PID_t PitchPID;
PID_t YawPID;
PID_t RollVelocityPID;
PID_t PitchVelocityPID;
portMUX_TYPE sPidMux = portMUX_INITIALIZER_UNLOCKED;

constexpr float kMaxRollPitchRateDps = 120.0f;
constexpr float kMaxYawRateDps = 150.0f;
constexpr float kMaxRateCorrection = 1000.0f;
constexpr uint32_t kCommandTimeoutMs = 500;
constexpr float kMaxCommandThrust = 65000.0f;

float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void configurePid(
    PID_t &pid,
    float kp,
    float ki,
    float kd,
    float outMin,
    float outMax,
    float integralMin,
    float integralMax
)
{
    pid = {};
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.OutMin = outMin;
    pid.OutMax = outMax;
    pid.IntegralMin = integralMin;
    pid.IntegralMax = integralMax;
    pid.DFilterCoef = 0.2f;
}

void resetPid(PID_t &pid)
{
    pid.Target = 0.0f;
    pid.Actual = 0.0f;
    pid.Actual1 = 0.0f;
    pid.Out = 0.0f;
    pid.Error0 = 0.0f;
    pid.Error1 = 0.0f;
    pid.Diff = 0.0f;
    pid.Inter = 0.0f;
    pid.POut = 0.0f;
    pid.IOut = 0.0f;
    pid.DOut = 0.0f;
    pid.dt = 0.0f;
    pid.DFiltered = 0.0f;
    pid.LastActual = 0.0f;
    pid.Initialized = false;
}

void resetPidInternal()
{
    resetPid(RollPID);
    resetPid(PitchPID);
    resetPid(RollVelocityPID);
    resetPid(PitchVelocityPID);
    resetPid(YawPID);
}

float updatePid(PID_t &pid, float target, float actual, float dt)
{
    if (dt <= 0.0f)
    {
        return pid.Out;
    }

    if (!pid.Initialized)
    {
        pid.LastActual = actual;
        pid.Initialized = true;
    }

    pid.Target = target;
    pid.Actual = actual;
    pid.Error0 = target - actual;
    pid.POut = pid.Kp * pid.Error0;

    pid.Inter += pid.Ki * pid.Error0 * dt;
    pid.Inter = clampFloat(
        pid.Inter,
        pid.IntegralMin,
        pid.IntegralMax
    );
    pid.IOut = pid.Inter;

    const float derivative =
        -(actual - pid.LastActual) / dt;
    pid.DFiltered =
        pid.DFilterCoef * derivative +
        (1.0f - pid.DFilterCoef) * pid.DFiltered;
    pid.DOut = pid.Kd * pid.DFiltered;

    pid.Out = clampFloat(
        pid.POut + pid.IOut + pid.DOut + pid.OutOffset,
        pid.OutMin,
        pid.OutMax
    );

    pid.Actual1 = pid.Actual;
    pid.LastActual = actual;
    pid.Error1 = pid.Error0;
    pid.dt = dt;
    return pid.Out;
}

int16_t clampMotorCommand(float value)
{
    return static_cast<int16_t>(
        lroundf(
            clampFloat(
                value,
                static_cast<float>(hardware::motor::kMinSpeed),
                static_cast<float>(hardware::motor::kMaxSpeed)
            )
        )
    );
}

} // namespace

//KP,KI,KD
void configureMotorPid()
{
    // Tiny-Drone defaults converted from its UINT16 motor-output scale
    // (65535) to this firmware's 0..1000 motor scale.
    configurePid(
        RollPID,
        5.3f, 2.5f, 0.0f,
        -kMaxRollPitchRateDps, kMaxRollPitchRateDps,
        -40.0f, 40.0f
    );
    configurePid(
        PitchPID,
        5.3f, 2.5f, 0.0f,
        -kMaxRollPitchRateDps, kMaxRollPitchRateDps,
        -40.0f, 40.0f
    );
    configurePid(
        RollVelocityPID,
        2.9f, 6.7f, 0.04f,
        -kMaxRateCorrection, kMaxRateCorrection,
        -200.0f, 200.0f
    );
    configurePid(
        PitchVelocityPID,
        2.9f, 6.7f, 0.04f,
        -kMaxRateCorrection, kMaxRateCorrection,
        -200.0f, 200.0f
    );
    configurePid(
        YawPID,
        1.8f, 0.25f, 0.0f,
        -kMaxYawRateDps, kMaxYawRateDps,
        -45.0f, 45.0f
    );
}

bool applyMotorPidSettings(const PidSettings &settings)
{
    const PidLoopSettings *loops[] = {
        &settings.rollAngle,
        &settings.pitchAngle,
        &settings.yawRate,
        &settings.rollRate,
        &settings.pitchRate,
    };
    for (const PidLoopSettings *loop : loops)
    {
        if (!isfinite(loop->kp) || !isfinite(loop->ki) ||
            !isfinite(loop->kd) ||
            loop->kp < 0.0f || loop->kp > 100.0f ||
            loop->ki < 0.0f || loop->ki > 100.0f ||
            loop->kd < 0.0f || loop->kd > 100.0f)
        {
            return false;
        }
    }

    portENTER_CRITICAL(&sPidMux);
    configurePid(
        RollPID,
        settings.rollAngle.kp,
        settings.rollAngle.ki,
        settings.rollAngle.kd,
        -kMaxRollPitchRateDps,
        kMaxRollPitchRateDps,
        -10.0f,
        10.0f
    );
    configurePid(
        PitchPID,
        settings.pitchAngle.kp,
        settings.pitchAngle.ki,
        settings.pitchAngle.kd,
        -kMaxRollPitchRateDps,
        kMaxRollPitchRateDps,
        -10.0f,
        10.0f
    );
    configurePid(
        YawPID,
        settings.yawRate.kp,
        settings.yawRate.ki,
        settings.yawRate.kd,
        -kMaxYawRateDps,
        kMaxYawRateDps,
        -30.0f,
        30.0f
    );
    configurePid(
        RollVelocityPID,
        settings.rollRate.kp,
        settings.rollRate.ki,
        settings.rollRate.kd,
        -kMaxRateCorrection,
        kMaxRateCorrection,
        -40.0f,
        40.0f
    );
    configurePid(
        PitchVelocityPID,
        settings.pitchRate.kp,
        settings.pitchRate.ki,
        settings.pitchRate.kd,
        -kMaxRateCorrection,
        kMaxRateCorrection,
        -40.0f,
        40.0f
    );
    resetPidInternal();
    portEXIT_CRITICAL(&sPidMux);
    return true;
}

//重置电机控制，清除积累误差
void resetMotorControl()
{
    portENTER_CRITICAL(&sPidMux);
    resetPidInternal();
    portEXIT_CRITICAL(&sPidMux);
}

//电机控制
bool MotorControl(const softapp::wifi::RemoteCommand &command,const softapp::mpu6050::Data &imu,float dt,hardware::motor::motor &output){
    output = hardware::motor::motor{};

    if (!command.valid ||!softapp::wifi::WiFiReceiver::isCommandFresh(command,kCommandTimeoutMs) ||command.thrust == 0 ||dt <= 0.0f)
    {
        resetMotorControl();
        return false;
    }

    portENTER_CRITICAL(&sPidMux);
    const float targetRollRate =
        updatePid(RollPID, command.rollDeg, imu.rollDeg, dt);
    const float targetPitchRate =
        updatePid(PitchPID, command.pitchDeg, imu.pitchDeg, dt);

    const float rollCorrection =
        updatePid(RollVelocityPID,targetRollRate,imu.gyroDps[0],dt);

    const float pitchCorrection =
        updatePid(PitchVelocityPID,targetPitchRate,imu.gyroDps[1],dt);
    const float yawCorrection =
        updatePid(YawPID,command.yawRateDps,imu.gyroDps[2],dt);

    const float throttle =
        static_cast<float>(command.thrust) *
        static_cast<float>(hardware::motor::kMaxSpeed) /
        kMaxCommandThrust;

    // X 型机架：对应 Crazyflie 原版 QUAD_FORMATION_X 混控。
    const float halfRoll = rollCorrection / 2.0f;
    const float halfPitch = pitchCorrection / 2.0f;
    output.speed[0] = clampMotorCommand(
        throttle - halfRoll + halfPitch + yawCorrection
    );
    output.speed[1] = clampMotorCommand(
        throttle - halfRoll - halfPitch - yawCorrection
    );
    output.speed[2] = clampMotorCommand(
        throttle + halfRoll - halfPitch + yawCorrection
    );
    output.speed[3] = clampMotorCommand(
        throttle + halfRoll + halfPitch - yawCorrection
    );

    portEXIT_CRITICAL(&sPidMux);
    return true;
}
