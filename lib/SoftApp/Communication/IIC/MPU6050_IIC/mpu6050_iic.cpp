#include "mpu6050_iic.h"

#include <math.h>

namespace softapp
{
namespace mpu6050
{
namespace
{

constexpr uint8_t SmplrtDivReg = 0x19;
constexpr uint8_t ConfigReg = 0x1A;
constexpr uint8_t GyroConfigReg = 0x1B;
constexpr uint8_t AccelConfigReg = 0x1C;
constexpr uint8_t AccelXoutHReg = 0x3B;
constexpr uint8_t PwrMgmt1Reg = 0x6B;
constexpr uint8_t WhoAmIReg = 0x75;

constexpr uint8_t DeviceResetBit = 0x80;
constexpr uint8_t ClockPllGyroX = 0x01;
constexpr uint8_t ExpectedWhoAmI = 0x68;
constexpr float Pi = 3.14159265358979323846f;
constexpr float RadToDeg = 180.0f / Pi;

int16_t readInt16(const uint8_t *data)
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(data[0]) << 8) |
        static_cast<uint16_t>(data[1])
    );
}

float accelScale(AccelRange range)
{
    switch (range)
    {
        case AccelRange::G2:
            return 1.0f / 16384.0f;
        case AccelRange::G4:
            return 1.0f / 8192.0f;
        case AccelRange::G8:
            return 1.0f / 4096.0f;
        case AccelRange::G16:
            return 1.0f / 2048.0f;
    }
    return 1.0f / 8192.0f;
}

float gyroScale(GyroRange range)
{
    switch (range)
    {
        case GyroRange::Dps250:
            return 1.0f / 131.0f;
        case GyroRange::Dps500:
            return 1.0f / 65.5f;
        case GyroRange::Dps1000:
            return 1.0f / 32.8f;
        case GyroRange::Dps2000:
            return 1.0f / 16.4f;
    }
    return 1.0f / 65.5f;
}

float wrapDegrees(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

} // namespace

Result MPU6050_IIC::begin(iic::Bus &bus, const Config &config)
{
    mReady = false;
    mBus = nullptr;
    mConfig = config;
    mAttitudeInitialized = false;
    mLastSampleUs = 0;
    mRollOffsetDeg = 0.0f;
    mPitchOffsetDeg = 0.0f;
    mRollDeg = 0.0f;
    mPitchDeg = 0.0f;
    mYawDeg = 0.0f;

    if (!bus.isReady() ||
        (config.address != kAddressAd0Low &&
         config.address != kAddressAd0High) ||
        config.sampleRateHz < 4 ||
        config.sampleRateHz > 1000 ||
        config.digitalLowPassFilter > 6 ||
        config.complementaryAlpha < 0.0f ||
        config.complementaryAlpha > 1.0f)
    {
        return Result::InvalidConfig;
    }

    if (bus.probe(config.address) != iic::Result::Ok)
    {
        return Result::BusError;
    }

    mBus = &bus;

    uint8_t identity = 0;
    if (readRegisters(WhoAmIReg, &identity, 1) != Result::Ok)
    {
        mBus = nullptr;
        return Result::BusError;
    }

    if (identity != ExpectedWhoAmI)
    {
        mBus = nullptr;
        return Result::DeviceMismatch;
    }

    const Result configureResult = configureDevice();
    if (configureResult != Result::Ok)
    {
        mBus = nullptr;
        return configureResult;
    }

    mReady = true;
    return Result::Ok;
}

bool MPU6050_IIC::isReady() const
{
    return mReady;
}

Result MPU6050_IIC::read(Data &data)
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    uint8_t raw[14] = {};
    if (readRegisters(AccelXoutHReg, raw, sizeof(raw)) != Result::Ok)
    {
        return Result::BusError;
    }

    for (uint8_t i = 0; i < 3; ++i)
    {
        data.rawAccel[i] = readInt16(&raw[i * 2]);
        data.rawGyro[i] = readInt16(&raw[8 + i * 2]);
    }
    data.rawTemperature = readInt16(&raw[6]);

    const float aScale = accelScale(mConfig.accelRange);
    const float gScale = gyroScale(mConfig.gyroRange);
    for (uint8_t i = 0; i < 3; ++i)
    {
        data.accelG[i] = static_cast<float>(data.rawAccel[i]) * aScale;
        data.gyroDps[i] =
            static_cast<float>(data.rawGyro[i]) * gScale -
            mGyroBiasDps[i];
    }

    if (fabsf(data.gyroDps[2]) < mConfig.yawGyroDeadbandDps)
    {
        data.gyroDps[2] = 0.0f;
    }

    data.temperatureC =
        static_cast<float>(data.rawTemperature) / 340.0f + 36.53f;

    const float accelRoll =
        atan2f(data.accelG[1], data.accelG[2]) * RadToDeg;
    const float accelPitch =
        atan2f(
            -data.accelG[0],
            sqrtf(
                data.accelG[1] * data.accelG[1] +
                data.accelG[2] * data.accelG[2]
            )
        ) * RadToDeg;
    const float levelRoll = accelRoll - mRollOffsetDeg;
    const float levelPitch = accelPitch - mPitchOffsetDeg;

    const uint32_t nowUs = micros();
    if (!mAttitudeInitialized)
    {
        mRollDeg = levelRoll;
        mPitchDeg = levelPitch;
        mYawDeg = 0.0f;
        data.rollDeg = mRollDeg;
        data.pitchDeg = mPitchDeg;
        data.yawDeg = mYawDeg;
        mAttitudeInitialized = true;
        mLastSampleUs = nowUs;
        return Result::Ok;
    }

    float dt = static_cast<float>(
        static_cast<uint32_t>(nowUs - mLastSampleUs)
    ) / 1000000.0f;
    mLastSampleUs = nowUs;

    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f;
    }

    mRollDeg = mConfig.complementaryAlpha *
        (mRollDeg + data.gyroDps[0] * dt) +
        (1.0f - mConfig.complementaryAlpha) * levelRoll;
    mPitchDeg = mConfig.complementaryAlpha *
        (mPitchDeg + data.gyroDps[1] * dt) +
        (1.0f - mConfig.complementaryAlpha) * levelPitch;
    mYawDeg = wrapDegrees(mYawDeg + data.gyroDps[2] * dt);

    data.rollDeg = mRollDeg;
    data.pitchDeg = mPitchDeg;
    data.yawDeg = mYawDeg;
    return Result::Ok;
}

//校准
Result MPU6050_IIC::calibrateGyro(uint16_t samples)
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }
    if (samples == 0)
    {
        return Result::InvalidConfig;
    }

    const float scale = gyroScale(mConfig.gyroRange);
    float sum[3] = {};

    for (uint16_t i = 0; i < samples; ++i)
    {
        uint8_t raw[6] = {};
        if (readRegisters(0x43, raw, sizeof(raw)) != Result::Ok)
        {
            return Result::BusError;
        }

        for (uint8_t axis = 0; axis < 3; ++axis)
        {
            sum[axis] +=
                static_cast<float>(readInt16(&raw[axis * 2])) * scale;
        }
        delay(2);
    }

    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        mGyroBiasDps[axis] = sum[axis] / static_cast<float>(samples);
    }

    resetAttitude();
    return Result::Ok;
}

Result MPU6050_IIC::calibrateLevelAndGyro(uint16_t samples)
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }
    if (samples == 0)
    {
        return Result::InvalidConfig;
    }

    const float aScale = accelScale(mConfig.accelRange);
    const float gScale = gyroScale(mConfig.gyroRange);
    float gyroSum[3] = {};
    float rollSum = 0.0f;
    float pitchSum = 0.0f;

    for (uint16_t i = 0; i < samples; ++i)
    {
        uint8_t raw[14] = {};
        if (readRegisters(AccelXoutHReg, raw, sizeof(raw)) != Result::Ok)
        {
            return Result::BusError;
        }

        float accel[3] = {};
        for (uint8_t axis = 0; axis < 3; ++axis)
        {
            accel[axis] =
                static_cast<float>(readInt16(&raw[axis * 2])) * aScale;
            gyroSum[axis] +=
                static_cast<float>(readInt16(&raw[8 + axis * 2])) * gScale;
        }

        rollSum += atan2f(accel[1], accel[2]) * RadToDeg;
        pitchSum += atan2f(
            -accel[0],
            sqrtf(accel[1] * accel[1] + accel[2] * accel[2])
        ) * RadToDeg;
        delay(2);
    }

    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        mGyroBiasDps[axis] =
            gyroSum[axis] / static_cast<float>(samples);
    }
    mRollOffsetDeg = rollSum / static_cast<float>(samples);
    mPitchOffsetDeg = pitchSum / static_cast<float>(samples);

    resetAttitude();
    return Result::Ok;
}

void MPU6050_IIC::resetAttitude()
{
    mAttitudeInitialized = false;
    mLastSampleUs = 0;
    mRollDeg = 0.0f;
    mPitchDeg = 0.0f;
    mYawDeg = 0.0f;
}

const Config &MPU6050_IIC::config() const
{
    return mConfig;
}

const float *MPU6050_IIC::gyroBiasDps() const
{
    return mGyroBiasDps;
}

uint8_t MPU6050_IIC::whoAmI()
{
    uint8_t identity = 0;
    if (!mReady || readRegisters(WhoAmIReg, &identity, 1) != Result::Ok)
    {
        return 0;
    }
    return identity;
}

Result MPU6050_IIC::readRegisters(
    uint8_t reg,
    uint8_t *data,
    size_t length
)
{
    if (mBus == nullptr)
    {
        return Result::NotInitialized;
    }

    return mBus->readRegisters(mConfig.address, reg, data, length) ==
        iic::Result::Ok
        ? Result::Ok
        : Result::BusError;
}

Result MPU6050_IIC::writeRegister(uint8_t reg, uint8_t value)
{
    if (mBus == nullptr)
    {
        return Result::NotInitialized;
    }

    return mBus->writeRegisters(mConfig.address, reg, &value, 1) ==
        iic::Result::Ok
        ? Result::Ok
        : Result::BusError;
}

Result MPU6050_IIC::configureDevice()
{
    // begin() 在配置完成前尚未置 Ready，因此这里直接通过 Bus 访问。
    if (mBus == nullptr)
    {
        return Result::NotInitialized;
    }

    const uint8_t reset = DeviceResetBit;
    if (mBus->writeRegisters(
            mConfig.address,
            PwrMgmt1Reg,
            &reset,
            1) != iic::Result::Ok)
    {
        return Result::BusError;
    }
    delay(100);

    const uint8_t clockSource = ClockPllGyroX;
    if (mBus->writeRegisters(
            mConfig.address,
            PwrMgmt1Reg,
            &clockSource,
            1) != iic::Result::Ok)
    {
        return Result::BusError;
    }
    delay(10);

    const uint16_t divider16 =
        1000 / mConfig.sampleRateHz;
    const uint8_t sampleDiv = static_cast<uint8_t>(
        divider16 == 0 ? 0 : divider16 - 1
    );

    const uint8_t settings[] = {
        mConfig.digitalLowPassFilter,
        sampleDiv,
        static_cast<uint8_t>(
            static_cast<uint8_t>(mConfig.gyroRange) << 3
        ),
        static_cast<uint8_t>(
            static_cast<uint8_t>(mConfig.accelRange) << 3
        ),
    };
    const uint8_t registers[] = {
        ConfigReg,
        SmplrtDivReg,
        GyroConfigReg,
        AccelConfigReg,
    };

    for (uint8_t i = 0; i < sizeof(settings); ++i)
    {
        if (mBus->writeRegisters(
                mConfig.address,
                registers[i],
                &settings[i],
                1) != iic::Result::Ok)
        {
            return Result::BusError;
        }
    }

    return Result::Ok;
}

const char *toString(Result result)
{
    switch (result)
    {
        case Result::Ok:
            return "ok";
        case Result::InvalidConfig:
            return "invalid config";
        case Result::NotInitialized:
            return "not initialized";
        case Result::BusError:
            return "I2C bus error";
        case Result::DeviceMismatch:
            return "MPU6050 WHO_AM_I mismatch";
    }

    return "unknown MPU6050 error";
}

} // namespace mpu6050
} // namespace softapp
