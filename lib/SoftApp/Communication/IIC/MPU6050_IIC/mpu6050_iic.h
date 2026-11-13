#ifndef MPU6050_IIC_H
#define MPU6050_IIC_H

#include "../iic.h"

namespace softapp
{
namespace mpu6050
{

constexpr uint8_t kAddressAd0Low = 0x68;
constexpr uint8_t kAddressAd0High = 0x69;

enum class Result : uint8_t
{
    Ok = 0,
    InvalidConfig,
    NotInitialized,
    BusError,
    DeviceMismatch,
};

enum class AccelRange : uint8_t
{
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
};

enum class GyroRange : uint8_t
{
    Dps250 = 0,
    Dps500 = 1,
    Dps1000 = 2,
    Dps2000 = 3,
};

struct Config
{
    uint8_t address = kAddressAd0Low;
    AccelRange accelRange = AccelRange::G4;
    GyroRange gyroRange = GyroRange::Dps500;
    uint16_t sampleRateHz = 100;
    uint8_t digitalLowPassFilter = 3;
    float complementaryAlpha = 0.98f;
    float yawGyroDeadbandDps = 0.2f;
};

struct Data
{
    int16_t rawAccel[3] = {};
    int16_t rawGyro[3] = {};
    int16_t rawTemperature = 0;

    float accelG[3] = {};
    float gyroDps[3] = {};
    float temperatureC = 0.0f;

    // Roll/Pitch 使用加速度计和陀螺仪互补滤波。
    // Yaw 只积分陀螺仪，没有磁力计参考，会随时间漂移。
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 0.0f;
};

class MPU6050_IIC
{
public:
    Result begin(iic::Bus &bus, const Config &config);
    bool isReady() const;

    Result read(Data &data);
    Result calibrateGyro(uint16_t samples = 2000);
    Result calibrateLevelAndGyro(uint16_t samples = 1000);
    void resetAttitude();

    const Config &config() const;
    const float *gyroBiasDps() const;
    uint8_t whoAmI();

private:
    Result readRegisters(uint8_t reg, uint8_t *data, size_t length);
    Result writeRegister(uint8_t reg, uint8_t value);
    Result configureDevice();

    iic::Bus *mBus = nullptr;
    Config mConfig{};
    bool mReady = false;
    bool mAttitudeInitialized = false;
    uint32_t mLastSampleUs = 0;
    float mGyroBiasDps[3] = {};
    float mRollOffsetDeg = 0.0f;
    float mPitchOffsetDeg = 0.0f;
    float mQ0 = 1.0f;
    float mQ1 = 0.0f;
    float mQ2 = 0.0f;
    float mQ3 = 0.0f;
    float mExInt = 0.0f;
    float mEyInt = 0.0f;
    float mEzInt = 0.0f;
    uint8_t mFusionDivider = 0;
    float mFusionDt = 0.0f;
    float mRollDeg = 0.0f;
    float mPitchDeg = 0.0f;
    float mYawDeg = 0.0f;
};

const char *toString(Result result);

} // namespace mpu6050
} // namespace softapp

#endif
