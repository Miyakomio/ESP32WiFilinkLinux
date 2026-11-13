#ifndef HARDWARE_MOTOR_H
#define HARDWARE_MOTOR_H

#include <Arduino.h>

namespace hardware
{
namespace motor
{

constexpr uint8_t kMotorCount = 4;
constexpr int16_t kStopSpeed = 0;//停止输出
constexpr int16_t kMinSpeed = 100;//正转启动下限
constexpr int16_t kMaxSpeed = 1000;//电机转速上限
//motor引脚配置
struct Config
{
    int pins[kMotorCount] = {3, 4, 42, 45};
    uint8_t channels[kMotorCount] = {0, 1, 2, 3};
    uint32_t pwmFrequencyHz = 20000;//pwm
    uint8_t pwmResolutionBits = 10;
    bool activeLow = false;
};

struct motor
{
    uint8_t mid[kMotorCount] = {1, 2, 3, 4};//motor设备id
    int16_t speed[kMotorCount] = {};
};

using MotorHandler = void (*)(const motor &motor);

class MotorController
{
public:
    bool begin(const Config &config);
    void end();

    bool isReady() const;
    void setSpeed(uint8_t index, int16_t speed);//单电机控制
    void setSpeeds(const motor &command);//多电机控制
    void stopAll();

    int16_t speed(uint8_t index) const;//获取电机速度
    const motor &command() const;

private:
    void applySpeed(uint8_t index, int16_t speed);
    uint16_t speedToDuty(int16_t speed) const;

    Config mConfig{};
    motor mCommand{};
    uint16_t mMaxDuty = 0;
    bool mReady = false;
};

} // namespace motor
} // namespace hardware

#endif
