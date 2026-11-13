#include "motor.h"

namespace hardware
{
namespace motor
{
namespace
{

int16_t clampSpeed(int16_t value)
{
    if (value < kStopSpeed)
    {
        return kStopSpeed;
    }
    if (value > kMaxSpeed)
    {
        return kMaxSpeed;
    }
    return value;
}

bool containsChannel(
    const Config &config,
    uint8_t channel,
    uint8_t count
)
{
    for (uint8_t i = 0; i < count; ++i)
    {
        if (config.channels[i] == channel)
        {
            return true;
        }
    }
    return false;
}

bool containsPin(const Config &config, int pin, uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i)
    {
        if (config.pins[i] == pin)
        {
            return true;
        }
    }
    return false;
}

} // namespace

bool MotorController::begin(const Config &config)
{
    mReady = false;
    mConfig = config;
    mMaxDuty = 0;
    mCommand = motor{};

    if (config.pwmFrequencyHz == 0 ||
        config.pwmResolutionBits == 0 ||
        config.pwmResolutionBits > 16)
    {
        return false;
    }

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        if (config.pins[i] < 0 ||
            containsPin(config, config.pins[i], i) ||
            containsChannel(config, config.channels[i], i))
        {
            return false;
        }
    }

    const uint32_t maxDuty =
        (1UL << config.pwmResolutionBits) - 1UL;
    mMaxDuty = static_cast<uint16_t>(maxDuty);

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        if (ledcSetup(
                config.channels[i],
                config.pwmFrequencyHz,
                config.pwmResolutionBits) == 0)
        {
            end();
            return false;
        }

        ledcAttachPin(config.pins[i], config.channels[i]);
        ledcWrite(
            config.channels[i],
            config.activeLow ? mMaxDuty : 0
        );
    }

    mReady = true;
    stopAll();
    return true;
}

void MotorController::end()
{
    if (mReady)
    {
        stopAll();
    }

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        ledcDetachPin(mConfig.pins[i]);
        ledcWrite(
            mConfig.channels[i],
            mConfig.activeLow ? mMaxDuty : 0
        );
    }

    mReady = false;
}

bool MotorController::isReady() const
{
    return mReady;
}

void MotorController::setSpeed(uint8_t index, int16_t value)
{
    if (!mReady || index >= kMotorCount)
    {
        return;
    }

    const int16_t limited = clampSpeed(value);
    mCommand.speed[index] = limited;
    applySpeed(index, limited);
}

void MotorController::setSpeeds(const motor &command)
{
    if (!mReady)
    {
        return;
    }

    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        setSpeed(i, command.speed[i]);
    }
}

void MotorController::stopAll()
{
    for (uint8_t i = 0; i < kMotorCount; ++i)
    {
        setSpeed(i, kStopSpeed);
    }
}

int16_t MotorController::speed(uint8_t index) const
{
    return index < kMotorCount ? mCommand.speed[index] : 0;
}

const motor &MotorController::command() const
{
    return mCommand;
}

void MotorController::applySpeed(uint8_t index, int16_t value)
{
    const uint16_t duty = speedToDuty(value);
    ledcWrite(
        mConfig.channels[index],
        mConfig.activeLow
            ? static_cast<uint16_t>(mMaxDuty - duty)
            : duty
    );
}

uint16_t MotorController::speedToDuty(int16_t value) const
{
    const int16_t limited = clampSpeed(value);
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(limited) * mMaxDuty) /
        static_cast<uint32_t>(kMaxSpeed)
    );
}

} // namespace motor
} // namespace hardware
