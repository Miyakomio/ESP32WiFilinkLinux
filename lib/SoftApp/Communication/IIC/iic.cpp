#include "iic.h"

namespace softapp
{
namespace iic
{
namespace
{

bool isValidAddress(uint8_t address)
{
    return address >= 0x08 && address <= 0x77;
}

bool hasValidPins(const Config &config)
{
    return (config.sdaPin < 0) == (config.sclPin < 0);
}

} // namespace

Result Bus::begin(TwoWire &wire, const Config &config)
{
    mReady = false;
    mWire = nullptr;
    mOwnsBus = false;

    if (config.clockHz == 0 || !hasValidPins(config))
    {
        return Result::InvalidArgument;
    }

    mWire = &wire;

    if (config.initializeBus)
    {
        if (!wire.begin(config.sdaPin, config.sclPin, config.clockHz))
        {
            mWire = nullptr;
            return Result::StartFailed;
        }
        mOwnsBus = true;
    }
    else
    {
        wire.setClock(config.clockHz);
    }

    mReady = true;
    return Result::Ok;
}

void Bus::end()
{
    if (mOwnsBus && mWire != nullptr)
    {
        mWire->end();
    }

    mOwnsBus = false;
    mReady = false;
    mWire = nullptr;
}

bool Bus::isReady() const
{
    return mReady;
}

Result Bus::probe(uint8_t deviceAddress)
{
    if (!mReady || mWire == nullptr)
    {
        return Result::NotInitialized;
    }

    if (!isValidAddress(deviceAddress))
    {
        return Result::InvalidArgument;
    }

    mWire->beginTransmission(deviceAddress);
    return mWire->endTransmission() == 0
        ? Result::Ok
        : Result::WriteFailed;
}

Result Bus::readRegisters(
    uint8_t deviceAddress,
    uint8_t startRegister,
    uint8_t *data,
    size_t length
)
{
    if (!mReady || mWire == nullptr)
    {
        return Result::NotInitialized;
    }

    if (!isValidAddress(deviceAddress) ||
        data == nullptr ||
        length == 0)
    {
        return Result::InvalidArgument;
    }

    mWire->beginTransmission(deviceAddress);
    mWire->write(startRegister);
    if (mWire->endTransmission(false) != 0)
    {
        return Result::WriteFailed;
    }

    if (mWire->requestFrom(deviceAddress, length) != length)
    {
        return Result::ShortRead;
    }

    for (size_t i = 0; i < length; ++i)
    {
        if (!mWire->available())
        {
            return Result::ReadFailed;
        }
        data[i] = static_cast<uint8_t>(mWire->read());
    }

    return Result::Ok;
}

Result Bus::writeRegisters(
    uint8_t deviceAddress,
    uint8_t startRegister,
    const uint8_t *data,
    size_t length
)
{
    if (!mReady || mWire == nullptr)
    {
        return Result::NotInitialized;
    }

    if (!isValidAddress(deviceAddress) ||
        data == nullptr ||
        length == 0)
    {
        return Result::InvalidArgument;
    }

    mWire->beginTransmission(deviceAddress);
    mWire->write(startRegister);
    for (size_t i = 0; i < length; ++i)
    {
        mWire->write(data[i]);
    }

    return mWire->endTransmission() == 0
        ? Result::Ok
        : Result::WriteFailed;
}

Result Bus::readRegister16(
    uint8_t deviceAddress,
    uint8_t reg,
    uint16_t &value
)
{
    uint8_t data[2] = {};
    const Result result = readRegisters(
        deviceAddress,
        reg,
        data,
        sizeof(data)
    );

    if (result == Result::Ok)
    {
        value = static_cast<uint16_t>(
            (static_cast<uint16_t>(data[0]) << 8) |
            static_cast<uint16_t>(data[1])
        );
    }

    return result;
}

Result Bus::writeRegister16(
    uint8_t deviceAddress,
    uint8_t reg,
    uint16_t value
)
{
    const uint8_t data[2] = {
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF),
    };

    return writeRegisters(
        deviceAddress,
        reg,
        data,
        sizeof(data)
    );
}

const char *toString(Result result)
{
    switch (result)
    {
        case Result::Ok:
            return "ok";
        case Result::InvalidArgument:
            return "invalid argument";
        case Result::NotInitialized:
            return "not initialized";
        case Result::StartFailed:
            return "I2C start failed";
        case Result::WriteFailed:
            return "I2C write failed";
        case Result::ReadFailed:
            return "I2C read failed";
        case Result::ShortRead:
            return "I2C short read";
    }

    return "unknown I2C error";
}

} // namespace iic
} // namespace softapp
