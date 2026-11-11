#include "MAX1704_IIC.h"

namespace softapp
{
namespace max1704
{
namespace
{

constexpr uint8_t VcellReg = 0x02;
constexpr uint8_t SocReg = 0x04;
constexpr uint8_t ModeReg = 0x06;
constexpr uint8_t VersionReg = 0x08;
constexpr uint8_t ConfigReg = 0x0C;
constexpr uint8_t CrateReg = 0x16;
constexpr uint8_t StatusReg = 0x1A;

constexpr uint16_t QuickStartCommand = 0x4000;
constexpr uint16_t ConfigAlertBit = 0x0020;
constexpr uint16_t ConfigAthdMask = 0x001F;
constexpr float VoltageLsbMv = 0.078125f;
constexpr float SocLsbPercent = 1.0f / 256.0f;
constexpr float RateLsbPercentPerHour = 0.208f;

} // namespace

iic::Result MAX1704_IIC::begin(
    iic::Bus &bus,
    hardware::max1704::Model model
)
{
    mReady = false;
    mBus = nullptr;
    mModel = model;

    if (!bus.isReady() ||
        (model != hardware::max1704::Model::MAX17048 &&
         model != hardware::max1704::Model::MAX17049))
    {
        return iic::Result::InvalidArgument;
    }

    const iic::Result result = bus.probe(hardware::max1704::kDeviceAddress);
    if (result != iic::Result::Ok)
    {
        return result;
    }

    mBus = &bus;
    mReady = true;
    return iic::Result::Ok;
}

bool MAX1704_IIC::isReady() const
{
    return mReady;
}

hardware::max1704::Model MAX1704_IIC::model() const
{
    return mModel;
}

iic::Result MAX1704_IIC::readMeasurement(hardware::max1704::Reading &reading)
{
    if (!mReady)
    {
        return iic::Result::NotInitialized;
    }

    hardware::max1704::Reading next;
    iic::Result result = readCellVoltage(next.rawVoltage);
    if (result != iic::Result::Ok)
    {
        return result;
    }

    result = readStateOfCharge(next.rawStateOfCharge);
    if (result != iic::Result::Ok)
    {
        return result;
    }

    result = readRate(next.rawRate);
    if (result != iic::Result::Ok)
    {
        return result;
    }

    result = readStatus(next.status);
    if (result != iic::Result::Ok)
    {
        return result;
    }

    result = readVersion(next.version);
    if (result != iic::Result::Ok)
    {
        return result;
    }

    next.model = mModel;
    next.cellVoltageMv =
        static_cast<float>(next.rawVoltage) * VoltageLsbMv;
    next.stateOfChargePercent =
        static_cast<float>(next.rawStateOfCharge) * SocLsbPercent;
    next.ratePercentPerHour =
        static_cast<float>(static_cast<int16_t>(next.rawRate)) *
        RateLsbPercentPerHour;

    reading = next;
    return iic::Result::Ok;
}

iic::Result MAX1704_IIC::readCellVoltage(uint16_t &raw)
{
    return readRegister(VcellReg, raw);
}

iic::Result MAX1704_IIC::readStateOfCharge(uint16_t &raw)
{
    return readRegister(SocReg, raw);
}

iic::Result MAX1704_IIC::readRate(uint16_t &raw)
{
    return readRegister(CrateReg, raw);
}

iic::Result MAX1704_IIC::readVersion(uint16_t &version)
{
    return readRegister(VersionReg, version);
}

iic::Result MAX1704_IIC::readStatus(uint16_t &status)
{
    return readRegister(StatusReg, status);
}

iic::Result MAX1704_IIC::quickStart()
{
    if (!mReady)
    {
        return iic::Result::NotInitialized;
    }

    return writeRegister(ModeReg, QuickStartCommand);
}

iic::Result MAX1704_IIC::clearAlert()
{
    if (!mReady)
    {
        return iic::Result::NotInitialized;
    }

    uint16_t config = 0;
    iic::Result result = readRegister(ConfigReg, config);
    if (result != iic::Result::Ok)
    {
        return result;
    }

    config = static_cast<uint16_t>(config & ~ConfigAlertBit);
    return writeRegister(ConfigReg, config);
}

iic::Result MAX1704_IIC::setLowSocThreshold(uint8_t percent)
{
    if (!mReady)
    {
        return iic::Result::NotInitialized;
    }

    if (percent == 0 || percent > 32)
    {
        return iic::Result::InvalidArgument;
    }

    uint16_t config = 0;
    iic::Result result = readRegister(ConfigReg, config);
    if (result != iic::Result::Ok)
    {
        return result;
    }

    const uint16_t athd =
        static_cast<uint16_t>(32 - percent) & ConfigAthdMask;
    config = static_cast<uint16_t>(
        (config & ~ConfigAthdMask) | athd
    );
    return writeRegister(ConfigReg, config);
}

iic::Result MAX1704_IIC::readRegister(uint8_t reg, uint16_t &value)
{
    if (!mReady || mBus == nullptr)
    {
        return iic::Result::NotInitialized;
    }

    return mBus->readRegister16(
        hardware::max1704::kDeviceAddress,
        reg,
        value
    );
}

iic::Result MAX1704_IIC::writeRegister(uint8_t reg, uint16_t value)
{
    if (!mReady || mBus == nullptr)
    {
        return iic::Result::NotInitialized;
    }

    return mBus->writeRegister16(
        hardware::max1704::kDeviceAddress,
        reg,
        value
    );
}

const char *toString(hardware::max1704::Model model)
{
    switch (model)
    {
        case hardware::max1704::Model::MAX17048:
            return "MAX17048";
        case hardware::max1704::Model::MAX17049:
            return "MAX17049";
    }

    return "unknown";
}

} // namespace max1704
} // namespace softapp
