#ifndef HARDWARE_MAX1704_H
#define HARDWARE_MAX1704_H

#include <stdint.h>

namespace hardware
{
namespace max1704
{

constexpr uint8_t kDeviceAddress = 0x36;

enum class Model : uint8_t
{
    MAX17048 = 0,
    MAX17049,
};

enum class StatusFlag : uint16_t
{
    Reset = 0x0100,
    VoltageHigh = 0x0200,
    VoltageLow = 0x0400,
    VoltageReset = 0x0800,
    SocLow = 0x1000,
    SocChanged = 0x2000,
    VoltageResetAlertEnabled = 0x4000,
};

struct Reading
{
    Model model = Model::MAX17048;
    float cellVoltageMv = 0.0f;
    float stateOfChargePercent = 0.0f;
    float ratePercentPerHour = 0.0f;
    uint16_t rawVoltage = 0;
    uint16_t rawStateOfCharge = 0;
    uint16_t rawRate = 0;
    uint16_t status = 0;
    uint16_t version = 0;
};

} // namespace max1704
} // namespace hardware

#endif
