#ifndef MAX1704_IIC_H
#define MAX1704_IIC_H

#include "../iic.h"
#include "../../../../Hardware/MAX1704/MAX1704.h"

namespace softapp
{
namespace max1704
{

class MAX1704_IIC
{
public:
    iic::Result begin(
        iic::Bus &bus,
        hardware::max1704::Model model
    );

    bool isReady() const;
    hardware::max1704::Model model() const;

    iic::Result readMeasurement(hardware::max1704::Reading &reading);
    iic::Result readCellVoltage(uint16_t &raw);
    iic::Result readStateOfCharge(uint16_t &raw);
    iic::Result readRate(uint16_t &raw);
    iic::Result readVersion(uint16_t &version);
    iic::Result readStatus(uint16_t &status);

    iic::Result quickStart();
    iic::Result clearAlert();
    iic::Result setLowSocThreshold(uint8_t percent);

private:
    iic::Result readRegister(uint8_t reg, uint16_t &value);
    iic::Result writeRegister(uint8_t reg, uint16_t value);

    iic::Bus *mBus = nullptr;
    hardware::max1704::Model mModel =
        hardware::max1704::Model::MAX17048;
    bool mReady = false;
};

const char *toString(hardware::max1704::Model model);

} // namespace max1704
} // namespace softapp

#endif
