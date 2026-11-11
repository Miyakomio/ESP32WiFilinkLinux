#ifndef SOFTAPP_IIC_H
#define SOFTAPP_IIC_H

#include <Arduino.h>
#include <Wire.h>

namespace softapp
{
namespace iic
{

enum class Result : uint8_t
{
    Ok = 0,
    InvalidArgument,
    NotInitialized,
    StartFailed,
    WriteFailed,
    ReadFailed,
    ShortRead,
};

struct Config
{
    int sdaPin = -1;
    int sclPin = -1;
    uint32_t clockHz = 100000;
    bool initializeBus = true;
};

class Bus
{
public:
    Result begin(TwoWire &wire, const Config &config);
    void end();

    bool isReady() const;
    Result probe(uint8_t deviceAddress);

    Result readRegisters(
        uint8_t deviceAddress,
        uint8_t startRegister,
        uint8_t *data,
        size_t length
    );

    Result writeRegisters(
        uint8_t deviceAddress,
        uint8_t startRegister,
        const uint8_t *data,
        size_t length
    );

    Result readRegister16(
        uint8_t deviceAddress,
        uint8_t reg,
        uint16_t &value
    );

    Result writeRegister16(
        uint8_t deviceAddress,
        uint8_t reg,
        uint16_t value
    );

private:
    TwoWire *mWire = nullptr;
    bool mReady = false;
    bool mOwnsBus = false;
};

const char *toString(Result result);

} // namespace iic
} // namespace softapp

#endif
