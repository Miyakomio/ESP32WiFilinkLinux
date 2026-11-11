#include "RC522_SPI.h"

#include <string.h>

namespace rc522
{
namespace
{

constexpr uint8_t CommandReg = 0x01;
constexpr uint8_t ComIEnReg = 0x02;
constexpr uint8_t DivIEnReg = 0x03;
constexpr uint8_t ComIrqReg = 0x04;
constexpr uint8_t DivIrqReg = 0x05;
constexpr uint8_t ErrorReg = 0x06;
constexpr uint8_t Status2Reg = 0x08;
constexpr uint8_t FIFODataReg = 0x09;
constexpr uint8_t FIFOLevelReg = 0x0A;
constexpr uint8_t ControlReg = 0x0C;
constexpr uint8_t BitFramingReg = 0x0D;
constexpr uint8_t CollReg = 0x0E;
constexpr uint8_t ModeReg = 0x11;
constexpr uint8_t TxControlReg = 0x14;
constexpr uint8_t TxAutoReg = 0x15;
constexpr uint8_t CRCResultRegH = 0x21;
constexpr uint8_t CRCResultRegL = 0x22;
constexpr uint8_t TModeReg = 0x2A;
constexpr uint8_t TPrescalerReg = 0x2B;
constexpr uint8_t TReloadRegH = 0x2C;
constexpr uint8_t TReloadRegL = 0x2D;
constexpr uint8_t VersionReg = 0x37;

constexpr uint8_t PcdIdle = 0x00;
constexpr uint8_t PcdAuthent = 0x0E;
constexpr uint8_t PcdTransceive = 0x0C;
constexpr uint8_t PcdResetPhase = 0x0F;
constexpr uint8_t PcdCalcCrc = 0x03;

constexpr uint8_t PiccAnticoll1 = 0x93;
constexpr uint8_t PiccRead = 0x30;
constexpr uint8_t PiccWrite = 0xA0;
constexpr uint8_t PiccHalt = 0x50;

constexpr uint8_t TimerIrq = 0x01;
constexpr uint8_t IdleIrq = 0x10;
constexpr uint8_t RxIrq = 0x20;

constexpr uint8_t ErrorProtocol = 0x02;
constexpr uint8_t ErrorParity = 0x01;
constexpr uint8_t ErrorCrc = 0x04;
constexpr uint8_t ErrorCollision = 0x08;
constexpr uint8_t ErrorBufferOverflow = 0x10;
constexpr uint8_t ErrorTemperature = 0x40;
constexpr uint8_t ErrorWrite = 0x80;

constexpr uint8_t Crypto1On = 0x08;
constexpr uint8_t AckNibble = 0x0A;
constexpr uint8_t MaxFifoBytes = 64;

bool hasBit(uint8_t value, uint8_t mask)
{
    return (value & mask) != 0;
}

} // namespace

bool RC522_SPI::begin(SPIClass &spi, const Config &config)
{
    mReady = false;
    mSpi = nullptr;
    mConfig = config;

    if (config.csPin < 0 ||
        config.spiClockHz == 0 ||
        config.timeoutUs == 0)
    {
        return false;
    }

    pinMode(config.csPin, OUTPUT);
    digitalWrite(config.csPin, HIGH);

    if (config.resetPin >= 0)
    {
        pinMode(config.resetPin, OUTPUT);
        digitalWrite(config.resetPin, LOW);
    }

    if (config.initializeSpiBus)
    {
        spi.begin(
            config.sckPin,
            config.misoPin,
            config.mosiPin,
            config.csPin
        );
    }

    mSpi = &spi;

    if (!resetChip())
    {
        end();
        return false;
    }

    const uint8_t chipVersion = readRegister(VersionReg);
    if (chipVersion == 0x00 || chipVersion == 0xFF)
    {
        end();
        return false;
    }

    mReady = true;
    if (setAntenna(true) != Result::Ok)
    {
        end();
        return false;
    }

    return true;
}

void RC522_SPI::end()
{
    if (mReady)
    {
        setAntenna(false);
    }

    if (mSpi != nullptr && mConfig.csPin >= 0)
    {
        pinMode(mConfig.csPin, OUTPUT);
        digitalWrite(mConfig.csPin, HIGH);
    }

    if (mConfig.resetPin >= 0)
    {
        digitalWrite(mConfig.resetPin, LOW);
    }

    mReady = false;
}

bool RC522_SPI::isReady() const
{
    return mReady;
}

uint8_t RC522_SPI::version()
{
    if (!mReady)
    {
        return 0;
    }
    return readRegister(VersionReg);
}

Result RC522_SPI::setAntenna(bool enabled)
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    const uint8_t before = readRegister(TxControlReg);
    const uint8_t after = enabled
        ? static_cast<uint8_t>(before | 0x03)
        : static_cast<uint8_t>(before & static_cast<uint8_t>(~0x03));

    if (before != after)
    {
        writeRegister(TxControlReg, after);
        delay(1);
    }

    return Result::Ok;
}

Result RC522_SPI::request(RequestMode mode, uint16_t &atqa)
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    if (mode != RequestMode::Idle && mode != RequestMode::All)
    {
        return Result::InvalidArgument;
    }

    stopCrypto1();
    writeRegister(BitFramingReg, 0x07);

    const uint8_t tx = static_cast<uint8_t>(mode);
    uint8_t rx[2] = {};
    uint16_t receivedBits = 0;
    const Result result = transceive(
        PcdTransceive,
        &tx,
        1,
        rx,
        sizeof(rx),
        receivedBits
    );

    if (result != Result::Ok)
    {
        return result;
    }

    if (receivedBits != 16)
    {
        return Result::BadResponse;
    }

    atqa = static_cast<uint16_t>(
        (static_cast<uint16_t>(rx[0]) << 8) |
        static_cast<uint16_t>(rx[1])
    );
    return Result::Ok;
}

Result RC522_SPI::anticollision(uint8_t uid[4])
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    if (uid == nullptr)
    {
        return Result::InvalidArgument;
    }

    clearBitMask(Status2Reg, Crypto1On);
    writeRegister(BitFramingReg, 0x00);
    clearBitMask(CollReg, 0x80);

    const uint8_t tx[2] = {PiccAnticoll1, 0x20};
    uint8_t rx[5] = {};
    uint16_t receivedBits = 0;
    const Result result = transceive(
        PcdTransceive,
        tx,
        sizeof(tx),
        rx,
        sizeof(rx),
        receivedBits
    );

    setBitMask(CollReg, 0x80);

    if (result != Result::Ok)
    {
        return result;
    }

    if (receivedBits != 40)
    {
        return Result::BadResponse;
    }

    uint8_t bcc = 0;
    for (uint8_t i = 0; i < 4; ++i)
    {
        bcc ^= rx[i];
    }

    if (bcc != rx[4])
    {
        return Result::BadResponse;
    }

    memcpy(uid, rx, 4);
    return Result::Ok;
}

Result RC522_SPI::select(const uint8_t uid[4], uint8_t &sak)
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    if (uid == nullptr)
    {
        return Result::InvalidArgument;
    }

    uint8_t tx[9] = {};
    tx[0] = PiccAnticoll1;
    tx[1] = 0x70;

    uint8_t bcc = 0;
    for (uint8_t i = 0; i < 4; ++i)
    {
        tx[i + 2] = uid[i];
        bcc ^= uid[i];
    }
    tx[6] = bcc;

    if (!calculateCrc(tx, 7, &tx[7]))
    {
        return Result::HardwareError;
    }

    clearBitMask(Status2Reg, Crypto1On);

    uint8_t rx[3] = {};
    uint16_t receivedBits = 0;
    const Result result = transceive(
        PcdTransceive,
        tx,
        sizeof(tx),
        rx,
        sizeof(rx),
        receivedBits
    );

    if (result != Result::Ok)
    {
        return result;
    }

    if (receivedBits != 24)
    {
        return Result::BadResponse;
    }

    sak = rx[0];
    return Result::Ok;
}

Result RC522_SPI::readCard(
    uint8_t uid[4],
    uint16_t &atqa,
    uint8_t &sak,
    RequestMode mode
)
{
    if (uid == nullptr)
    {
        return Result::InvalidArgument;
    }

    Result result = request(mode, atqa);
    if (result != Result::Ok)
    {
        return result;
    }

    result = anticollision(uid);
    if (result != Result::Ok)
    {
        return result;
    }

    return select(uid, sak);
}

Result RC522_SPI::authenticate(
    uint8_t block,
    KeyType keyType,
    const uint8_t key[6],
    const uint8_t uid[4]
)
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    if (block > 63 || key == nullptr || uid == nullptr)
    {
        return Result::InvalidArgument;
    }

    uint8_t tx[12] = {};
    tx[0] = static_cast<uint8_t>(keyType);
    tx[1] = block;
    memcpy(&tx[2], key, 6);
    memcpy(&tx[8], uid, 4);

    uint16_t receivedBits = 0;
    const Result result = transceive(
        PcdAuthent,
        tx,
        sizeof(tx),
        nullptr,
        0,
        receivedBits
    );

    if (result == Result::NoCard || result == Result::Timeout)
    {
        return Result::AuthenticationFailed;
    }

    if (result != Result::Ok)
    {
        return result;
    }

    if (!hasBit(readRegister(Status2Reg), Crypto1On))
    {
        return Result::AuthenticationFailed;
    }

    return Result::Ok;
}

Result RC522_SPI::readBlock(uint8_t block, uint8_t data[16])
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    if (block > 63 || data == nullptr)
    {
        return Result::InvalidArgument;
    }

    uint8_t tx[4] = {PiccRead, block, 0, 0};
    if (!calculateCrc(tx, 2, &tx[2]))
    {
        return Result::HardwareError;
    }

    uint8_t rx[18] = {};
    uint16_t receivedBits = 0;
    const Result result = transceive(
        PcdTransceive,
        tx,
        sizeof(tx),
        rx,
        sizeof(rx),
        receivedBits
    );

    if (result != Result::Ok)
    {
        return result;
    }

    // M1 读块返回 16 byte 数据和 2 byte CRC。
    if (receivedBits != 144)
    {
        return Result::BadResponse;
    }

    memcpy(data, rx, 16);
    return Result::Ok;
}

Result RC522_SPI::writeBlock(uint8_t block, const uint8_t data[16])
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    if (block > 63 || data == nullptr)
    {
        return Result::InvalidArgument;
    }

    uint8_t command[4] = {PiccWrite, block, 0, 0};
    if (!calculateCrc(command, 2, &command[2]))
    {
        return Result::HardwareError;
    }

    uint8_t ack[2] = {};
    uint16_t receivedBits = 0;
    Result result = transceive(
        PcdTransceive,
        command,
        sizeof(command),
        ack,
        sizeof(ack),
        receivedBits
    );

    if (result != Result::Ok ||
        receivedBits != 4 ||
        (ack[0] & 0x0F) != AckNibble)
    {
        return result == Result::Ok ? Result::BadResponse : result;
    }

    uint8_t frame[18] = {};
    memcpy(frame, data, 16);
    if (!calculateCrc(frame, 16, &frame[16]))
    {
        return Result::HardwareError;
    }

    memset(ack, 0, sizeof(ack));
    receivedBits = 0;
    result = transceive(
        PcdTransceive,
        frame,
        sizeof(frame),
        ack,
        sizeof(ack),
        receivedBits
    );

    if (result != Result::Ok ||
        receivedBits != 4 ||
        (ack[0] & 0x0F) != AckNibble)
    {
        return result == Result::Ok ? Result::BadResponse : result;
    }

    return Result::Ok;
}

Result RC522_SPI::halt()
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    uint8_t tx[4] = {PiccHalt, 0, 0, 0};
    if (!calculateCrc(tx, 2, &tx[2]))
    {
        return Result::HardwareError;
    }

    uint16_t receivedBits = 0;
    const Result result = transceive(
        PcdTransceive,
        tx,
        sizeof(tx),
        nullptr,
        0,
        receivedBits
    );

    stopCrypto1();

    // HALT 后卡不再回复，因此无卡或等待超时都表示命令已正常发出。
    if (result == Result::NoCard || result == Result::Timeout)
    {
        return Result::Ok;
    }

    return result;
}

void RC522_SPI::stopCrypto1()
{
    if (mReady)
    {
        clearBitMask(Status2Reg, Crypto1On);
    }
}

CardType RC522_SPI::cardType(uint16_t atqa)
{
    switch (atqa)
    {
        case 0x4400:
            return CardType::MifareUltralight;
        case 0x0400:
            return CardType::MifareClassic1K;
        case 0x0200:
            return CardType::MifareClassic4K;
        case 0x0800:
            return CardType::MifarePlus;
        case 0x4403:
            return CardType::MifareDesfire;
        default:
            return CardType::Unknown;
    }
}

Result RC522_SPI::transceive(
    uint8_t command,
    const uint8_t *txData,
    uint8_t txLength,
    uint8_t *rxData,
    uint8_t rxCapacity,
    uint16_t &rxBitLength
)
{
    if (!mReady)
    {
        return Result::NotInitialized;
    }

    if ((txLength > 0 && txData == nullptr) ||
        txLength > MaxFifoBytes ||
        (rxCapacity > 0 && rxData == nullptr) ||
        (command != PcdAuthent && command != PcdTransceive))
    {
        return Result::InvalidArgument;
    }

    rxBitLength = 0;

    uint8_t irqEnable = 0x00;
    uint8_t waitIrq = 0x00;

    if (command == PcdAuthent)
    {
        irqEnable = 0x12;
        waitIrq = IdleIrq;
    }
    else
    {
        irqEnable = 0x77;
        waitIrq = static_cast<uint8_t>(RxIrq | IdleIrq);
    }

    writeRegister(ComIEnReg, static_cast<uint8_t>(irqEnable | 0x80));
    writeRegister(ComIrqReg, 0x7F);
    writeRegister(CommandReg, PcdIdle);
    writeRegister(FIFOLevelReg, 0x80);

    for (uint8_t i = 0; i < txLength; ++i)
    {
        writeRegister(FIFODataReg, txData[i]);
    }

    writeRegister(CommandReg, command);
    if (command == PcdTransceive)
    {
        setBitMask(BitFramingReg, 0x80);
    }

    uint8_t irq = 0;
    const uint32_t startedAt = micros();
    bool completed = false;

    while (static_cast<uint32_t>(micros() - startedAt) <
           mConfig.timeoutUs)
    {
        irq = readRegister(ComIrqReg);
        if (hasBit(irq, waitIrq) || hasBit(irq, TimerIrq))
        {
            completed = true;
            break;
        }
        delayMicroseconds(1);
    }

    clearBitMask(BitFramingReg, 0x80);

    if (!completed)
    {
        writeRegister(CommandReg, PcdIdle);
        return Result::Timeout;
    }

    if (hasBit(irq, TimerIrq))
    {
        writeRegister(CommandReg, PcdIdle);
        return Result::NoCard;
    }

    const uint8_t error = readRegister(ErrorReg);
    if (error != 0)
    {
        setBitMask(ControlReg, 0x80);
        writeRegister(CommandReg, PcdIdle);
        return mapError(error);
    }

    if (command == PcdTransceive && rxCapacity > 0)
    {
        const uint8_t fifoLength = readRegister(FIFOLevelReg);
        const uint8_t lastBits = readRegister(ControlReg) & 0x07;

        if (fifoLength > rxCapacity)
        {
            setBitMask(ControlReg, 0x80);
            writeRegister(CommandReg, PcdIdle);
            return Result::BufferOverflow;
        }

        rxBitLength = fifoLength == 0
            ? 0
            : static_cast<uint16_t>(
                (static_cast<uint16_t>(fifoLength - 1) * 8) +
                (lastBits == 0 ? 8 : lastBits)
            );

        for (uint8_t i = 0; i < fifoLength; ++i)
        {
            rxData[i] = readRegister(FIFODataReg);
        }
    }

    setBitMask(ControlReg, 0x80);
    writeRegister(CommandReg, PcdIdle);
    return Result::Ok;
}

bool RC522_SPI::calculateCrc(
    const uint8_t *data,
    uint8_t length,
    uint8_t crc[2]
)
{
    if (!mReady || data == nullptr || crc == nullptr ||
        length > MaxFifoBytes)
    {
        return false;
    }

    clearBitMask(DivIrqReg, 0x04);
    writeRegister(CommandReg, PcdIdle);
    writeRegister(FIFOLevelReg, 0x80);

    for (uint8_t i = 0; i < length; ++i)
    {
        writeRegister(FIFODataReg, data[i]);
    }

    writeRegister(CommandReg, PcdCalcCrc);

    bool completed = false;
    const uint32_t startedAt = micros();
    while (static_cast<uint32_t>(micros() - startedAt) <
           mConfig.timeoutUs)
    {
        if (hasBit(readRegister(DivIrqReg), 0x04))
        {
            completed = true;
            break;
        }
        delayMicroseconds(1);
    }

    if (completed)
    {
        crc[0] = readRegister(CRCResultRegL);
        crc[1] = readRegister(CRCResultRegH);
    }

    writeRegister(CommandReg, PcdIdle);
    return completed;
}

uint8_t RC522_SPI::readRegister(uint8_t reg)
{
    if (mSpi == nullptr)
    {
        return 0;
    }

    mSpi->beginTransaction(
        SPISettings(mConfig.spiClockHz, MSBFIRST, SPI_MODE0)
    );
    digitalWrite(mConfig.csPin, LOW);
    mSpi->transfer(static_cast<uint8_t>(((reg << 1) & 0x7E) | 0x80));
    const uint8_t value = mSpi->transfer(0x00);
    digitalWrite(mConfig.csPin, HIGH);
    mSpi->endTransaction();
    return value;
}

void RC522_SPI::writeRegister(uint8_t reg, uint8_t value)
{
    if (mSpi == nullptr)
    {
        return;
    }

    mSpi->beginTransaction(
        SPISettings(mConfig.spiClockHz, MSBFIRST, SPI_MODE0)
    );
    digitalWrite(mConfig.csPin, LOW);
    mSpi->transfer(static_cast<uint8_t>((reg << 1) & 0x7E));
    mSpi->transfer(value);
    digitalWrite(mConfig.csPin, HIGH);
    mSpi->endTransaction();
}

void RC522_SPI::setBitMask(uint8_t reg, uint8_t mask)
{
    writeRegister(reg, static_cast<uint8_t>(readRegister(reg) | mask));
}

void RC522_SPI::clearBitMask(uint8_t reg, uint8_t mask)
{
    writeRegister(
        reg,
        static_cast<uint8_t>(readRegister(reg) & static_cast<uint8_t>(~mask))
    );
}

bool RC522_SPI::resetChip()
{
    if (mSpi == nullptr)
    {
        return false;
    }

    if (mConfig.resetPin >= 0)
    {
        digitalWrite(mConfig.resetPin, LOW);
        delay(10);
        digitalWrite(mConfig.resetPin, HIGH);
        delay(50);
    }

    writeRegister(CommandReg, PcdResetPhase);
    delay(5);

    writeRegister(TModeReg, 0x8D);
    writeRegister(TPrescalerReg, 0x3E);
    writeRegister(TReloadRegH, 0x00);
    writeRegister(TReloadRegL, 30);
    writeRegister(TxAutoReg, 0x40);
    writeRegister(ModeReg, 0x3D);
    return true;
}

Result RC522_SPI::mapError(uint8_t errorReg)
{
    if (hasBit(errorReg, ErrorBufferOverflow))
    {
        return Result::BufferOverflow;
    }
    if (hasBit(errorReg, ErrorCollision))
    {
        return Result::Collision;
    }
    if (hasBit(errorReg, ErrorCrc))
    {
        return Result::CrcError;
    }
    if (hasBit(errorReg, ErrorParity))
    {
        return Result::ParityError;
    }
    if (hasBit(errorReg, ErrorProtocol))
    {
        return Result::ProtocolError;
    }
    if (hasBit(errorReg, ErrorTemperature) ||
        hasBit(errorReg, ErrorWrite))
    {
        return Result::HardwareError;
    }
    return Result::HardwareError;
}

const char *RC522_SPI::toString(Result result)
{
    switch (result)
    {
        case Result::Ok:
            return "ok";
        case Result::NotInitialized:
            return "not initialized";
        case Result::InvalidArgument:
            return "invalid argument";
        case Result::NoCard:
            return "no card";
        case Result::Timeout:
            return "timeout";
        case Result::Collision:
            return "collision";
        case Result::CrcError:
            return "crc error";
        case Result::ParityError:
            return "parity error";
        case Result::ProtocolError:
            return "protocol error";
        case Result::BufferOverflow:
            return "buffer overflow";
        case Result::HardwareError:
            return "hardware error";
        case Result::BadResponse:
            return "bad response";
        case Result::AuthenticationFailed:
            return "authentication failed";
    }

    return "unknown error";
}

} // namespace rc522
