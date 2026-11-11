#ifndef RC522_SPI_H
#define RC522_SPI_H
#include <Arduino.h>
#include <SPI.h>



namespace rc522
{

enum class Result : uint8_t
{
    Ok = 0,
    NotInitialized,
    InvalidArgument,
    NoCard,
    Timeout,
    Collision,
    CrcError,
    ParityError,
    ProtocolError,
    BufferOverflow,
    HardwareError,
    BadResponse,
    AuthenticationFailed,
};

enum class RequestMode : uint8_t
{
    Idle = 0x26, // REQA：只响应空闲卡
    All = 0x52,  // WUPA：响应所有卡，包括休眠卡
};

enum class KeyType : uint8_t
{
    A = 0x60,
    B = 0x61,
};

enum class CardType : uint8_t
{
    Unknown = 0,
    MifareUltralight,
    MifareClassic1K,
    MifareClassic4K,
    MifarePlus,
    MifareDesfire,
};

class RC522_SPI
{
public:
    struct Config
    {
        int sckPin = -1;
        int misoPin = -1;
        int mosiPin = -1;
        int csPin = -1;
        int resetPin = -1;
        uint32_t spiClockHz = 4000000;
        uint32_t timeoutUs = 30000;

        // 共用一个已经启动的 SPI 总线时设为 false。
        bool initializeSpiBus = true;
    };

    bool begin(SPIClass &spi, const Config &config);
    void end();

    bool isReady() const;
    uint8_t version();

    Result setAntenna(bool enabled);
    Result request(RequestMode mode, uint16_t &atqa);
    Result anticollision(uint8_t uid[4]);
    Result select(const uint8_t uid[4], uint8_t &sak);

    // 完成寻卡、防碰撞和选卡；UID 为 4 byte 的 MIFARE Classic/Ultralight。
    Result readCard(
        uint8_t uid[4],
        uint16_t &atqa,
        uint8_t &sak,
        RequestMode mode = RequestMode::Idle
    );

    Result authenticate(
        uint8_t block,
        KeyType keyType,
        const uint8_t key[6],
        const uint8_t uid[4]
    );

    Result readBlock(uint8_t block, uint8_t data[16]);
    Result writeBlock(uint8_t block, const uint8_t data[16]);
    Result halt();

    void stopCrypto1();

    static CardType cardType(uint16_t atqa);
    static const char *toString(Result result);

private:
    Result transceive(
        uint8_t command,
        const uint8_t *txData,
        uint8_t txLength,
        uint8_t *rxData,
        uint8_t rxCapacity,
        uint16_t &rxBitLength
    );

    bool calculateCrc(
        const uint8_t *data,
        uint8_t length,
        uint8_t crc[2]
    );

    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void setBitMask(uint8_t reg, uint8_t mask);
    void clearBitMask(uint8_t reg, uint8_t mask);
    bool resetChip();
    Result mapError(uint8_t errorReg);

    SPIClass *mSpi = nullptr;
    Config mConfig{};
    bool mReady = false;
};

} // namespace rc522

#endif
