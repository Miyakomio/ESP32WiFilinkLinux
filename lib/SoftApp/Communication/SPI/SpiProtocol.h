#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace spi_link
{

// 软件通信层统一消息模型，后续传输方式复用同一套语义。
constexpr size_t kMaxPayloadSize = 48;
constexpr uint8_t kProtocolVersion = 0x01;

enum class MessageKind : uint8_t
{
    Noop = 0x00,
    Request = 0x01,
    Response = 0x02,
    Event = 0x03,
};

enum class Status : uint8_t
{
    Ok = 0x00,
    Busy = 0x01,
    Unsupported = 0x02,
    BadRequest = 0x03,
    CrcError = 0x04,
    NoSpace = 0x05,
    InternalError = 0x06,
};

struct Message
{
    MessageKind kind = MessageKind::Noop;
    uint8_t flags = 0;
    uint16_t sequence = 0;
    Status status = Status::Ok;
    uint8_t payload_length = 0;
    uint8_t payload[kMaxPayloadSize] = {};
};

enum class ValidationError : uint8_t
{
    None = 0,
    UnknownKind,
    UnknownStatus,
    PayloadTooLarge,
};

bool isKnownKind(MessageKind kind);
bool isKnownStatus(Status status);
ValidationError validate(const Message &message);
const char *toString(ValidationError error);

// Wi-Fi、串口等后续传输方式实现相同接口。
class Transport
{
public:
    virtual ~Transport() = default;
    virtual bool send(const Message &message) = 0;
    virtual bool receive(Message &message) = 0;
};

class SequenceAllocator
{
public:
    uint16_t next()
    {
        ++value_;
        if (value_ == 0)
        {
            ++value_;
        }
        return value_;
    }

    void reset(uint16_t value = 0)
    {
        value_ = value;
    }

private:
    uint16_t value_ = 0;
};

// 固定帧长让主机无需先查询长度，也便于 SPI 从机预提交收发缓冲区。
constexpr size_t kFrameSize = 64;
constexpr uint8_t kMagic = 0xA5;
constexpr uint8_t kTrailer0 = 0x5A;
constexpr uint8_t kTrailer1 = 0xA5;

using Frame = Message;

enum class DecodeError : uint8_t
{
    None = 0,
    NullBuffer,
    WrongLength,
    BadMagic,
    UnsupportedVersion,
    UnknownKind,
    UnknownStatus,
    PayloadTooLarge,
    BadChecksum,
    BadTrailer,
};

uint16_t crc16Ccitt(const uint8_t *data, size_t length);

// encode() 会把 out 完整填充为 kFrameSize 字节，未使用区域清零。
bool encode(const Frame &frame, uint8_t (&out)[kFrameSize]);

DecodeError decode(
    const uint8_t *in,
    size_t length,
    Frame &out
);

void makeNoop(Status status, Frame &out);

const char *toString(DecodeError error);

} // namespace spi_link

#endif
