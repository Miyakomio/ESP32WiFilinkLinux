#include "SpiProtocol.h"

#include <string.h>

namespace spi_link
{
namespace
{

constexpr size_t kKindOffset = 2;
constexpr size_t kFlagsOffset = 3;
constexpr size_t kSequenceOffset = 4;
constexpr size_t kStatusOffset = 6;
constexpr size_t kPayloadLengthOffset = 7;
constexpr size_t kPayloadOffset = 8;
constexpr size_t kChecksumOffset = 56;
constexpr size_t kTrailerOffset = 58;
constexpr size_t kReservedOffset = 60;

void writeU16(uint8_t *out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value >> 8);
    out[1] = static_cast<uint8_t>(value & 0xFF);
}

uint16_t readU16(const uint8_t *in)
{
    return static_cast<uint16_t>(
        (static_cast<uint16_t>(in[0]) << 8) |
        static_cast<uint16_t>(in[1])
    );
}

} // namespace

bool isKnownKind(MessageKind kind)
{
    return static_cast<uint8_t>(kind) <=
        static_cast<uint8_t>(MessageKind::Event);
}

bool isKnownStatus(Status status)
{
    return static_cast<uint8_t>(status) <=
        static_cast<uint8_t>(Status::InternalError);
}

ValidationError validate(const Message &message)
{
    if (!isKnownKind(message.kind))
    {
        return ValidationError::UnknownKind;
    }

    if (!isKnownStatus(message.status))
    {
        return ValidationError::UnknownStatus;
    }

    if (message.payload_length > kMaxPayloadSize)
    {
        return ValidationError::PayloadTooLarge;
    }

    return ValidationError::None;
}

const char *toString(ValidationError error)
{
    switch (error)
    {
        case ValidationError::None:
            return "ok";
        case ValidationError::UnknownKind:
            return "unknown message kind";
        case ValidationError::UnknownStatus:
            return "unknown status";
        case ValidationError::PayloadTooLarge:
            return "payload too large";
    }

    return "unknown validation error";
}

uint16_t crc16Ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    if (data == nullptr)
    {
        return crc;
    }

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x8000U) != 0)
            {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

bool encode(const Frame &frame, uint8_t (&out)[kFrameSize])
{
    if (validate(frame) != ValidationError::None)
    {
        return false;
    }

    memset(out, 0, kFrameSize);
    out[0] = kMagic;
    out[1] = kProtocolVersion;
    out[kKindOffset] = static_cast<uint8_t>(frame.kind);
    out[kFlagsOffset] = frame.flags;
    writeU16(out + kSequenceOffset, frame.sequence);
    out[kStatusOffset] = static_cast<uint8_t>(frame.status);
    out[kPayloadLengthOffset] = frame.payload_length;

    if (frame.payload_length > 0)
    {
        memcpy(
            out + kPayloadOffset,
            frame.payload,
            frame.payload_length
        );
    }

    writeU16(out + kChecksumOffset, crc16Ccitt(out, kChecksumOffset));
    out[kTrailerOffset] = kTrailer0;
    out[kTrailerOffset + 1] = kTrailer1;
    memset(out + kReservedOffset, 0, kFrameSize - kReservedOffset);
    return true;
}

DecodeError decode(
    const uint8_t *in,
    size_t length,
    Frame &out
)
{
    if (in == nullptr)
    {
        return DecodeError::NullBuffer;
    }

    if (length != kFrameSize)
    {
        return DecodeError::WrongLength;
    }

    if (in[0] != kMagic)
    {
        return DecodeError::BadMagic;
    }

    if (in[1] != kProtocolVersion)
    {
        return DecodeError::UnsupportedVersion;
    }

    if (!isKnownKind(
            static_cast<MessageKind>(in[kKindOffset])))
    {
        return DecodeError::UnknownKind;
    }

    if (!isKnownStatus(
            static_cast<Status>(in[kStatusOffset])))
    {
        return DecodeError::UnknownStatus;
    }

    if (in[kPayloadLengthOffset] > kMaxPayloadSize)
    {
        return DecodeError::PayloadTooLarge;
    }

    const uint16_t expectedChecksum = readU16(in + kChecksumOffset);
    const uint16_t actualChecksum = crc16Ccitt(in, kChecksumOffset);

    if (actualChecksum != expectedChecksum)
    {
        return DecodeError::BadChecksum;
    }

    if (in[kTrailerOffset] != kTrailer0 ||
        in[kTrailerOffset + 1] != kTrailer1)
    {
        return DecodeError::BadTrailer;
    }

    out.kind = static_cast<MessageKind>(in[kKindOffset]);
    out.flags = in[kFlagsOffset];
    out.sequence = readU16(in + kSequenceOffset);
    out.status = static_cast<Status>(in[kStatusOffset]);
    out.payload_length = in[kPayloadLengthOffset];
    memset(out.payload, 0, kMaxPayloadSize);

    if (out.payload_length > 0)
    {
        memcpy(
            out.payload,
            in + kPayloadOffset,
            out.payload_length
        );
    }

    return DecodeError::None;
}

void makeNoop(Status status, Frame &out)
{
    out = Frame{};
    out.kind = MessageKind::Noop;
    out.status = status;
}

const char *toString(DecodeError error)
{
    switch (error)
    {
        case DecodeError::None:
            return "ok";
        case DecodeError::NullBuffer:
            return "null buffer";
        case DecodeError::WrongLength:
            return "wrong frame length";
        case DecodeError::BadMagic:
            return "bad magic";
        case DecodeError::UnsupportedVersion:
            return "unsupported version";
        case DecodeError::UnknownKind:
            return "unknown message kind";
        case DecodeError::UnknownStatus:
            return "unknown status";
        case DecodeError::PayloadTooLarge:
            return "payload too large";
        case DecodeError::BadChecksum:
            return "bad checksum";
        case DecodeError::BadTrailer:
            return "bad trailer";
    }

    return "unknown error";
}

} // namespace spi_link
