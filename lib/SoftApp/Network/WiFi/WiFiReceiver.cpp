#include "WiFiReceiver.h"

#include <math.h>
#include <string.h>

namespace softapp
{
namespace wifi
{
namespace
{

constexpr uint32_t kPeerTimeoutMs = 3000;
constexpr uint8_t kCommanderPort = 3;
constexpr uint8_t kCommanderChannel = 0;
constexpr uint8_t kHoverChannel = 1;
constexpr uint8_t kArmChannel = 2;
constexpr uint8_t kPidSettingsChannel = 3;
constexpr uint8_t kCalibrationPort = 8;
constexpr uint8_t kCalibrationChannel = 0;
constexpr uint8_t kRidPort = 13;
constexpr uint8_t kRidChannel = 2;
constexpr uint8_t kPidSettingsMagic = 0x50;
constexpr uint8_t kPidSettingsVersion = 1;
constexpr uint8_t kCalibrationMagic = 0xCA;
constexpr uint8_t kCalibrationVersion = 1;
constexpr size_t kPidLoopCount = 5;
constexpr size_t kPidGainCount = kPidLoopCount * 3;
constexpr size_t kPidSettingsPayloadSize = 4 + kPidGainCount * 4;
constexpr float kMaxPidGain = 100.0f;

template <typename T>
T readLittleEndian(const uint8_t *data)
{
    T value{};
    memcpy(&value, data, sizeof(T));
    return value;
}

} // namespace

ReceiverResult WiFiReceiver::begin(uint16_t port)
{
    if (mRunning)
    {
        return ReceiverResult::Ok;
    }

    if (port == 0)
    {
        return ReceiverResult::InvalidConfig;
    }

    mPort = port;
    if (!mUdp.begin(port))
    {
        mPort = 0;
        return ReceiverResult::BindFailed;
    }

    mRunning = true;
    return ReceiverResult::Ok;
}

void WiFiReceiver::end()
{
    if (mRunning)
    {
        mUdp.stop();
    }

    mRunning = false;
    mPort = 0;

    portENTER_CRITICAL(&mDataMux);
    mLastHover = RemoteHover{};
    mLastArm = RemoteArm{};
    portEXIT_CRITICAL(&mDataMux);
}

void WiFiReceiver::poll()
{
    if (!mRunning)
    {
        return;
    }

    int packetSize = 0;
    while ((packetSize = mUdp.parsePacket()) > 0)
    {
        const int received = mUdp.read(
            mRxBuffer,
            sizeof(mRxBuffer)
        );

        if (received > 0)
        {
            handlePacket(
                mRxBuffer,
                static_cast<size_t>(received),
                mUdp.remoteIP(),
                mUdp.remotePort()
            );
        }
    }
}

bool WiFiReceiver::isRunning() const
{
    return mRunning;
}

bool WiFiReceiver::hasPeer()
{
    portENTER_CRITICAL(&mDataMux);
    const bool connected =
        mPeerPort != 0 &&
        (millis() - mPeerLastMs) < kPeerTimeoutMs;
    portEXIT_CRITICAL(&mDataMux);
    return connected;
}

IPAddress WiFiReceiver::peerIp()
{
    portENTER_CRITICAL(&mDataMux);
    const IPAddress result = mPeerIp;
    portEXIT_CRITICAL(&mDataMux);
    return result;
}

uint16_t WiFiReceiver::peerPort()
{
    portENTER_CRITICAL(&mDataMux);
    const uint16_t result = mPeerPort;
    portEXIT_CRITICAL(&mDataMux);
    return result;
}

bool WiFiReceiver::sendTelemetry(
    float batteryVoltage,
    float batteryPercent,
    float relativeHeightM
)
{
    if (!mRunning ||
        batteryVoltage < 0.0f ||
        batteryVoltage > 10.0f)
    {
        return false;
    }

    const IPAddress targetIp = peerIp();
    const uint16_t targetPort = peerPort();
    if (targetPort == 0)
    {
        return false;
    }

    uint8_t packet[18] = {};
    packet[0] = 0x81; // Port 8, channel 1
    packet[1] = 0x00;
    memcpy(packet + 2, &batteryVoltage, sizeof(batteryVoltage));
    memcpy(packet + 6, &relativeHeightM, sizeof(relativeHeightM));

    const float limitedPercent =
        batteryPercent < 0.0f
            ? 0.0f
            : (batteryPercent > 100.0f ? 100.0f : batteryPercent);
    memcpy(packet + 13, &limitedPercent, sizeof(limitedPercent));

    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(packet) - 1; ++i)
    {
        checksum = static_cast<uint8_t>(checksum + packet[i]);
    }
    packet[sizeof(packet) - 1] = checksum;

    if (!mUdp.beginPacket(targetIp, targetPort))
    {
        return false;
    }
    mUdp.write(packet, sizeof(packet));
    return mUdp.endPacket() == 1;
}

RemoteCommand WiFiReceiver::latestCommand()
{
    portENTER_CRITICAL(&mDataMux);
    const RemoteCommand command = mLastCommand;
    portEXIT_CRITICAL(&mDataMux);
    return command;
}

RemoteHover WiFiReceiver::latestHover()
{
    portENTER_CRITICAL(&mDataMux);
    const RemoteHover hover = mLastHover;
    portEXIT_CRITICAL(&mDataMux);
    return hover;
}

RemoteArm WiFiReceiver::latestArm()
{
    portENTER_CRITICAL(&mDataMux);
    const RemoteArm arm = mLastArm;
    portEXIT_CRITICAL(&mDataMux);
    return arm;
}

RidInfo WiFiReceiver::latestRid()
{
    portENTER_CRITICAL(&mDataMux);
    const RidInfo rid = mLastRid;
    portEXIT_CRITICAL(&mDataMux);
    return rid;
}

ReceiverStats WiFiReceiver::stats()
{
    portENTER_CRITICAL(&mDataMux);
    const ReceiverStats result = mStats;
    portEXIT_CRITICAL(&mDataMux);
    return result;
}

void WiFiReceiver::resetStats()
{
    portENTER_CRITICAL(&mDataMux);
    mStats = ReceiverStats{};
    portEXIT_CRITICAL(&mDataMux);
}

void WiFiReceiver::setCommandHandler(CommandHandler handler)
{
    mCommandHandler = handler;
}

void WiFiReceiver::setHoverHandler(HoverHandler handler)
{
    mHoverHandler = handler;
}

void WiFiReceiver::setArmHandler(ArmHandler handler)
{
    mArmHandler = handler;
}

void WiFiReceiver::setPidSettingsHandler(PidSettingsHandler handler)
{
    mPidSettingsHandler = handler;
}

void WiFiReceiver::setCalibrationHandler(CalibrationHandler handler)
{
    mCalibrationHandler = handler;
}

bool WiFiReceiver::isCommandFresh(
    const RemoteCommand &command,
    uint32_t maxAgeMs
)
{
    return command.valid &&
        (millis() - command.receivedAtMs) <= maxAgeMs;
}

bool WiFiReceiver::isHoverFresh(
    const RemoteHover &hover,
    uint32_t maxAgeMs
)
{
    return hover.valid &&
        (millis() - hover.receivedAtMs) <= maxAgeMs;
}

bool WiFiReceiver::isArmFresh(
    const RemoteArm &arm,
    uint32_t maxAgeMs
)
{
    return arm.valid &&
        (millis() - arm.receivedAtMs) <= maxAgeMs;
}

bool WiFiReceiver::handlePacket(
    const uint8_t *data,
    size_t length,
    IPAddress sourceIp,
    uint16_t sourcePort
)
{
    const uint32_t now = millis();

    portENTER_CRITICAL(&mDataMux);
    ++mStats.packets;
    mStats.bytes += length;
    mStats.lastRxMs = now;
    mPeerIp = sourceIp;
    mPeerPort = sourcePort;
    mPeerLastMs = now;
    portEXIT_CRITICAL(&mDataMux);

    if (length < 2)
    {
        portENTER_CRITICAL(&mDataMux);
        ++mStats.invalidPackets;
        portEXIT_CRITICAL(&mDataMux);
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i + 1 < length; ++i)
    {
        checksum = static_cast<uint8_t>(checksum + data[i]);
    }

    if (checksum != data[length - 1])
    {
        portENTER_CRITICAL(&mDataMux);
        ++mStats.checksumErrors;
        portEXIT_CRITICAL(&mDataMux);
        return false;
    }

    const CrtpHeader header = parseHeader(data[0]);
    const uint8_t *payload = data + 1;
    const size_t payloadLength = length - 2;

    if (header.port == kCommanderPort &&
        header.channel == kCommanderChannel)
    {
        RemoteCommand command;
        if (!decodeCommand(
                payload,
                payloadLength,
                header.reserved,
                command))
        {
            portENTER_CRITICAL(&mDataMux);
            ++mStats.invalidPackets;
            portEXIT_CRITICAL(&mDataMux);
            return false;
        }

        portENTER_CRITICAL(&mDataMux);
        mLastCommand = command;
        ++mStats.commandPackets;
        portEXIT_CRITICAL(&mDataMux);

        if (mCommandHandler != nullptr)
        {
            mCommandHandler(command);
        }
        return true;
    }

    if (header.port == kCommanderPort &&
        header.channel == kHoverChannel)
    {
        RemoteHover hover;
        if (!decodeHover(payload, payloadLength, hover))
        {
            portENTER_CRITICAL(&mDataMux);
            ++mStats.invalidPackets;
            portEXIT_CRITICAL(&mDataMux);
            return false;
        }

        portENTER_CRITICAL(&mDataMux);
        mLastHover = hover;
        ++mStats.hoverPackets;
        portEXIT_CRITICAL(&mDataMux);

        if (mHoverHandler != nullptr)
        {
            mHoverHandler(hover);
        }
        return true;
    }

    if (header.port == kCommanderPort &&
        header.channel == kArmChannel)
    {
        RemoteArm arm;
        if (!decodeArm(payload, payloadLength, arm))
        {
            portENTER_CRITICAL(&mDataMux);
            ++mStats.invalidPackets;
            portEXIT_CRITICAL(&mDataMux);
            return false;
        }

        portENTER_CRITICAL(&mDataMux);
        mLastArm = arm;
        ++mStats.armPackets;
        portEXIT_CRITICAL(&mDataMux);

        if (mArmHandler != nullptr)
        {
            mArmHandler(arm);
        }
        return true;
    }

    if (header.port == kCommanderPort &&
        header.channel == kPidSettingsChannel)
    {
        RemotePidSettings settings;
        if (!decodePidSettings(payload, payloadLength, settings))
        {
            portENTER_CRITICAL(&mDataMux);
            ++mStats.invalidPackets;
            portEXIT_CRITICAL(&mDataMux);
            return false;
        }

        portENTER_CRITICAL(&mDataMux);
        mLastPidSettings = settings;
        ++mStats.pidPackets;
        portEXIT_CRITICAL(&mDataMux);

        if (mPidSettingsHandler != nullptr)
        {
            mPidSettingsHandler(settings);
        }
        return true;
    }

    if (header.port == kCalibrationPort &&
        header.channel == kCalibrationChannel)
    {
        if (!decodeCalibration(payload, payloadLength))
        {
            portENTER_CRITICAL(&mDataMux);
            ++mStats.invalidPackets;
            portEXIT_CRITICAL(&mDataMux);
            return false;
        }

        portENTER_CRITICAL(&mDataMux);
        ++mStats.calibrationPackets;
        portEXIT_CRITICAL(&mDataMux);

        if (mCalibrationHandler != nullptr)
        {
            mCalibrationHandler();
        }
        return true;
    }

    if (header.port == kRidPort && header.channel == kRidChannel)
    {
        RidInfo rid;
        if (!decodeRid(payload, payloadLength, rid))
        {
            portENTER_CRITICAL(&mDataMux);
            ++mStats.invalidPackets;
            portEXIT_CRITICAL(&mDataMux);
            return false;
        }

        portENTER_CRITICAL(&mDataMux);
        mLastRid = rid;
        ++mStats.ridPackets;
        portEXIT_CRITICAL(&mDataMux);
        return true;
    }

    return false;
}

bool WiFiReceiver::decodeCommand(
    const uint8_t *payload,
    size_t length,
    uint8_t reserved,
    RemoteCommand &command
)
{
    if (payload == nullptr || length < 14)
    {
        return false;
    }

    command.rollDeg = readLittleEndian<float>(payload);
    command.pitchDeg = readLittleEndian<float>(payload + 4);
    command.yawRateDps = readLittleEndian<float>(payload + 8);
    command.thrust = readLittleEndian<uint16_t>(payload + 12);
    command.carefree = (reserved & 0x01) != 0;
    command.receivedAtMs = millis();
    command.valid = true;

    return isfinite(command.rollDeg) &&
        isfinite(command.pitchDeg) &&
        isfinite(command.yawRateDps) &&
        command.rollDeg >= -90.0f &&
        command.rollDeg <= 90.0f &&
        command.pitchDeg >= -90.0f &&
        command.pitchDeg <= 90.0f &&
        command.yawRateDps >= -2000.0f &&
        command.yawRateDps <= 2000.0f;
}

bool WiFiReceiver::decodeRid(
    const uint8_t *payload,
    size_t length,
    RidInfo &rid
)
{
    if (payload == nullptr || length < 24 || payload[0] != 0x52)
    {
        return false;
    }

    const int32_t longitude = readLittleEndian<int32_t>(payload + 4);
    const int32_t latitude = readLittleEndian<int32_t>(payload + 8);
    const int32_t altitude = readLittleEndian<int32_t>(payload + 12);
    const uint8_t flags = payload[2];

    rid.positionValid = (flags & 0x01) != 0;
    rid.altitudeValid = (flags & 0x02) != 0;
    rid.timeValid = (flags & 0x04) != 0;
    rid.longitude = static_cast<double>(longitude) / 10000000.0;
    rid.latitude = static_cast<double>(latitude) / 10000000.0;
    rid.altitudeM = static_cast<float>(altitude) / 100.0f;

    rid.unixTimeMs = 0;
    for (int i = 5; i >= 0; --i)
    {
        rid.unixTimeMs =
            (rid.unixTimeMs << 8) | payload[16 + i];
    }
    rid.receivedAtMs = millis();
    return true;
}

bool WiFiReceiver::decodeHover(
    const uint8_t *payload,
    size_t length,
    RemoteHover &hover
)
{
    if (payload == nullptr || length < 1 || payload[0] > 1)
    {
        return false;
    }

    hover.enabled = payload[0] != 0;
    hover.valid = true;
    hover.receivedAtMs = millis();
    return true;
}

bool WiFiReceiver::decodeArm(
    const uint8_t *payload,
    size_t length,
    RemoteArm &arm
)
{
    if (payload == nullptr || length < 1 || payload[0] > 1)
    {
        return false;
    }

    arm.enabled = payload[0] != 0;
    arm.valid = true;
    arm.receivedAtMs = millis();
    return true;
}

bool WiFiReceiver::decodePidSettings(
    const uint8_t *payload,
    size_t length,
    RemotePidSettings &settings
)
{
    if (payload == nullptr ||
        length < kPidSettingsPayloadSize ||
        payload[0] != kPidSettingsMagic ||
        payload[1] != kPidSettingsVersion ||
        payload[2] != kPidLoopCount ||
        payload[3] != 0)
    {
        return false;
    }

    RemotePidSettings decoded;
    size_t offset = 4;
    for (size_t loop = 0; loop < kPidLoopCount; ++loop)
    {
        for (size_t gain = 0; gain < 3; ++gain)
        {
            const float value = readLittleEndian<float>(payload + offset);
            if (!isfinite(value) ||
                value < 0.0f ||
                value > kMaxPidGain)
            {
                return false;
            }
            decoded.gains[loop][gain] = value;
            offset += sizeof(float);
        }
    }
    decoded.receivedAtMs = millis();
    decoded.valid = true;
    settings = decoded;
    return true;
}

bool WiFiReceiver::decodeCalibration(
    const uint8_t *payload,
    size_t length
)
{
    return payload != nullptr &&
        length == 2 &&
        payload[0] == kCalibrationMagic &&
        payload[1] == kCalibrationVersion;
}

CrtpHeader WiFiReceiver::parseHeader(uint8_t header)
{
    CrtpHeader parsed;
    parsed.port = (header >> 4) & 0x0F;
    parsed.reserved = (header >> 2) & 0x03;
    parsed.channel = header & 0x03;
    return parsed;
}

const char *toString(ReceiverResult result)
{
    switch (result)
    {
        case ReceiverResult::Ok:
            return "ok";
        case ReceiverResult::InvalidConfig:
            return "invalid config";
        case ReceiverResult::BindFailed:
            return "UDP bind failed";
        case ReceiverResult::NotRunning:
            return "not running";
    }

    return "unknown receiver error";
}

} // namespace wifi
} // namespace softapp
