#ifndef WIFI_RECEIVER_H
#define WIFI_RECEIVER_H

#include <Arduino.h>
#include <WiFiUdp.h>

namespace softapp
{
namespace wifi
{

enum class ReceiverResult : uint8_t
{
    Ok = 0,
    InvalidConfig,
    BindFailed,
    NotRunning,
};

struct CrtpHeader
{
    uint8_t port = 0;
    uint8_t reserved = 0;
    uint8_t channel = 0;
};

struct RemoteCommand
{
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawRateDps = 0.0f;
    uint16_t thrust = 0;
    bool carefree = false;
    bool valid = false;
    uint32_t receivedAtMs = 0;
};

struct RidInfo
{
    bool positionValid = false;
    bool altitudeValid = false;
    bool timeValid = false;
    double longitude = 0.0;
    double latitude = 0.0;
    float altitudeM = 0.0f;
    uint64_t unixTimeMs = 0;
    uint32_t receivedAtMs = 0;
};

struct RemoteHover
{
    bool enabled = false;
    bool valid = false;
    uint32_t receivedAtMs = 0;
};

struct RemoteArm
{
    bool enabled = false;
    bool valid = false;
    uint32_t receivedAtMs = 0;
};

struct RemotePidSettings
{
    // roll-angle, pitch-angle, yaw-rate, roll-rate, pitch-rate
    // Each loop contains Kp, Ki and Kd.
    float gains[5][3] = {};
    bool valid = false;
    uint32_t receivedAtMs = 0;
};

struct ReceiverStats
{
    uint32_t packets = 0;
    uint32_t bytes = 0;
    uint32_t checksumErrors = 0;
    uint32_t invalidPackets = 0;
    uint32_t commandPackets = 0;
    uint32_t hoverPackets = 0;
    uint32_t armPackets = 0;
    uint32_t pidPackets = 0;
    uint32_t ridPackets = 0;
    uint32_t lastRxMs = 0;
};

class WiFiReceiver
{
public:
    using CommandHandler = void (*)(const RemoteCommand &command);
    using HoverHandler = void (*)(const RemoteHover &hover);
    using ArmHandler = void (*)(const RemoteArm &arm);
    using PidSettingsHandler = void (*)(
        const RemotePidSettings &settings
    );

    ReceiverResult begin(uint16_t port = 2390);
    void end();

    void poll();
    bool isRunning() const;

    bool hasPeer();
    IPAddress peerIp();
    uint16_t peerPort();
    bool sendTelemetry(
        float batteryVoltage,
        float batteryPercent,
        float relativeHeightM
    );

    RemoteCommand latestCommand();
    RemoteHover latestHover();
    RemoteArm latestArm();
    RidInfo latestRid();
    ReceiverStats stats();
    void resetStats();

    void setCommandHandler(CommandHandler handler);
    void setHoverHandler(HoverHandler handler);
    void setArmHandler(ArmHandler handler);
    void setPidSettingsHandler(PidSettingsHandler handler);

    static bool isCommandFresh(
        const RemoteCommand &command,
        uint32_t maxAgeMs
    );

    static bool isHoverFresh(
        const RemoteHover &hover,
        uint32_t maxAgeMs
    );

    static bool isArmFresh(
        const RemoteArm &arm,
        uint32_t maxAgeMs
    );

private:
    bool handlePacket(
        const uint8_t *data,
        size_t length,
        IPAddress sourceIp,
        uint16_t sourcePort
    );

    bool decodeCommand(
        const uint8_t *payload,
        size_t length,
        uint8_t reserved,
        RemoteCommand &command
    );

    bool decodeRid(const uint8_t *payload, size_t length, RidInfo &rid);
    bool decodeHover(
        const uint8_t *payload,
        size_t length,
        RemoteHover &hover
    );
    bool decodeArm(
        const uint8_t *payload,
        size_t length,
        RemoteArm &arm
    );
    bool decodePidSettings(
        const uint8_t *payload,
        size_t length,
        RemotePidSettings &settings
    );
    static CrtpHeader parseHeader(uint8_t header);

    WiFiUDP mUdp;
    uint8_t mRxBuffer[1600] = {};
    uint16_t mPort = 0;
    bool mRunning = false;

    IPAddress mPeerIp;
    uint16_t mPeerPort = 0;
    uint32_t mPeerLastMs = 0;

    RemoteCommand mLastCommand;
    RemoteHover mLastHover;
    RemoteArm mLastArm;
    RemotePidSettings mLastPidSettings;
    RidInfo mLastRid;
    ReceiverStats mStats;
    CommandHandler mCommandHandler = nullptr;
    HoverHandler mHoverHandler = nullptr;
    ArmHandler mArmHandler = nullptr;
    PidSettingsHandler mPidSettingsHandler = nullptr;
    portMUX_TYPE mDataMux = portMUX_INITIALIZER_UNLOCKED;
};

const char *toString(ReceiverResult result);

} // namespace wifi
} // namespace softapp

#endif
