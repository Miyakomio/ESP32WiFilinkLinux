#include "WiFiService.h"

#include <string.h>

namespace softapp
{
namespace wifi
{
namespace
{

bool hasTerminator(const char *text, size_t capacity)
{
    return text != nullptr &&
        memchr(text, '\0', capacity) != nullptr;
}

} // namespace

Result AccessPointService::begin(const AccessPointConfig &config)
{
    if (mRunning)
    {
        return Result::Ok;
    }

    if (!hasTerminator(config.ssid, sizeof(config.ssid)) ||
        !hasTerminator(config.password, sizeof(config.password)) ||
        config.ssid[0] == '\0' ||
        (config.password[0] != '\0' &&
         strlen(config.password) < 8) ||
        config.channel == 0 ||
        config.channel > 13)
    {
        return Result::InvalidConfig;
    }

    const IPAddress localIp(
        config.localIp[0],
        config.localIp[1],
        config.localIp[2],
        config.localIp[3]
    );
    const IPAddress gateway(
        config.gateway[0],
        config.gateway[1],
        config.gateway[2],
        config.gateway[3]
    );
    const IPAddress subnet(
        config.subnet[0],
        config.subnet[1],
        config.subnet[2],
        config.subnet[3]
    );

    WiFi.mode(WIFI_AP);

    if (!WiFi.softAPConfig(localIp, gateway, subnet))
    {
        WiFi.mode(WIFI_OFF);
        return Result::ConfigureFailed;
    }

    if (!WiFi.softAP(
            config.ssid,
            config.password,
            config.channel,
            config.hidden,
            config.maxClients))
    {
        WiFi.mode(WIFI_OFF);
        return Result::StartFailed;
    }

    mConfig = config;
    mRunning = true;
    return Result::Ok;
}

void AccessPointService::end()
{
    if (!mRunning)
    {
        return;
    }

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    mRunning = false;
}

bool AccessPointService::isRunning() const
{
    return mRunning;
}

uint8_t AccessPointService::stationCount() const
{
    return mRunning
        ? static_cast<uint8_t>(WiFi.softAPgetStationNum())
        : 0;
}

IPAddress AccessPointService::ip() const
{
    return mRunning ? WiFi.softAPIP() : IPAddress();
}

const AccessPointConfig &AccessPointService::config() const
{
    return mConfig;
}

const char *toString(Result result)
{
    switch (result)
    {
        case Result::Ok:
            return "ok";
        case Result::InvalidConfig:
            return "invalid config";
        case Result::ConfigureFailed:
            return "configure failed";
        case Result::StartFailed:
            return "start failed";
    }

    return "unknown error";
}

} // namespace wifi
} // namespace softapp
