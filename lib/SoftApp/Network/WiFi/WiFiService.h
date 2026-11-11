#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#include <Arduino.h>
#include <WiFi.h>

namespace softapp
{
namespace wifi
{

enum class Result : uint8_t
{
    Ok = 0,
    InvalidConfig,
    ConfigureFailed,
    StartFailed,
};

//WiFi信息
struct AccessPointConfig
{
    char ssid[33] = "FLYInternet";
    char password[64] = "12345678";

    uint8_t localIp[4] = {192, 168, 26, 28};
    uint8_t gateway[4] = {192, 168, 26, 28};
    uint8_t subnet[4] = {255, 255, 255, 0};

    uint8_t channel = 6;
    bool hidden = false;
    uint8_t maxClients = 4;
};

class AccessPointService
{
public:
    Result begin(const AccessPointConfig &config);
    void end();

    bool isRunning() const;
    uint8_t stationCount() const;
    IPAddress ip() const;

    const AccessPointConfig &config() const;

private:
    AccessPointConfig mConfig{};
    bool mRunning = false;
};

const char *toString(Result result);

} // namespace wifi
} // namespace softapp

#endif
