#include "WiFiTask.h"

#include "BatteryTask.h"
#include "MotorTask.h"
#include "MpuTask.h"
#include "../../SoftApp/Control/MotorControl.h"

namespace task_group
{
namespace wifi_task
{
namespace
{

constexpr uint32_t kStatusIntervalMs = 2000;
constexpr uint32_t kRetryIntervalMs = 2000;
constexpr uint32_t kReceivePollIntervalMs = 1;
constexpr uint32_t kCommandPrintIntervalMs = 100;
constexpr uint16_t kMaxCommandThrust = 65000;
constexpr uint16_t kArmMaxThrottle =
    kMaxCommandThrust * 5 / 100;

softapp::wifi::AccessPointService sService;
softapp::wifi::WiFiReceiver sReceiver;
softapp::wifi::AccessPointConfig sConfig;
softapp::wifi::WiFiReceiver::CommandHandler sUserCommandHandler =
    nullptr;
softapp::wifi::WiFiReceiver::HoverHandler sUserHoverHandler =
    nullptr;
softapp::wifi::WiFiReceiver::ArmHandler sUserArmHandler =
    nullptr;

//控制端遥感数据接收
void handleCommand(const softapp::wifi::RemoteCommand &command)
{
    static uint32_t lastPrintMs = 0;
    static bool firstCommand = true;

    const uint32_t now = millis();
    if (firstCommand ||
        now - lastPrintMs >= kCommandPrintIntervalMs)
    {
        firstCommand = false;
        lastPrintMs = now;

        Serial.printf(
            "RX R=%.1f P=%.1f Y=%.1f T=%u\n",
            command.rollDeg,
            command.pitchDeg,
            command.yawRateDps,
            static_cast<unsigned>(command.thrust)
        );
    }

    if (sUserCommandHandler != nullptr)
    {
        sUserCommandHandler(command);
    }
}

//控制端悬停信息接收::hover.enabled 悬停开关使能控制
void handleHover(const softapp::wifi::RemoteHover &hover)
{
    static bool initialized = false;
    static bool lastEnabled = false;

    if (!initialized || hover.enabled != lastEnabled)
    {
        initialized = true;
        lastEnabled = hover.enabled;
        Serial.printf(
            "Hover request: %s\n",
            hover.enabled ? "ON" : "OFF"
        );
    }

    if (sUserHoverHandler != nullptr)
    {
        sUserHoverHandler(hover);
    }
}

void handleArm(const softapp::wifi::RemoteArm &arm)
{
    static bool initialized = false;
    static bool armed = false;
    static uint32_t lastRejectLogMs = 0;

    if (!arm.valid)
    {
        motor_task::lock();
        armed = false;
        return;
    }

    if (!arm.enabled)
    {
        if (!initialized || armed)
        {
            initialized = true;
            armed = false;
            motor_task::lock();
            Serial.println("Arm request: LOCKED");
        }
    }
    else if (!armed)
    {
        if (mpu_task::isCalibrating())
        {
            motor_task::lock();
            if (millis() - lastRejectLogMs >= 1000)
            {
                lastRejectLogMs = millis();
                Serial.println("Arm request: REJECTED (calibration active)");
            }
            return;
        }

        const softapp::wifi::RemoteCommand command =
            sReceiver.latestCommand();
        const bool commandFresh =
            softapp::wifi::WiFiReceiver::isCommandFresh(command, 500);
        const bool throttleSafe =
            commandFresh && command.thrust <= kArmMaxThrottle;

        if (throttleSafe)
        {
            initialized = true;
            armed = motor_task::unlock();
            Serial.println(
                armed
                    ? "Arm request: ARMED"
                    : "Arm request: REJECTED"
            );
        }
        else if (millis() - lastRejectLogMs >= 1000)
        {
            lastRejectLogMs = millis();
            Serial.printf(
                "Arm request: REJECTED (%s, T=%u, max=%u)\n",
                commandFresh ? "throttle too high" : "command stale",
                static_cast<unsigned>(command.thrust),
                static_cast<unsigned>(kArmMaxThrottle)
            );
        }
    }

    if (sUserArmHandler != nullptr)
    {
        sUserArmHandler(arm);
    }
}

void handleCalibration()
{
    motor_task::lock();
    const bool accepted = mpu_task::requestCalibration();
    Serial.println(
        accepted
            ? "Calibration request: ACCEPTED (motors locked)"
            : "Calibration request: REJECTED"
    );
}

void handlePidSettings(
    const softapp::wifi::RemotePidSettings &settings
)
{
    if (!settings.valid)
    {
        return;
    }

    if (!motor_task::isLocked())
    {
        Serial.println("PID update rejected: motors are armed");
        return;
    }

    PidSettings pid;
    pid.rollAngle = PidLoopSettings(
        settings.gains[0][0],
        settings.gains[0][1],
        settings.gains[0][2]
    );
    pid.pitchAngle = PidLoopSettings(
        settings.gains[1][0],
        settings.gains[1][1],
        settings.gains[1][2]
    );
    pid.yawRate = PidLoopSettings(
        settings.gains[2][0],
        settings.gains[2][1],
        settings.gains[2][2]
    );
    pid.rollRate = PidLoopSettings(
        settings.gains[3][0],
        settings.gains[3][1],
        settings.gains[3][2]
    );
    pid.pitchRate = PidLoopSettings(
        settings.gains[4][0],
        settings.gains[4][1],
        settings.gains[4][2]
    );

    if (applyMotorPidSettings(pid))
    {
        Serial.printf(
            "PID updated: angleR=%.3f/%.3f/%.3f "
            "angleP=%.3f/%.3f/%.3f rateR=%.3f/%.3f/%.3f "
            "rateP=%.3f/%.3f/%.3f yaw=%.3f/%.3f/%.3f\n",
            pid.rollAngle.kp,
            pid.rollAngle.ki,
            pid.rollAngle.kd,
            pid.pitchAngle.kp,
            pid.pitchAngle.ki,
            pid.pitchAngle.kd,
            pid.rollRate.kp,
            pid.rollRate.ki,
            pid.rollRate.kd,
            pid.pitchRate.kp,
            pid.pitchRate.ki,
            pid.pitchRate.kd,
            pid.yawRate.kp,
            pid.yawRate.ki,
            pid.yawRate.kd
        );
    }
    else
    {
        Serial.println("PID update rejected: invalid gains");
    }
}

void entry(void *context)
{
    (void)context;

    bool started = false;

    for (;;)
    {
        if (!started)
        {
            const softapp::wifi::Result result =
                sService.begin(sConfig);

            if (result != softapp::wifi::Result::Ok)
            {
                Serial.printf(
                    "Wi-Fi 热点启动失败: %s\n",
                    softapp::wifi::toString(result)
                );
                vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                continue;
            }

            const softapp::wifi::ReceiverResult receiverResult =
                sReceiver.begin(2390);
            if (receiverResult != softapp::wifi::ReceiverResult::Ok)
            {
                Serial.printf(
                    "Wi-Fi UDP 接收启动失败: %s\n",
                    softapp::wifi::toString(receiverResult)
                );
                sReceiver.end();
                vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                continue;
            }

            Serial.printf(
                "WiFi ready: SSID=%s IP=%s UDP=2390\n",
                sConfig.ssid,
                sService.ip().toString().c_str()
            );
            started = true;
        }

        sReceiver.poll();

        static uint32_t lastTelemetryMs = 0;
        static uint32_t lastStatusMs = 0;
        const uint32_t now = millis();

        if (now - lastTelemetryMs >= 1000)
        {
            lastTelemetryMs = now;
            const battery_task::DataSnapshot battery =
                battery_task::latestData();
            if (battery.valid &&
                now - battery.receivedAtMs <= 5000 &&
                sReceiver.hasPeer())
            {
                sReceiver.sendTelemetry(
                    battery.reading.cellVoltageMv / 1000.0f,
                    battery.reading.stateOfChargePercent,
                    0.0f
                );
            }
        }

        if (now - lastStatusMs >= kStatusIntervalMs)
        {
            lastStatusMs = now;
            const softapp::wifi::ReceiverStats stats =
                sReceiver.stats();
            const softapp::wifi::RemoteHover hover =
                sReceiver.latestHover();
            const bool hoverFresh =
                softapp::wifi::WiFiReceiver::isHoverFresh(hover, 1000);
            Serial.printf(
                "WiFi dev=%u rx=%lu cmd=%lu hover=%lu cal=%lu rid=%lu bad=%lu state=%s\n",
                static_cast<unsigned>(sService.stationCount()),
                static_cast<uint32_t>(stats.packets),
                static_cast<uint32_t>(stats.commandPackets),
                static_cast<uint32_t>(stats.hoverPackets),
                static_cast<uint32_t>(stats.calibrationPackets),
                static_cast<uint32_t>(stats.ridPackets),
                static_cast<uint32_t>(
                    stats.checksumErrors + stats.invalidPackets
                ),
                hoverFresh
                    ? (hover.enabled ? "hover-on" : "hover-off")
                    : "hover-stale"
            );
        }

        vTaskDelay(pdMS_TO_TICKS(kReceivePollIntervalMs));
    }
}

} // namespace

void configure(const softapp::wifi::AccessPointConfig &config)
{
    sConfig = config;
    sReceiver.setCommandHandler(handleCommand);
    sReceiver.setHoverHandler(handleHover);
    sReceiver.setArmHandler(handleArm);
    sReceiver.setPidSettingsHandler(handlePidSettings);
    sReceiver.setCalibrationHandler(handleCalibration);
}

void setCommandHandler(
    softapp::wifi::WiFiReceiver::CommandHandler handler
)
{
    sUserCommandHandler = handler;
}

void setHoverHandler(
    softapp::wifi::WiFiReceiver::HoverHandler handler
)
{
    sUserHoverHandler = handler;
}

void setArmHandler(
    softapp::wifi::WiFiReceiver::ArmHandler handler
)
{
    sUserArmHandler = handler;
}

softapp::wifi::RemoteCommand latestCommand()
{
    return sReceiver.latestCommand();
}

softapp::wifi::RemoteHover latestHover()
{
    return sReceiver.latestHover();
}

softapp::wifi::RemoteArm latestArm()
{
    return sReceiver.latestArm();
}

softapp::wifi::ReceiverStats receiverStats()
{
    return sReceiver.stats();
}

bool hasPeer()
{
    return sReceiver.hasPeer();
}

IPAddress peerIp()
{
    return sReceiver.peerIp();
}

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
)
{
    if (handle == nullptr || *handle != nullptr)
    {
        return pdFAIL;
    }

    sReceiver.setCommandHandler(handleCommand);
    sReceiver.setHoverHandler(handleHover);
    sReceiver.setArmHandler(handleArm);
    sReceiver.setPidSettingsHandler(handlePidSettings);
    return xTaskCreatePinnedToCore(
        entry,
        "wifi",
        stackBytes,
        nullptr,
        priority,
        handle,
        core
    );
}

void stop(TaskHandle_t *handle)
{
    if (handle == nullptr)
    {
        return;
    }

    if (*handle != nullptr)
    {
        vTaskDelete(*handle);
        *handle = nullptr;
    }

    sReceiver.end();
    sService.end();
}

} // namespace wifi_task
} // namespace task_group
