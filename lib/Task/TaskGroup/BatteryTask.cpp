#include "BatteryTask.h"
#include <Wire.h>
#include "../../SoftApp/Communication/IIC/MAX1704_IIC/MAX1704_IIC.h"

namespace task_group
{
namespace battery_task
{
namespace
{

constexpr uint32_t kRetryIntervalMs = 2000;
constexpr uint32_t kErrorLogIntervalMs = 5000;
constexpr uint32_t kWarningBlinkIntervalMs = 250;
constexpr uint32_t kWarningPollIntervalMs = 50;
constexpr float kLowBatteryPercent = 10.0f;
constexpr int kBatteryLedPin = 5;

Config sConfig;
softapp::iic::Bus sBus;
softapp::max1704::MAX1704_IIC sGauge;
bool sConfigured = false;
DataSnapshot sLatestData;
portMUX_TYPE sDataMux = portMUX_INITIALIZER_UNLOCKED;


void entry(void *context)
{
    (void)context;

    bool started = false;
    uint32_t lastErrorLogMs = 0;
    uint32_t lastBatteryWarningMs = 0;
    uint32_t nextSampleMs = 0;
    bool lowBattery = false;
    uint8_t ledStatus = 0;
    pinMode(kBatteryLedPin, OUTPUT);
    digitalWrite(kBatteryLedPin, HIGH);//初始关闭

    for (;;)
    {
        if (!sConfigured)
        {
            vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
            continue;
        }

        if (!started)
        {
            const softapp::iic::Result busResult =
                sBus.begin(Wire1, sConfig.iic);

            if (busResult != softapp::iic::Result::Ok)
            {
                Serial.printf(
                    "MAX1704 I2C 初始化失败: %s\n",
                    softapp::iic::toString(busResult)
                );
                vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                continue;
            }

            const softapp::iic::Result gaugeResult =
                sGauge.begin(sBus, sConfig.model);

            if (gaugeResult != softapp::iic::Result::Ok)
            {
                Serial.printf(
                    "MAX1704 未响应: %s\n",
                    softapp::iic::toString(gaugeResult)
                );
                sBus.end();
                vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                continue;
            }

            uint16_t version = 0;
            const softapp::iic::Result versionResult =
                sGauge.readVersion(version);

            Serial.printf(
                "MAX1704 初始化成功: %s, VERSION=0x%04X\n",
                softapp::max1704::toString(sConfig.model),
                static_cast<unsigned>(versionResult == softapp::iic::Result::Ok
                    ? version
                    : 0)
            );
            started = true;
        }
        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextSampleMs) >= 0)
        {
            nextSampleMs = now + sConfig.sampleIntervalMs;
            hardware::max1704::Reading reading;
            const softapp::iic::Result result =
                sGauge.readMeasurement(reading);

            if (result == softapp::iic::Result::Ok)//监测电池数据
            {
                lowBattery =
                    reading.stateOfChargePercent <= kLowBatteryPercent;
                portENTER_CRITICAL(&sDataMux);
                sLatestData.reading = reading;
                sLatestData.receivedAtMs = millis();
                sLatestData.valid = true;
                portEXIT_CRITICAL(&sDataMux);

                Serial.printf(
                    "Battery: %.2f mV, %.2f%%, %.2f%%/h, STATUS=0x%04X\n",
                    reading.cellVoltageMv,
                    reading.stateOfChargePercent,
                    reading.ratePercentPerHour,
                    static_cast<unsigned>(reading.status)
                );
            }
            else
            {
                lowBattery = false;
                portENTER_CRITICAL(&sDataMux);
                sLatestData.valid = false;
                portEXIT_CRITICAL(&sDataMux);

                if (millis() - lastErrorLogMs >= kErrorLogIntervalMs)
                {
                    Serial.printf(
                        "MAX1704 读取失败: %s\n",
                        softapp::iic::toString(result)
                    );
                    lastErrorLogMs = millis();
                }
            }
        }

        //低电量闪烁
        if (lowBattery)
        {
            if (now - lastBatteryWarningMs >= kWarningBlinkIntervalMs)
            {
                lastBatteryWarningMs = now;
                ledStatus = !ledStatus;
                digitalWrite(kBatteryLedPin, ledStatus);
            }
        }
        else
        {
            ledStatus = 0;
            digitalWrite(kBatteryLedPin, HIGH);
        }

        vTaskDelay(pdMS_TO_TICKS(kWarningPollIntervalMs));
    }
}

} // namespace

void configure(const Config &config)
{
    sConfig = config;
    portENTER_CRITICAL(&sDataMux);
    sLatestData = DataSnapshot{};
    portEXIT_CRITICAL(&sDataMux);
    sConfigured = true;
}

bool isConfigured()
{
    return sConfigured;
}

DataSnapshot latestData()
{
    portENTER_CRITICAL(&sDataMux);
    const DataSnapshot snapshot = sLatestData;
    portEXIT_CRITICAL(&sDataMux);
    return snapshot;
}

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
)
{
    if (handle == nullptr || *handle != nullptr || !sConfigured)
    {
        return pdFAIL;
    }

    return xTaskCreatePinnedToCore(
        entry,
        "battery",
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

    sBus.end();
    portENTER_CRITICAL(&sDataMux);
    sLatestData.valid = false;
    portEXIT_CRITICAL(&sDataMux);
    digitalWrite(kBatteryLedPin, LOW);
}

} // namespace battery_task
} // namespace task_group
