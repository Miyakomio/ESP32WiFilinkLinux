#include "MpuTask.h"

#include <Wire.h>

namespace task_group
{
namespace mpu_task
{
namespace
{

constexpr uint32_t kRetryIntervalMs = 2000;

Config sConfig;
softapp::iic::Bus sBus;
softapp::mpu6050::MPU6050_IIC sSensor;
DataHandler sDataHandler = nullptr;
bool sConfigured = false;
DataSnapshot sLatestData;
portMUX_TYPE sDataMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE sCalibrationMux = portMUX_INITIALIZER_UNLOCKED;
bool sCalibrationRequested = false;
bool sCalibrationActive = false;

void entry(void *context)
{
    (void)context;

    bool started = false;
    bool sampling = false;
    uint32_t lastStatusMs = 0;
    static uint8_t mpuWarningCount=0;
    for (;;)
    {
        if (!sConfig.sampleIntervalMs)
        {
            vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
            continue;
        }

        if (!started)
        {
            // SDA=GPIO10，SCL=GPIO11。
            const softapp::iic::Result busResult =
                sBus.begin(Wire, sConfig.iic);

            if (busResult != softapp::iic::Result::Ok && mpuWarningCount > 3)
            {
                Serial.printf(
                    "MPU6050 I2C 初始化失败: %s\n",
                    softapp::iic::toString(busResult)
                );
                mpuWarningCount++;
                vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                continue;
            }

            const softapp::mpu6050::Result sensorResult =
                sSensor.begin(sBus, sConfig.sensor);

            if (sensorResult != softapp::mpu6050::Result::Ok)
            {
                Serial.printf(
                    "MPU6050 初始化失败: %s\n",
                    softapp::mpu6050::toString(sensorResult)
                );
                sBus.end();
                vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                continue;
            }

            Serial.printf(
                "MPU6050 初始化成功，WHO_AM_I=0x%02X\n",
                sSensor.whoAmI()
            );
            mpuWarningCount = 0;


            if (sConfig.gyroCalibrationSamples > 0)
            {
                Serial.println("MPU6050 陀螺仪校准中，请保持静止");
                if (sSensor.calibrateGyro(
                        sConfig.gyroCalibrationSamples) !=
                    softapp::mpu6050::Result::Ok)
                {
                    Serial.println("MPU6050 陀螺仪校准失败");
                    sBus.end();
                    vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                    continue;
                }
            }

            Serial.println("MPU6050 采样开始");
            started = true;
            sampling = true;
        }

        bool runCalibration = false;
        portENTER_CRITICAL(&sCalibrationMux);
        if (sCalibrationRequested)
        {
            sCalibrationRequested = false;
            sCalibrationActive = true;
            runCalibration = true;
        }
        portEXIT_CRITICAL(&sCalibrationMux);

        if (runCalibration)
        {
            portENTER_CRITICAL(&sDataMux);
            sLatestData.valid = false;
            portEXIT_CRITICAL(&sDataMux);

            Serial.println("MPU calibration started: keep level and still");
            const softapp::mpu6050::Result calibrationResult =
                sSensor.calibrateLevelAndGyro(
                    sConfig.gyroCalibrationSamples
                );
            if (calibrationResult == softapp::mpu6050::Result::Ok)
            {
                const float *bias = sSensor.gyroBiasDps();
                Serial.printf(
                    "MPU calibration complete: G=[%.3f %.3f %.3f]dps\n",
                    bias[0],
                    bias[1],
                    bias[2]
                );
            }
            else
            {
                Serial.printf(
                    "MPU calibration failed: %s\n",
                    softapp::mpu6050::toString(calibrationResult)
                );
            }

            portENTER_CRITICAL(&sCalibrationMux);
            sCalibrationActive = false;
            portEXIT_CRITICAL(&sCalibrationMux);
            continue;
        }

        softapp::mpu6050::Data data;
        const softapp::mpu6050::Result result = sSensor.read(data);

        if (result == softapp::mpu6050::Result::Ok)
        {
            const uint32_t now = millis();
            portENTER_CRITICAL(&sDataMux);
            sLatestData.data = data;
            sLatestData.receivedAtMs = now;
            sLatestData.valid = true;
            portEXIT_CRITICAL(&sDataMux);

            if (sDataHandler != nullptr)
            {
                sDataHandler(data);
            }

            if (sConfig.statusIntervalMs > 0 &&
                now - lastStatusMs >= sConfig.statusIntervalMs)
            {
                lastStatusMs = now;
                Serial.printf(
                    "MPU: Roll=%.2f Pitch=%.2f Yaw=%.2f | "
                    "A=[%.2f %.2f %.2f]g G=[%.1f %.1f %.1f]dps\n",
                    data.rollDeg,
                    data.pitchDeg,
                    data.yawDeg,
                    data.accelG[0],
                    data.accelG[1],
                    data.accelG[2],
                    data.gyroDps[0],
                    data.gyroDps[1],
                    data.gyroDps[2]
                );
            }
        }
        else
        {
            Serial.printf(
                "MPU6050 读取失败: %s\n",
                softapp::mpu6050::toString(result)
            );
            started = false;
            sampling = false;
            sBus.end();
            portENTER_CRITICAL(&sDataMux);
            sLatestData.valid = false;
            portEXIT_CRITICAL(&sDataMux);
        }

        if (sampling)
        {
            vTaskDelay(pdMS_TO_TICKS(sConfig.sampleIntervalMs));
        }
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

void setDataHandler(DataHandler handler)
{
    sDataHandler = handler;
}

DataSnapshot latestData()
{
    portENTER_CRITICAL(&sDataMux);
    const DataSnapshot snapshot = sLatestData;
    portEXIT_CRITICAL(&sDataMux);
    return snapshot;
}

bool requestCalibration()
{
    if (!sConfigured)
    {
        return false;
    }

    bool accepted = false;
    portENTER_CRITICAL(&sCalibrationMux);
    if (!sCalibrationActive)
    {
        sCalibrationRequested = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&sCalibrationMux);

    if (accepted)
    {
        portENTER_CRITICAL(&sDataMux);
        sLatestData.valid = false;
        portEXIT_CRITICAL(&sDataMux);
    }
    return accepted;
}

bool isCalibrating()
{
    portENTER_CRITICAL(&sCalibrationMux);
    const bool calibrating =
        sCalibrationRequested || sCalibrationActive;
    portEXIT_CRITICAL(&sCalibrationMux);
    return calibrating;
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
        "mpu6050",
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
}

} // namespace mpu_task
} // namespace task_group
