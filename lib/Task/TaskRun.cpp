#include "TaskRun.h"
#include <Wire.h>
#include "TaskGroup/BatteryTask.h"
#include "TaskGroup/CameraTask.h"
#include "TaskGroup/MotorTask.h"
#include "TaskGroup/MpuTask.h"
#include "TaskGroup/NfcTask.h"
#include "TaskGroup/WiFiTask.h"

namespace
{

constexpr uint32_t kNfcTaskStack = 4096;
constexpr UBaseType_t kNfcTaskPriority = 1;
constexpr BaseType_t kNfcTaskCore = 1;

constexpr uint32_t kWifiTaskStack = 6144;
constexpr UBaseType_t kWifiTaskPriority = 3;
constexpr BaseType_t kWifiTaskCore = 1;

constexpr uint32_t kBatteryTaskStack = 4096;
constexpr UBaseType_t kBatteryTaskPriority = 1;
constexpr BaseType_t kBatteryTaskCore = 1;

constexpr uint32_t kCameraTaskStack = 8192;
constexpr UBaseType_t kCameraTaskPriority = 2;
constexpr BaseType_t kCameraTaskCore = 1;

TaskHandle_t sNfcTaskHandle = nullptr;
TaskHandle_t sWifiTaskHandle = nullptr;
TaskHandle_t sBatteryTaskHandle = nullptr;
TaskHandle_t sCameraTaskHandle = nullptr;

TaskHandle_t sMotorTaskHandle = nullptr;
constexpr uint32_t kMotorTaskStack = 4096;
constexpr UBaseType_t kMotorTaskPriority = 3;
constexpr BaseType_t kMotorTaskCore = 1;

TaskHandle_t sMpuTaskHandle = nullptr;
constexpr uint32_t kMpuTaskStack = 6144;
constexpr UBaseType_t kMpuTaskPriority = 4;
constexpr BaseType_t kMpuTaskCore = 1;

} // namespace

bool StartTask()
{
    const bool batteryReady =
        !task_group::battery_task::isConfigured() ||
        sBatteryTaskHandle != nullptr;
    const bool motorReady =
        !task_group::motor_task::isConfigured() ||
        sMotorTaskHandle != nullptr;
    const bool mpuReady =
        !task_group::mpu_task::isConfigured() ||
        sMpuTaskHandle != nullptr;
    const bool cameraReady =
        !task_group::camera_task::isConfigured() ||
        sCameraTaskHandle != nullptr;
    if (sWifiTaskHandle != nullptr &&
        batteryReady &&
        motorReady &&
        mpuReady &&
        cameraReady)
    {
        return true;
    }
#if 0
    //DEBUG MODE NFC调试开启
    // RC522_SPI：暂时停用，避免与 MPU 的 GPIO10/11 冲突。
    {
        rc522::RC522_SPI::Config rfidConfig;
        rfidConfig.sckPin = 12;
        rfidConfig.misoPin = 13;
        rfidConfig.mosiPin = 11;
        rfidConfig.csPin = 10;
        rfidConfig.resetPin = 9;    // 不接 RST 写 -1
        rfidConfig.spiClockHz = 4000000;
        task_group::nfc_task::configure(rfidConfig);
    }
#endif

    //MAX1704_IIC
    {
        Wire1.begin(38, 39, 100000);

        task_group::battery_task::Config batteryConfig;
        batteryConfig.iic.sdaPin = 38;  
        batteryConfig.iic.sclPin = 39;  
        batteryConfig.iic.clockHz = 100000;
        batteryConfig.iic.initializeBus = false;
        batteryConfig.model = hardware::max1704::Model::MAX17048;
        batteryConfig.sampleIntervalMs = 2000;
        task_group::battery_task::configure(batteryConfig);
    }

    //MOTOR
    {
        task_group::motor_task::Config motorConfig;
        task_group::motor_task::configure(motorConfig);
    }

    //MPU6050_IIC：SDA=GPIO10，SCL=GPIO11。
    {
        task_group::mpu_task::Config mpuConfig;
        mpuConfig.iic.sdaPin = 10;
        mpuConfig.iic.sclPin = 11;
        mpuConfig.iic.clockHz = 100000;
        mpuConfig.sensor.address =
            softapp::mpu6050::kAddressAd0Low;
        mpuConfig.sensor.sampleRateHz = 100;
        mpuConfig.sampleIntervalMs = 10;
        mpuConfig.gyroCalibrationSamples = 2000;
        mpuConfig.statusIntervalMs = 200;
        task_group::mpu_task::configure(mpuConfig);
    }

    // OV3660 camera
    {
        task_group::camera_task::Config cameraConfig;
        task_group::camera_task::configure(cameraConfig);
    }

    if (sMotorTaskHandle == nullptr)
    {
        if (task_group::motor_task::create(
                &sMotorTaskHandle,
                kMotorTaskStack,
                kMotorTaskPriority,
                kMotorTaskCore
            ) != pdPASS)
        {
            return false;
        }
    }

#if 0
    //DEBUG MODE NFC调试开启
    if (sNfcTaskHandle == nullptr)
    {
        if (task_group::nfc_task::create(
                &sNfcTaskHandle,
                kNfcTaskStack,
                kNfcTaskPriority,
                kNfcTaskCore
            ) != pdPASS)
        {
            task_group::wifi_task::stop(&sWifiTaskHandle);
            task_group::motor_task::stop(&sMotorTaskHandle);
            return false;
        }
    }
#endif

    if (sMpuTaskHandle == nullptr)
    {
        if (task_group::mpu_task::create(
                &sMpuTaskHandle,
                kMpuTaskStack,
                kMpuTaskPriority,
                kMpuTaskCore
            ) != pdPASS)
        {
            // NFC 已停用，不调用 nfc_task::stop。
            task_group::motor_task::stop(&sMotorTaskHandle);
            return false;
        }
    }

    
    if (sWifiTaskHandle == nullptr)
    {
        if (task_group::wifi_task::create(
                &sWifiTaskHandle,
                kWifiTaskStack,
                kWifiTaskPriority,
                kWifiTaskCore
            ) != pdPASS)
        {
            // NFC 已停用，不调用 nfc_task::stop。
            task_group::mpu_task::stop(&sMpuTaskHandle);
            task_group::motor_task::stop(&sMotorTaskHandle);
            return false;
        }
    }

    if (sCameraTaskHandle == nullptr)
    {
        if (task_group::camera_task::create(
                &sCameraTaskHandle,
                kCameraTaskStack,
                kCameraTaskPriority,
                kCameraTaskCore
            ) != pdPASS)
        {
            task_group::wifi_task::stop(&sWifiTaskHandle);
            task_group::mpu_task::stop(&sMpuTaskHandle);
            task_group::motor_task::stop(&sMotorTaskHandle);
            return false;
        }
    }

    if (task_group::battery_task::isConfigured() &&
        sBatteryTaskHandle == nullptr)
    {
        if (task_group::battery_task::create(
                &sBatteryTaskHandle,
                kBatteryTaskStack,
                kBatteryTaskPriority,
                kBatteryTaskCore
            ) != pdPASS)
        {
            task_group::motor_task::stop(&sMotorTaskHandle);
            task_group::mpu_task::stop(&sMpuTaskHandle);
            // NFC 已停用，不调用 nfc_task::stop。
            task_group::wifi_task::stop(&sWifiTaskHandle);
            task_group::camera_task::stop(&sCameraTaskHandle);
            return false;
        }
    }

    return true;
}

void StopTask()
{
    task_group::motor_task::stop(&sMotorTaskHandle);
    task_group::mpu_task::stop(&sMpuTaskHandle);
    task_group::camera_task::stop(&sCameraTaskHandle);
    task_group::battery_task::stop(&sBatteryTaskHandle);
    // NFC 已停用，不调用 nfc_task::stop。
    task_group::wifi_task::stop(&sWifiTaskHandle);
    Wire1.end();
}

bool IsTaskRunning()
{
    const bool batteryReady =
        !task_group::battery_task::isConfigured() ||
        sBatteryTaskHandle != nullptr;
    const bool cameraReady =
        !task_group::camera_task::isConfigured() ||
        sCameraTaskHandle != nullptr;
    return sMotorTaskHandle != nullptr &&
        sMpuTaskHandle != nullptr &&
        sWifiTaskHandle != nullptr &&
        batteryReady &&
        cameraReady;
}
