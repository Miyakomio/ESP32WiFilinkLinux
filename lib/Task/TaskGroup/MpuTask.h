#ifndef MPU_TASK_H
#define MPU_TASK_H

#include <FreeRTOS.h>
#include <task.h>

#include "../../SoftApp/Communication/IIC/iic.h"
#include "../../SoftApp/Communication/IIC/MPU6050_IIC/mpu6050_iic.h"

namespace task_group
{
namespace mpu_task
{

struct Config
{
    softapp::iic::Config iic{};
    softapp::mpu6050::Config sensor{};
    uint16_t gyroCalibrationSamples = 2000;
    uint32_t sampleIntervalMs = 10;
    uint32_t statusIntervalMs = 200;
};

using DataHandler = void (*)(const softapp::mpu6050::Data &data);

struct DataSnapshot
{
    softapp::mpu6050::Data data;
    uint32_t receivedAtMs = 0;
    bool valid = false;
};

void configure(const Config &config);
bool isConfigured();
void setDataHandler(DataHandler handler);
DataSnapshot latestData();
bool requestCalibration();
bool isCalibrating();

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
);

void stop(TaskHandle_t *handle);

} // namespace mpu_task
} // namespace task_group

#endif
