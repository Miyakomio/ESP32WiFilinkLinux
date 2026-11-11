#ifndef BATTERY_TASK_H
#define BATTERY_TASK_H

#include <FreeRTOS.h>
#include <task.h>

#include "../../Hardware/MAX1704/MAX1704.h"
#include "../../SoftApp/Communication/IIC/iic.h"

namespace task_group
{
namespace battery_task
{

struct Config
{
    softapp::iic::Config iic{};
    hardware::max1704::Model model =
        hardware::max1704::Model::MAX17048;
    uint32_t sampleIntervalMs = 2000;
};

struct DataSnapshot
{
    hardware::max1704::Reading reading;
    uint32_t receivedAtMs = 0;
    bool valid = false;
};

void configure(const Config &config);
bool isConfigured();
DataSnapshot latestData();

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
);

void stop(TaskHandle_t *handle);

} // namespace battery_task
} // namespace task_group

#endif
