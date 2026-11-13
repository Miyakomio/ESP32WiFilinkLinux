#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <FreeRTOS.h>
#include <task.h>

#include "../../Hardware/MOTOR/motor.h"

namespace task_group
{
namespace motor_task
{

struct Config
{
    hardware::motor::Config controller{};
    uint32_t outputIntervalMs = 2;
    uint32_t commandTimeoutMs = 2000;//超时时间
};

void configure(const Config &config);
bool isConfigured();

bool unlock();
void lock();
bool isLocked();

void setSpeed(uint8_t index, int16_t speed);
void setCommand(const hardware::motor::motor &command);
hardware::motor::motor command();

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
);

void stop(TaskHandle_t *handle);

} // namespace motor_task
} // namespace task_group

#endif
