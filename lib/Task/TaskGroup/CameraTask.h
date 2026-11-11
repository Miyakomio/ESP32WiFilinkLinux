#ifndef CAMERA_TASK_H
#define CAMERA_TASK_H

#include <FreeRTOS.h>
#include <task.h>

#include "esp_camera.h"

namespace task_group
{
namespace camera_task
{

struct Config
{
    int pwdnPin = -1;
    int resetPin = 43;
    int xclkPin = 6;
    int sccbPort = 1;
    int d7Pin = 48;
    int d6Pin = 7;
    int d5Pin = 15;
    int d4Pin = 17;
    int d3Pin = 8;
    int d2Pin = 46;
    int d1Pin = 9;
    int d0Pin = 18;
    int vsyncPin = 44;
    int hrefPin = 2;
    int pclkPin = 16;
    framesize_t frameSize = FRAMESIZE_QVGA;
    int jpegQuality = 16;
    uint32_t frameIntervalMs = 100;
};

void configure(const Config &config);
bool isConfigured();

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
);

void stop(TaskHandle_t *handle);

} // namespace camera_task
} // namespace task_group

#endif
