#ifndef TASKRUN_H
#define TASKRUN_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

// 创建项目中的全部 FreeRTOS 任务。
bool StartTask();

// 删除已经创建的任务并释放其占用的资源。
void StopTask();

bool IsTaskRunning();

#endif
