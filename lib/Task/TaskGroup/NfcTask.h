#ifndef NFC_TASK_H
#define NFC_TASK_H

#include <FreeRTOS.h>
#include <task.h>

#include "../../Hardware/NFC/NFC.h"
#include "../../SoftApp/Communication/SPI/RC522_SPI/RC522_SPI.h"

namespace task_group
{
namespace nfc_task
{

// 在启动任务前注入实际 GPIO 和 SPI 参数。
void configure(const rc522::RC522_SPI::Config &config);

// 可在业务层注册卡片到达回调；该回调在 NFC 任务上下文中执行。
void setCardHandler(hardware::nfc::CardHandler handler);

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
);
extern bool Stability;//停留标志位
void stop(TaskHandle_t *handle);

} // namespace nfc_task
} // namespace task_group

#endif
