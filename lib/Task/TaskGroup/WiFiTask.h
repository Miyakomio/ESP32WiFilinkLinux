#ifndef WIFI_TASK_H
#define WIFI_TASK_H

#include <FreeRTOS.h>
#include <task.h>

#include "../../SoftApp/Network/WiFi/WiFiReceiver.h"
#include "../../SoftApp/Network/WiFi/WiFiService.h"

namespace task_group
{
namespace wifi_task
{

void configure(const softapp::wifi::AccessPointConfig &config);
void setCommandHandler(
    softapp::wifi::WiFiReceiver::CommandHandler handler
);
void setHoverHandler(
    softapp::wifi::WiFiReceiver::HoverHandler handler
);
void setArmHandler(
    softapp::wifi::WiFiReceiver::ArmHandler handler
);

softapp::wifi::RemoteCommand latestCommand();
softapp::wifi::RemoteHover latestHover();
softapp::wifi::RemoteArm latestArm();
softapp::wifi::ReceiverStats receiverStats();
bool hasPeer();
IPAddress peerIp();

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
);

void stop(TaskHandle_t *handle);

} // namespace wifi_task
} // namespace task_group

#endif
