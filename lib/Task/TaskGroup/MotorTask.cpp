#include "MotorTask.h"

#include "MpuTask.h"
#include "WiFiTask.h"
#include "../../SoftApp/Control/MotorControl.h"

namespace task_group
{
namespace motor_task
{
namespace
{

constexpr uint32_t kRetryIntervalMs = 1000;

Config sConfig;
hardware::motor::MotorController sController;
hardware::motor::motor sCommand;
portMUX_TYPE sCommandMux = portMUX_INITIALIZER_UNLOCKED;
bool sConfigured = false;
bool sArmed = false;
uint32_t sLastCommandMs = 0;

void entry(void *context)
{
    (void)context;

    bool started = false;

    for (;;)
    {
        if (!sConfigured)
        {
            vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
            continue;
        }

        if (!started)
        {
            started = sController.begin(sConfig.controller);//启动电机
            if (!started)
            {
                Serial.println("电机 PWM 初始化失败");
                vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                continue;
            }

            Serial.println("电机 PWM 初始化成功，当前处于停止状态");
        }

        bool controlValid = false;
        const softapp::wifi::RemoteArm arm =
            wifi_task::latestArm();
        const bool armSafe =
            softapp::wifi::WiFiReceiver::isArmFresh(arm, 500) &&
            arm.enabled;

        if (!armSafe)
        {
            lock();
        }

        if (!isLocked())
        {
            const uint32_t now = millis();
            const mpu_task::DataSnapshot imu =
                mpu_task::latestData();
            const softapp::wifi::RemoteCommand remote =
                wifi_task::latestCommand();

            static uint32_t lastControlMs = 0;
            float dt = 0.02f;
            if (lastControlMs != 0)
            {
                dt = static_cast<float>(
                    now - lastControlMs
                ) / 1000.0f;
            }
            lastControlMs = now;

            if (dt <= 0.0f || dt > 0.1f)
            {
                dt = 0.02f;
            }

            hardware::motor::motor controlOutput;
            if (imu.valid && now - imu.receivedAtMs <= 250)
            {
                controlValid = MotorControl(remote,imu.data,dt,controlOutput);//电机控制
            }

            if (controlValid)
            {
                setCommand(controlOutput);
            }
            else
            {
                resetMotorControl();
            }
        }
        else
        {
            resetMotorControl();
        }

        hardware::motor::motor output;
        bool armed = false;
        uint32_t lastCommandMs = 0;

        portENTER_CRITICAL(&sCommandMux);
        output = sCommand;
        armed = sArmed;
        lastCommandMs = sLastCommandMs;
        portEXIT_CRITICAL(&sCommandMux);

        if (!armed ||
            (sConfig.commandTimeoutMs > 0 &&
             millis() - lastCommandMs > sConfig.commandTimeoutMs))//指令超时或未操作将触发悬停并返回此时的经纬度
        {
            int16_t now_speed[4]={};
            //悬停代码::待补充
            for (uint8_t i = 0; i < hardware::motor::kMotorCount; ++i)
            {
                output.speed[i] = hardware::motor::kStopSpeed;
            }

            // if (armed)
            // {
            //     portENTER_CRITICAL(&sCommandMux);
            //     sArmed = false;
            //     portEXIT_CRITICAL(&sCommandMux);
            //     Serial.println("电机命令超时，已自动停机");
            // }
            

        }

        if (!controlValid)
        {
            for (uint8_t i = 0;
                 i < hardware::motor::kMotorCount;
                 ++i)
            {
                output.speed[i] = hardware::motor::kStopSpeed;
            }
        }

        sController.setSpeeds(output);
        static uint32_t lastMotorLogMs = 0;
        if (armed && millis() - lastMotorLogMs >= 500)
        {
            lastMotorLogMs = millis();
            Serial.printf(
                "MOTOR out=%d,%d,%d,%d control=%s\n",
                output.speed[0],
                output.speed[1],
                output.speed[2],
                output.speed[3],
                controlValid ? "on" : "off"
            );
        }
        vTaskDelay(pdMS_TO_TICKS(sConfig.outputIntervalMs));
    }
}

} // namespace

void configure(const Config &config)
{
    configureMotorPid();

    portENTER_CRITICAL(&sCommandMux);
    sConfig = config;
    sCommand = hardware::motor::motor{};
    sArmed = false;
    sLastCommandMs = millis();
    sConfigured = true;
    portEXIT_CRITICAL(&sCommandMux);
}

bool isConfigured()
{
    return sConfigured;
}
//解锁
bool unlock()
{
    if (!sConfigured)
    {
        return false;
    }

    portENTER_CRITICAL(&sCommandMux);
    sArmed = true;
    sLastCommandMs = millis();
    portEXIT_CRITICAL(&sCommandMux);
    return true;
}

//锁定
void lock()
{
    portENTER_CRITICAL(&sCommandMux);
    sArmed = false;
    for (uint8_t i = 0; i < hardware::motor::kMotorCount; ++i)
    {
        sCommand.speed[i] = hardware::motor::kStopSpeed;
    }
    portEXIT_CRITICAL(&sCommandMux);
}

//是否锁定
bool isLocked()
{
    portENTER_CRITICAL(&sCommandMux);
    const bool armed = sArmed;
    portEXIT_CRITICAL(&sCommandMux);
    return !armed;
}

void setSpeed(uint8_t index, int16_t speed)
{
    if (index >= hardware::motor::kMotorCount)
    {
        return;
    }

    portENTER_CRITICAL(&sCommandMux);
    sCommand.speed[index] = speed;
    sLastCommandMs = millis();
    portEXIT_CRITICAL(&sCommandMux);
}

void setCommand(const hardware::motor::motor &command)
{
    portENTER_CRITICAL(&sCommandMux);
    sCommand = command;
    sLastCommandMs = millis();
    portEXIT_CRITICAL(&sCommandMux);
}

hardware::motor::motor command()
{
    portENTER_CRITICAL(&sCommandMux);
    const hardware::motor::motor current = sCommand;
    portEXIT_CRITICAL(&sCommandMux);
    return current;
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
        "motor",
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

    lock();

    if (*handle != nullptr)
    {
        vTaskDelete(*handle);
        *handle = nullptr;
    }

    sController.end();
}

} // namespace motor_task
} // namespace task_group
