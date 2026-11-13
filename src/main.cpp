#include "main.h"

namespace
{

constexpr bool kEnableMotorSequenceTest = true;
constexpr uint32_t kMotorTestRunMs = 2000;
constexpr uint32_t kMotorTestPauseMs = 1000;

void runMotorSequenceTest()
{
    hardware::motor::MotorController controller;
    hardware::motor::Config config;

    if (!controller.begin(config))
    {
        Serial.println("Motor sequence test: PWM initialization failed");
        return;
    }

    Serial.printf(
        "Motor sequence test: M1-M4 at speed %d, %lu ms each\n",
        static_cast<int>(hardware::motor::kMinSpeed),
        static_cast<unsigned long>(kMotorTestRunMs)
    );

    for (uint8_t motor = 0;
         motor < hardware::motor::kMotorCount;
         ++motor)
    {
        Serial.printf("Motor sequence test: M%u ON\n", motor + 1);
        controller.setSpeed(motor, hardware::motor::kMinSpeed);
        delay(kMotorTestRunMs);

        controller.setSpeed(motor, hardware::motor::kStopSpeed);
        Serial.printf("Motor sequence test: M%u OFF\n", motor + 1);
        delay(kMotorTestPauseMs);
    }

    controller.stopAll();
    controller.end();
    Serial.println("Motor sequence test: complete");
}

} // namespace

void setup()
{
    Serial.begin(115200);
    delay(1000);

    if (kEnableMotorSequenceTest)
    {
        runMotorSequenceTest();
    }

    if (!StartTask())
    {
        Serial.println("任务创建失败");
    }
}

void loop()
{
    // static uint32_t lastTestMs = 0;

    // if (millis() - lastTestMs >= 500)
    // {
    //     lastTestMs = millis();

    //     task_group::motor_task::unlock();//由APP解锁
    //     task_group::motor_task::setSpeed(0, 300);
    //     task_group::motor_task::setSpeed(3, 300);
    // }

    delay(10);
}
