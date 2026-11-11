#include "main.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

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
