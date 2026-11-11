#include "NfcTask.h"
#include <SPI.h>
#include <string.h>

namespace task_group
{
namespace nfc_task
{
namespace
{

constexpr uint32_t kPollIntervalMs = 200;
constexpr uint32_t kRetryIntervalMs = 500;
constexpr uint32_t kErrorLogIntervalMs = 3000;

rc522::RC522_SPI sReader;
rc522::RC522_SPI::Config sConfig;
hardware::nfc::CardHandler sCardHandler = nullptr;
bool sConfigured = false;
constexpr uint8_t FLY1ID[4] = {0x88, 0x04, 0x97, 0x86};//1号机ID
constexpr uint8_t FLY2ID[4] = {89, 05, 98, 87}; // 暂定
constexpr uint8_t FLY3ID[4] = {90, 06, 99, 88}; // 暂定
constexpr uint8_t FLY4ID[4] = {91, 07, 99, 89}; // 暂定
bool Stability = false;
void entry(void *context)
{
    (void)context;

    bool readerStarted = false;
    uint32_t lastInitErrorLogMs = 0;
    for (;;)
    {
        if (!sConfigured)
        {
            vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
            continue;
        }

        //重连
        if (!readerStarted)
        {
            readerStarted = sReader.begin(SPI, sConfig);
            if (readerStarted)
            {
                Serial.printf(
                    "RC522 初始化成功，版本: 0x%02X\n",
                    sReader.version()
                );
            }
        }

        if (!readerStarted)
        {
            if (millis() - lastInitErrorLogMs >= kErrorLogIntervalMs)
            {
                Serial.println(
                    "RC522 初始化失败，请检查 3.3V、GND 和 SPI 引脚"
                );
                lastInitErrorLogMs = millis();
            }
            vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
            continue;
        }

        uint8_t uid[4] = {};
        uint16_t atqa = 0;
        uint8_t sak = 0;
        static uint8_t nfcWarningCount=0;
        static uint8_t TestingCount = 0;//检测计数
        // WUPA 可以唤醒上一轮 halt() 的卡片，便于持续检测。
        const rc522::Result result =
            sReader.readCard(
                uid,
                atqa,
                sak,
                rc522::RequestMode::All
            );

        if (result == rc522::Result::Ok)//感应成功
        {
            hardware::nfc::Card card;
            memcpy(card.uid, uid, sizeof(uid));
            card.uidLength = sizeof(uid);
            card.atqa = atqa;
            card.sak = sak;

            if (sCardHandler != nullptr)
            {
                sCardHandler(card);
            }
            else
            {
                Serial.printf(
                    "NFC UID: %02X %02X %02X %02X\n",
                    card.uid[0],
                    card.uid[1],
                    card.uid[2],
                    card.uid[3]
                );
                if(memcmp(card.uid, FLY1ID, sizeof(FLY1ID)) == 0)
                {
                    Serial.println("检测到 1 号机");
                    TestingCount++;
                    if(TestingCount>=3)//判断停留稳定
                    {
                        Stability = true;
                    }
                }
            }
            nfcWarningCount=0;
            sReader.halt();
        }
        else if(result == rc522::Result::NoCard)//无卡或感应失败
        {
            static uint32_t sendtick=0;
            nfcWarningCount++;
            if(millis()-sendtick>=5000 && nfcWarningCount>3)
            {
                Serial.println("NFC 无卡或感应失败\n");
                sendtick = millis();
                nfcWarningCount = 0;
                TestingCount=0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
    }
}

} // namespace

void configure(const rc522::RC522_SPI::Config &config)
{
    sConfig = config;
    sConfigured = true;
}

void setCardHandler(hardware::nfc::CardHandler handler)
{
    sCardHandler = handler;
}

BaseType_t create(
    TaskHandle_t *handle,
    uint32_t stackBytes,
    UBaseType_t priority,
    BaseType_t core
)
{
    if (handle == nullptr || *handle != nullptr)
    {
        return pdFAIL;
    }

    return xTaskCreatePinnedToCore(
        entry,
        "nfc",
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

    if (*handle != nullptr)
    {
        vTaskDelete(*handle);
        *handle = nullptr;
    }

    sReader.end();
}

} // namespace nfc_task
} // namespace task_group
