#include "CameraTask.h"

#include <Arduino.h>
#include <WiFiUdp.h>
#include <string.h>

#include "WiFiTask.h"

namespace task_group
{
namespace camera_task
{
namespace
{

constexpr uint32_t kRetryIntervalMs = 2000;
constexpr uint16_t kVideoPort = 5000;
constexpr uint16_t kVideoSourcePort = 5001;
constexpr size_t kMaxPacketSize = 1400;

struct __attribute__((packed)) VideoPacketHeader
{
    uint32_t frameId;
    uint32_t totalSize;
    uint16_t totalPackets;
    uint16_t sequence;
    uint16_t dataLength;
};

Config sConfig;
bool sConfigured = false;
bool sCameraReady = false;
WiFiUDP sVideoUdp;
bool sVideoUdpReady = false;

bool sendFrame(
    const uint8_t *data,
    size_t length,
    uint32_t frameId
)
{
    if (data == nullptr || length == 0 || !wifi_task::hasPeer())
    {
        return false;
    }

    if (!sVideoUdpReady)
    {
        sVideoUdpReady = sVideoUdp.begin(kVideoSourcePort);
        if (!sVideoUdpReady)
        {
            return false;
        }
    }

    const size_t maxPayload = kMaxPacketSize - sizeof(VideoPacketHeader);
    const uint16_t packetCount = static_cast<uint16_t>(
        (length + maxPayload - 1) / maxPayload
    );
    if (packetCount == 0)
    {
        return false;
    }

    const IPAddress peer = wifi_task::peerIp();
    uint8_t packet[kMaxPacketSize];

    for (uint16_t sequence = 0; sequence < packetCount; ++sequence)
    {
        const size_t offset = static_cast<size_t>(sequence) * maxPayload;
        const size_t remaining = length - offset;
        const uint16_t dataLength = static_cast<uint16_t>(
            remaining > maxPayload ? maxPayload : remaining
        );

        VideoPacketHeader header = {
            frameId,
            static_cast<uint32_t>(length),
            packetCount,
            sequence,
            dataLength,
        };

        memcpy(packet, &header, sizeof(header));
        memcpy(packet + sizeof(header), data + offset, dataLength);

        if (!sVideoUdp.beginPacket(peer, kVideoPort))
        {
            return false;
        }
        const size_t packetLength = sizeof(header) + dataLength;
        if (sVideoUdp.write(packet, packetLength) != packetLength ||
            sVideoUdp.endPacket() != 1)
        {
            return false;
        }

        if ((sequence & 3U) == 3U)
        {
            delay(1);
        }
    }

    return true;
}

void entry(void *context)
{
    (void)context;

    camera_config_t cameraConfig{};
    cameraConfig.pin_pwdn = sConfig.pwdnPin;
    cameraConfig.pin_reset = sConfig.resetPin;
    cameraConfig.pin_xclk = sConfig.xclkPin;
    cameraConfig.pin_sccb_sda = -1;
    cameraConfig.pin_sccb_scl = -1;
    cameraConfig.pin_d7 = sConfig.d7Pin;
    cameraConfig.pin_d6 = sConfig.d6Pin;
    cameraConfig.pin_d5 = sConfig.d5Pin;
    cameraConfig.pin_d4 = sConfig.d4Pin;
    cameraConfig.pin_d3 = sConfig.d3Pin;
    cameraConfig.pin_d2 = sConfig.d2Pin;
    cameraConfig.pin_d1 = sConfig.d1Pin;
    cameraConfig.pin_d0 = sConfig.d0Pin;
    cameraConfig.pin_vsync = sConfig.vsyncPin;
    cameraConfig.pin_href = sConfig.hrefPin;
    cameraConfig.pin_pclk = sConfig.pclkPin;
    cameraConfig.xclk_freq_hz = 12000000;
    cameraConfig.ledc_timer = LEDC_TIMER_2;
    cameraConfig.ledc_channel = LEDC_CHANNEL_4;
    cameraConfig.pixel_format = PIXFORMAT_JPEG;
    cameraConfig.frame_size = sConfig.frameSize;
    cameraConfig.jpeg_quality = sConfig.jpegQuality;
    cameraConfig.fb_count = 1;
    cameraConfig.fb_location = CAMERA_FB_IN_DRAM;
    cameraConfig.grab_mode = CAMERA_GRAB_LATEST;
    cameraConfig.sccb_i2c_port = sConfig.sccbPort;

    uint32_t frameId = 0;
    for (;;)
    {
        if (!sConfigured)
        {
            vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
            continue;
        }

        if (!sCameraReady)
        {
            const esp_err_t result = esp_camera_init(&cameraConfig);
            if (result != ESP_OK)
            {
                Serial.printf(
                    "OV3660 camera init failed: %s\n",
                    esp_err_to_name(result)
                );
                vTaskDelay(pdMS_TO_TICKS(kRetryIntervalMs));
                continue;
            }

            sCameraReady = true;
            Serial.println("OV3660 camera initialized: QVGA JPEG");
        }

        if (!wifi_task::hasPeer())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == nullptr)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (frame->format == PIXFORMAT_JPEG &&
            frame->len >= 2 &&
            frame->buf[0] == 0xFF &&
            frame->buf[1] == 0xD8)
        {
            sendFrame(frame->buf, frame->len, ++frameId);
        }

        esp_camera_fb_return(frame);
        vTaskDelay(pdMS_TO_TICKS(sConfig.frameIntervalMs));
    }
}

} // namespace

void configure(const Config &config)
{
    sConfig = config;
    sConfigured = true;
}

bool isConfigured()
{
    return sConfigured;
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
        "camera",
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

    if (sCameraReady)
    {
        esp_camera_deinit();
        sCameraReady = false;
    }

    if (sVideoUdpReady)
    {
        sVideoUdp.stop();
        sVideoUdpReady = false;
    }
}

} // namespace camera_task
} // namespace task_group
