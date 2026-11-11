#ifndef HARDWARE_NFC_H
#define HARDWARE_NFC_H

#include <stdint.h>

namespace hardware
{
namespace nfc
{

constexpr uint8_t kMaxUidLength = 10;

struct Card
{
    uint8_t uid[kMaxUidLength] = {};
    uint8_t uidLength = 0;
    uint16_t atqa = 0;
    uint8_t sak = 0;
};

using CardHandler = void (*)(const Card &card);

} // namespace nfc
} // namespace hardware

#endif
