#include "PamiFrame.h"

#include <string.h>

#include "PamiMessages.h"

namespace pami_frame {

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

size_t encode(const void* message, size_t size, uint8_t* out) {
    if (message == nullptr || size == 0 || size > MAX_MESSAGE) return 0;
    memcpy(out, message, size);
    out[size] = (uint8_t)size;
    out[size + 1] = crc8(out, size + 1);
    memcpy(out + size + 2, pami_msg::SIGNATURE, 4);
    return size + OVERHEAD;
}

size_t decode(const uint8_t* frame, size_t len) {
    if (len < OVERHEAD + 1) return 0;
    if (memcmp(frame + len - 4, pami_msg::SIGNATURE, 4) != 0) return 0;
    size_t size = frame[len - 6];
    if (size == 0 || size + OVERHEAD != len) return 0;
    if (crc8(frame, size + 1) != frame[len - 5]) return 0;
    return size;
}

}  // namespace pami_frame
