#include "FrameParser.h"

#include <string.h>

#include "PamiMessages.h"

size_t FrameParser::feed(uint8_t byte) {
    if (_len >= MAX_FRAME) {  // trop long sans signature : on jette le début
        memmove(_buf, _buf + 1, MAX_FRAME - 1);
        _len = MAX_FRAME - 1;
    }
    _buf[_len++] = byte;
    if (_len < pami_frame::OVERHEAD + 1) return 0;
    if (memcmp(_buf + _len - 4, pami_msg::SIGNATURE, 4) != 0) return 0;

    // Signature reçue : la trame est faite des (taille + 6) derniers octets
    size_t size = _buf[_len - 6];
    size_t frameLen = size + pami_frame::OVERHEAD;
    size_t ok = 0;
    if (size > 0 && frameLen <= _len) {
        const uint8_t* frame = _buf + _len - frameLen;
        ok = pami_frame::decode(frame, frameLen);
        if (ok) memcpy(_msg, frame, ok);
    }
    if (!ok) _bad++;
    _len = 0;  // repartir de zéro après chaque signature
    return ok;
}
