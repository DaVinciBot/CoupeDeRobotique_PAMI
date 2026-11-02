// Découpe un flux d'octets (UART) en trames :
//   | message | taille | CRC-8 | BA DD 1C C5 |
// Même principe que usb_com : on accumule les octets jusqu'à voir la
// signature, puis on vérifie taille et CRC (PamiFrame::decode).
// Sans dépendance à Arduino : testable sur PC.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "PamiFrame.h"

class FrameParser {
   public:
    // Ajoute un octet. Renvoie la taille du message si une trame valide
    // vient de se terminer (le message est alors dans message()), sinon 0.
    size_t feed(uint8_t byte);
    const uint8_t* message() const { return _msg; }

    uint32_t badFrames() const { return _bad; }

   private:
    static constexpr size_t MAX_FRAME = pami_frame::MAX_MESSAGE + pami_frame::OVERHEAD;
    uint8_t _buf[MAX_FRAME] = {};
    size_t _len = 0;
    uint8_t _msg[pami_frame::MAX_MESSAGE] = {};
    uint32_t _bad = 0;
};
