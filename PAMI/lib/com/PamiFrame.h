// Trame d'un message (même format que usb_com / com_pami) :
//
//   | message (taille octets) | taille | CRC-8 | BA DD 1C C5 |
//
// Le CRC-8 (polynôme 0x07, comme la table de crc.h) porte sur le message et
// l'octet de taille. En UDP, un datagramme = une trame.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace pami_frame {

constexpr size_t OVERHEAD = 6;  // taille + CRC + signature
constexpr size_t MAX_MESSAGE = 250;

uint8_t crc8(const uint8_t* data, size_t len);

// Écrit la trame dans `out` (au moins size + OVERHEAD octets). Renvoie sa
// longueur, 0 si le message est vide ou trop long.
size_t encode(const void* message, size_t size, uint8_t* out);

// Vérifie une trame reçue. Renvoie la taille du message (qui commence au
// début de `frame`), 0 si la trame est invalide (taille, CRC, signature).
size_t decode(const uint8_t* frame, size_t len);

}  // namespace pami_frame
