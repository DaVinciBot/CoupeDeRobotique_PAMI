/*
 * Trames des PAMI, côté passerelle (même format que lib/com/PamiFrame.h et
 * FrameParser.h sur l'ESP32 : à garder alignés) :
 *
 *   | message (taille octets) | taille (1 octet) | CRC-8 (poly 0x07) | BA DD 1C C5 |
 *
 * Le CRC-8 porte sur le message et l'octet de taille.
 */
#ifndef FRAME_H_
#define FRAME_H_

#include <stddef.h>
#include <stdint.h>

#define FRAME_OVERHEAD    (6u)    /* taille + CRC + signature */
#define FRAME_MAX_MESSAGE (250u)
#define FRAME_MAX         (FRAME_MAX_MESSAGE + FRAME_OVERHEAD)

uint8_t frame_crc8(const uint8_t *data, size_t len);

/* Écrit la trame dans out (au moins size + FRAME_OVERHEAD octets).
 * Renvoie sa longueur, 0 si le message est vide ou trop long. */
size_t frame_encode(const uint8_t *message, size_t size, uint8_t *out);

/* Vérifie une trame complète (un datagramme UDP). Renvoie la taille du
 * message, 0 si la trame est invalide (taille, CRC, signature). */
size_t frame_decode(const uint8_t *frame, size_t len);

/* Découpe un flux d'octets (UART) en trames. */
typedef struct
{
    uint8_t buf[FRAME_MAX];
    size_t len;
    uint8_t msg[FRAME_MAX_MESSAGE];
    uint32_t bad_frames;
} frame_parser_t;

void frame_parser_init(frame_parser_t *p);

/* Ajoute un octet. Renvoie la taille du message si une trame valide vient
 * de se terminer (le message est alors dans p->msg), sinon 0. */
size_t frame_parser_feed(frame_parser_t *p, uint8_t byte);

#endif /* FRAME_H_ */
