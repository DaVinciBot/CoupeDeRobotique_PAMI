/* Trames des PAMI : voir frame.h. Portage C de lib/com/PamiFrame.cpp et
 * lib/com/FrameParser.cpp. */
#include "frame.h"

#include <string.h>

static const uint8_t SIGNATURE[4] = { 0xBA, 0xDD, 0x1C, 0xC5 };

uint8_t frame_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

size_t frame_encode(const uint8_t *message, size_t size, uint8_t *out)
{
    if ((message == NULL) || (size == 0u) || (size > FRAME_MAX_MESSAGE))
    {
        return 0;
    }
    memcpy(out, message, size);
    out[size] = (uint8_t)size;
    out[size + 1u] = frame_crc8(out, size + 1u);
    memcpy(out + size + 2u, SIGNATURE, sizeof(SIGNATURE));
    return size + FRAME_OVERHEAD;
}

size_t frame_decode(const uint8_t *frame, size_t len)
{
    if (len < FRAME_OVERHEAD + 1u)
    {
        return 0;
    }
    if (memcmp(frame + len - 4u, SIGNATURE, sizeof(SIGNATURE)) != 0)
    {
        return 0;
    }
    size_t size = frame[len - 6u];
    if ((size == 0u) || (size + FRAME_OVERHEAD != len))
    {
        return 0;
    }
    if (frame_crc8(frame, size + 1u) != frame[len - 5u])
    {
        return 0;
    }
    return size;
}

void frame_parser_init(frame_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

size_t frame_parser_feed(frame_parser_t *p, uint8_t byte)
{
    if (p->len >= FRAME_MAX)  /* trop long sans signature : on jette le début */
    {
        memmove(p->buf, p->buf + 1, FRAME_MAX - 1u);
        p->len = FRAME_MAX - 1u;
    }
    p->buf[p->len++] = byte;
    if (p->len < FRAME_OVERHEAD + 1u)
    {
        return 0;
    }
    if (memcmp(p->buf + p->len - 4u, SIGNATURE, sizeof(SIGNATURE)) != 0)
    {
        return 0;
    }

    /* Signature reçue : la trame est faite des (taille + 6) derniers octets */
    size_t size = p->buf[p->len - 6u];
    size_t frame_len = size + FRAME_OVERHEAD;
    size_t ok = 0;
    if ((size > 0u) && (frame_len <= p->len))
    {
        const uint8_t *frame = p->buf + p->len - frame_len;
        ok = frame_decode(frame, frame_len);
        if (ok != 0u)
        {
            memcpy(p->msg, frame, ok);
        }
    }
    if (ok == 0u)
    {
        p->bad_frames++;
    }
    p->len = 0;  /* repartir de zéro après chaque signature */
    return ok;
}
