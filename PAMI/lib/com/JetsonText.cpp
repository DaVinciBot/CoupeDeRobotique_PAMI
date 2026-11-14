#include "JetsonText.h"

#include <math.h>
#include <string.h>

#include "PamiMessages.h"

namespace jetson_text {

namespace {

constexpr float PI_F = 3.14159265f;

// Entier strict (signe autorisé) entre `p` et le prochain '|' ou la fin.
// Avance `p` après le séparateur. false si vide ou non numérique.
bool readInt(const char*& p, const char* end, int& out) {
    bool neg = false;
    if (p < end && (*p == '-' || *p == '+')) {
        neg = *p == '-';
        ++p;
    }
    long v = 0;
    int digits = 0;
    while (p < end && *p != '|') {
        if (*p < '0' || *p > '9' || digits >= 9) return false;
        v = v * 10 + (*p - '0');
        ++digits;
        ++p;
    }
    if (digits == 0) return false;
    if (p < end) ++p;  // saute le '|'
    out = (int)(neg ? -v : v);
    return true;
}

}  // namespace

int parseVision(const char* line, size_t len, Marker* out, size_t max) {
    const char* p = line;
    const char* end = line + len;
    int cmd;
    if (!readInt(p, end, cmd) || cmd != 5) return -1;
    size_t count = 0;
    while (p < end) {
        Marker m;
        if (!readInt(p, end, m.id) || !readInt(p, end, m.xCm) || !readInt(p, end, m.yCm) ||
            !readInt(p, end, m.yawDeg)) {
            return -1;  // trame incomplète ou abîmée : rien n'est gardé
        }
        if (count < max) out[count++] = m;
    }
    return count > 0 ? (int)count : -1;
}

size_t Reassembler::feed(const uint8_t* msg, size_t size) {
    if (size < 3 || msg[0] != pami_msg::JETSON_TEXT) return 0;
    int index = msg[1];
    bool last = msg[2] != 0;
    size_t n = size - 3;

    if (index == 0) {
        _len = 0;
        _expected = 0;
    }
    if (_expected < 0 || index != _expected || _len + n > MAX_TEXT) {
        _expected = -1;
        return 0;
    }
    memcpy(_buf + _len, msg + 3, n);
    _len += n;
    ++_expected;
    if (!last) return 0;
    _buf[_len] = '\0';
    _expected = -1;
    return _len;
}

void toTable(const Marker& m, const FrameConfig& c, float& xMm, float& yMm, float& theta) {
    xMm = m.xCm * 10.0f;
    yMm = m.yCm * 10.0f;
    float yaw = m.yawDeg * PI_F / 180.0f;
    float dx = cosf(yaw), dy = sinf(yaw);
    if (c.mirrorX) {
        xMm = c.lengthMm - xMm;
        dx = -dx;
    }
    if (c.mirrorY) {
        yMm = c.widthMm - yMm;
        dy = -dy;
    }
    theta = atan2f(dy, dx);
}

}  // namespace jetson_text
