#include "LidarGS2.h"

// Constantes du SDK YDLIDAR (ydlidar_protocol.h) pour le GS2
static constexpr float ANGLE_PX = 1.22f;
static constexpr float ANGLE_PY = 5.315f;
static constexpr float PITCH_DEG = 22.5f;

void LidarGS2::begin(HardwareSerial& serial, const Config& cfg) {
    _serial = &serial;
    _cfg = cfg;
    _serial->setRxBufferSize(2048);  // trames de 331 octets à 921600 bauds
    _serial->begin(_cfg.baud, SERIAL_8N1, _cfg.rxPin, _cfg.txPin);
    delay(100);

    // Même séquence que le SDK YDLIDAR (GSLidarDriver::startScan) :
    // STOP, vider la réception, demander la calibration, puis SCAN.
    // Le STOP sert surtout si l'ESP32 a redémarré alors que le lidar scannait
    // encore : la réponse ne se retrouve pas noyée dans les trames de mesure.
    for (int attempt = 0; attempt < 3 && !_cal.valid; ++attempt) {
        sendCommand(CMD_STOP);
        delay(50);
        while (_serial->available()) _serial->read();
        _syncCount = 0;
        _pos = 0;

        sendCommand(CMD_GET_PARAMETER);
        uint32_t t0 = millis();
        while (!_cal.valid && millis() - t0 < 500) {
            while (_serial->available()) feed((uint8_t)_serial->read());
            delay(1);
        }
    }

    if (_cal.valid) {
        Serial.printf("[Lidar] calibration k0=%.4f b0=%.4f k1=%.4f b1=%.4f bias=%.1f deg\n",
                      _cal.k0, _cal.b0, _cal.k1, _cal.b1, _cal.biasDeg);
    } else {
        Serial.println("[Lidar] pas de calibration : detection par indices fixes");
    }

    sendCommand(CMD_SCAN);
}

void LidarGS2::startTask(uint8_t core) {
    xTaskCreatePinnedToCore(taskEntry, "lidar", 8192, this, 2, nullptr, core);
}

void LidarGS2::taskEntry(void* self) {
    static_cast<LidarGS2*>(self)->taskLoop();
}

void LidarGS2::taskLoop() {
    for (;;) {
        while (_serial->available()) feed((uint8_t)_serial->read());
        // Le lidar ne répond plus : on relance le scan
        if (millis() - _lastFrameMs > 1000 && millis() - _lastCommandMs > 1000) {
            sendCommand(CMD_SCAN);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void LidarGS2::sendCommand(uint8_t cmd) {
    const uint8_t addr = 0x00;
    uint8_t frame[] = {0xA5, 0xA5, 0xA5, 0xA5, addr, cmd, 0x00, 0x00,
                       (uint8_t)(addr + cmd)};
    _serial->write(frame, sizeof(frame));
    _lastCommandMs = millis();
}

void LidarGS2::feed(uint8_t b) {
    // Recherche de l'en-tête A5 A5 A5 A5
    if (_syncCount < 4) {
        _syncCount = (b == 0xA5) ? _syncCount + 1 : 0;
        _pos = 0;
        return;
    }

    // Plus de 4 A5 d'affilée : on reste synchronisé (comme le SDK)
    if (_pos == 0 && b == 0xA5) return;

    switch (_pos) {
        case 0: _addr = b; _sum = b; break;
        case 1: _type = b; _sum += b; break;
        case 2: _len = b; _sum += b; break;
        case 3:
            _len |= (uint16_t)b << 8;
            _sum += b;
            if (_len == 0 || _len > MAX_DATA) _syncCount = 0;  // trame invalide
            break;
        default: {
            uint16_t i = _pos - 4;
            if (i < _len) {
                _data[i] = b;
                _sum += b;
            } else {  // octet de checksum
                if (b == _sum) {
                    handleFrame(_type, _data, _len);
                } else {
                    _badChecksum++;
                }
                _syncCount = 0;
                return;
            }
        }
    }
    _pos++;
}

void LidarGS2::handleFrame(uint8_t type, const uint8_t* data, uint16_t len) {
    if (type == CMD_GET_PARAMETER) {
        handleParameters(data, len);
    } else if (type == CMD_SCAN && len == SCAN_DATA_LEN) {
        handleScan(data);
    }
}

void LidarGS2::handleParameters(const uint8_t* d, uint16_t len) {
    if (len < 9) return;
    auto u16 = [d](int i) { return (uint16_t)(d[i] | (d[i + 1] << 8)); };
    _cal.k0 = u16(0) / 10000.0f;
    _cal.b0 = u16(2) / 10000.0f;
    _cal.k1 = u16(4) / 10000.0f;
    _cal.b1 = u16(6) / 10000.0f;
    _cal.biasDeg = (int8_t)d[8] * 0.1f;
    _cal.valid = true;
}

// Conversion (distance brute, indice) -> (x, y), d'après le SDK officiel.
bool LidarGS2::toXY(uint16_t dist, int n, float& x, float& y) const {
    const float half = POINT_COUNT / 2;
    const float pitch = PITCH_DEG + _cal.biasDeg;
    float tx, ty;

    if (n < half) {
        float pixelU = half - n;
        float theta = (_cal.b0 > 1) ? _cal.k0 * pixelU - _cal.b0
                                    : atanf(_cal.k0 * pixelU - _cal.b0) * RAD_TO_DEG;
        float d = (dist - ANGLE_PX) / cosf((pitch - theta) * DEG_TO_RAD);
        float t = theta * DEG_TO_RAD, p = pitch * DEG_TO_RAD;
        tx = cosf(p) * d * cosf(t) + sinf(p) * d * sinf(t) + ANGLE_PX;
        ty = -sinf(p) * d * cosf(t) + cosf(p) * d * sinf(t) - ANGLE_PY;
        if (ty >= 0.0f) return false;  // filtre du SDK : caméra 0 = côté négatif
    } else {
        float pixelU = POINT_COUNT - n;
        float theta = (_cal.b1 > 1) ? _cal.k1 * pixelU - _cal.b1
                                    : atanf(_cal.k1 * pixelU - _cal.b1) * RAD_TO_DEG;
        float d = (dist - ANGLE_PX) / cosf((pitch + theta) * DEG_TO_RAD);
        float t = theta * DEG_TO_RAD, p = -pitch * DEG_TO_RAD;
        tx = cosf(p) * d * cosf(t) + sinf(p) * d * sinf(t) + ANGLE_PX;
        ty = -sinf(p) * d * cosf(t) + cosf(p) * d * sinf(t) + ANGLE_PY;
        if (ty < 0.0f) return false;  // caméra 1 = côté positif
    }

    if (tx <= 0.0f) return false;
    x = tx;
    y = _cfg.yLeftPositive ? ty : -ty;
    return true;
}

void LidarGS2::handleScan(const uint8_t* data) {
    const uint8_t* raw = data + 2;  // 2 octets d'environnement
    uint16_t dist[POINT_COUNT];
    LidarPoint pts[POINT_COUNT];

    for (uint16_t i = 0; i < POINT_COUNT; ++i) {
        dist[i] = (raw[2 * i] | (raw[2 * i + 1] << 8)) & 0x01FF;  // 9 bits = mm
        pts[i].valid = false;
        if (_cal.valid && validDist(dist[i])) {
            pts[i].valid = toXY(dist[i], i, pts[i].x, pts[i].y);
        }
    }

    // Un point n'est gardé que si un voisin de la même caméra est proche :
    // on élimine ainsi les points isolés (valeurs aberrantes du GS2).
    auto sameCamera = [](uint16_t a, uint16_t b) {
        return (a < POINT_COUNT / 2) == (b < POINT_COUNT / 2);
    };
    const float tol2 = (float)_cfg.spikeToleranceMm * _cfg.spikeToleranceMm;

    uint16_t frontMin = 0xFFFF;
    bool hasNearest = false;
    float nearestX = 0, nearestY = 0, nearestD2 = 1e12f;

    if (_cal.valid) {
        for (uint16_t i = 0; i < POINT_COUNT; ++i) {
            if (!pts[i].valid) continue;
            bool confirmed = false;
            for (int j : {(int)i - 1, (int)i + 1}) {
                if (j < 0 || j >= POINT_COUNT || !sameCamera(i, j) || !pts[j].valid) continue;
                float dx = pts[i].x - pts[j].x, dy = pts[i].y - pts[j].y;
                if (dx * dx + dy * dy <= tol2) confirmed = true;
            }
            if (!confirmed) continue;

            float d2 = pts[i].x * pts[i].x + pts[i].y * pts[i].y;
            if (d2 < nearestD2) {
                nearestD2 = d2;
                nearestX = pts[i].x;
                nearestY = pts[i].y;
                hasNearest = true;
            }
            if (fabsf(pts[i].y) <= _cfg.corridorHalfWidthMm && pts[i].x < frontMin) {
                frontMin = (uint16_t)pts[i].x;
            }
        }
    } else {
        // Sans calibration : indices "devant" fixes (code 2026)
        for (const Range& r : _cfg.fallbackFront) {
            for (uint16_t i = r.first; i <= r.last && i < POINT_COUNT; ++i) {
                if (!validDist(dist[i])) continue;
                bool confirmed =
                    (i > r.first && validDist(dist[i - 1]) &&
                     abs((int)dist[i] - (int)dist[i - 1]) <= _cfg.spikeToleranceMm) ||
                    (i < r.last && i + 1 < POINT_COUNT && validDist(dist[i + 1]) &&
                     abs((int)dist[i] - (int)dist[i + 1]) <= _cfg.spikeToleranceMm);
                if (confirmed && dist[i] < frontMin) frontMin = dist[i];
            }
        }
    }

    portENTER_CRITICAL(&_mux);
    memcpy(_dist, dist, sizeof(dist));
    memcpy(_points, pts, sizeof(pts));
    _hasNearest = hasNearest;
    _nearestX = nearestX;
    _nearestY = nearestY;
    portEXIT_CRITICAL(&_mux);

    _frontMin = frontMin;
    updateObstacle(frontMin);
    _lastFrameMs = millis();
    _frames++;
}

void LidarGS2::updateObstacle(uint16_t frontMin) {
    if (frontMin < _cfg.obstacleMm) {
        _clearCount = 0;
        if (_nearCount < 255) _nearCount++;
        if (_nearCount >= _cfg.onFrames) _obstacle = true;
    } else if (frontMin >= _cfg.obstacleMm + _cfg.hysteresisMm) {
        _nearCount = 0;
        if (_clearCount < 255) _clearCount++;
        if (_clearCount >= _cfg.offFrames) _obstacle = false;
    }
}

bool LidarGS2::healthy() const {
    return _frames > 0 && millis() - _lastFrameMs < 300;
}

bool LidarGS2::nearest(float& x, float& y) const {
    portENTER_CRITICAL(&_mux);
    bool ok = _hasNearest;
    x = _nearestX;
    y = _nearestY;
    portEXIT_CRITICAL(&_mux);
    return ok && healthy();
}

void LidarGS2::copyDistances(uint16_t* out) const {
    portENTER_CRITICAL(&_mux);
    memcpy(out, _dist, sizeof(_dist));
    portEXIT_CRITICAL(&_mux);
}

void LidarGS2::copyPoints(LidarPoint* out) const {
    portENTER_CRITICAL(&_mux);
    memcpy(out, _points, sizeof(_points));
    portEXIT_CRITICAL(&_mux);
}

LidarGS2::Stats LidarGS2::stats() const {
    return {_frames, _badChecksum,
            _frames ? (uint32_t)(millis() - _lastFrameMs) : 0xFFFFFFFF};
}
