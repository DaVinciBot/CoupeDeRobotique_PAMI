// Partie portable de TofArray : géométrie et détection (aussi utilisée par
// le simulateur).
#include "TofArray.h"

void TofArray::configure(const Config& cfg) {
    _cfg = cfg;
    if (_cfg.count > MAX_SENSORS) _cfg.count = MAX_SENSORS;
    setDetection(_cfg.detection);
}

void TofArray::setReading(uint8_t sensor, uint16_t mm, bool ok) {
    if (sensor >= _cfg.count) return;
    _mm[sensor] = mm;
    _ok[sensor] = ok && mm >= _cfg.minMm && mm <= _cfg.maxMm;
    _readingMs[sensor] = millis();
    _alive[sensor] = true;
    _fresh = true;
}

void TofArray::process() {
    if (!_fresh) return;  // pas de nouvelle mesure depuis la dernière trame
    _fresh = false;
    _lastProcessMs = millis();

    ObstaclePoint pts[MAX_SENSORS];
    for (uint8_t i = 0; i < _cfg.count; ++i) {
        const Mount& m = _cfg.sensors[i];
        bool recent = _lastProcessMs - _readingMs[i] <= _cfg.maxReadingAgeMs;
        float a = m.angleDeg * DEG_TO_RAD;
        float d = _mm[i];
        pts[i].x = m.x + d * cosf(a);
        pts[i].y = m.y + d * sinf(a);
        pts[i].valid = recent && _ok[i];
        // Une mesure ToF est filtrée par le capteur lui-même (statut de
        // mesure) : pas de point isolé à éliminer comme pour le lidar.
        pts[i].confirmed = pts[i].valid;
    }
    publish(pts, _cfg.count);
}
