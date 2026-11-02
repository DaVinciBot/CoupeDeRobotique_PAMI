#include "ObstacleSensor.h"

void ObstacleSensor::publish(const ObstaclePoint* points, uint8_t count) {
    if (count > MAX_POINTS) count = MAX_POINTS;
    uint16_t frontMin = 0xFFFF;
    bool hasNearest = false;
    float nearestX = 0, nearestY = 0, nearestD2 = 1e12f;

    for (uint8_t i = 0; i < count; ++i) {
        _points[i] = points[i];
        const ObstaclePoint& p = points[i];
        if (!p.valid || !p.confirmed || p.x <= 0.0f) continue;
        float d2 = p.x * p.x + p.y * p.y;
        if (d2 < nearestD2) {
            nearestD2 = d2;
            nearestX = p.x;
            nearestY = p.y;
            hasNearest = true;
        }
        if (fabsf(p.y) <= _det.corridorHalfWidthMm && p.x < frontMin) {
            frontMin = (uint16_t)p.x;
        }
    }
    _count = count;
    _hasNearest = hasNearest;
    _nearestX = nearestX;
    _nearestY = nearestY;
    _frontMin = frontMin;
    updateObstacle(frontMin);
    _lastFrameMs = millis();
    if (_lastFrameMs == 0) _lastFrameMs = 1;
    _frames++;
}

void ObstacleSensor::updateObstacle(uint16_t frontMin) {
    // Objet qui se rapproche vite (face à face) : une seule trame suffit
    bool fastApproach = _prevFrontMin != 0xFFFF && frontMin != 0xFFFF &&
                        frontMin + _det.fastApproachMm < _prevFrontMin;
    _prevFrontMin = frontMin;
    if (frontMin < _det.obstacleMm) {
        _clearCount = 0;
        if (_nearCount < 255) _nearCount++;
        if (_nearCount >= _det.onFrames || fastApproach) _obstacle = true;
    } else if (frontMin >= _det.obstacleMm + _det.hysteresisMm) {
        _nearCount = 0;
        if (_clearCount < 255) _clearCount++;
        if (_clearCount >= _det.offFrames) _obstacle = false;
    }
}

bool ObstacleSensor::healthy() const {
    return _frames > 0 && millis() - _lastFrameMs < _det.staleMs;
}

bool ObstacleSensor::nearest(float& x, float& y) const {
    x = _nearestX;
    y = _nearestY;
    return _hasNearest && healthy();
}

uint8_t ObstacleSensor::copyPoints(ObstaclePoint* out) const {
    memcpy(out, _points, sizeof(ObstaclePoint) * _count);
    return _count;
}
