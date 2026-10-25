// Simulation : remplace le driver du lidar YDLIDAR GS2 (lib/lidar).
// Même interface publique que la vraie classe (ce que le code PAMI utilise),
// mais les points viennent du monde simulé (SimWorld) au lieu de l'UART.
// La détection (couloir, hystérésis, point le plus proche) reprend la logique
// de lib/lidar/LidarGS2.cpp : à garder alignée si elle change.
#pragma once

#include <Arduino.h>

struct LidarPoint {
    float x;
    float y;
    bool valid;
    bool confirmed;
};

class LidarGS2 {
   public:
    static constexpr uint16_t POINT_COUNT = 160;

    struct Range {
        uint8_t first;
        uint8_t last;
    };

    struct Config {
        int8_t rxPin = -1;
        int8_t txPin = -1;
        uint32_t baud = 921600;
        uint16_t minMm = 25;
        uint16_t maxMm = 300;
        uint16_t obstacleMm = 120;
        uint16_t corridorHalfWidthMm = 90;
        uint16_t hysteresisMm = 15;
        uint8_t onFrames = 2;
        uint8_t offFrames = 3;
        uint16_t spikeToleranceMm = 20;
        bool yLeftPositive = true;
        Range fallbackFront[2] = {{0, 20}, {140, 159}};
    };

    struct Calibration {
        bool valid = true;
        float k0 = 0, b0 = 0, k1 = 0, b1 = 0;
        float biasDeg = 0;
    };

    struct Stats {
        uint32_t frames;
        uint32_t badChecksum;
        uint32_t lastFrameAgeMs;
    };

    void begin(HardwareSerial&, const Config& cfg) { _cfg = cfg; }
    void startTask(uint8_t = 0) {}

    bool healthy() const;
    bool obstacle() const { return _obstacle; }
    uint16_t frontMinMm() const { return _frontMin; }
    bool nearest(float& x, float& y) const;

    void setObstacleThreshold(uint16_t mm) { _cfg.obstacleMm = mm; }
    uint16_t obstacleThreshold() const { return _cfg.obstacleMm; }
    void setCorridorHalfWidth(uint16_t mm) { _cfg.corridorHalfWidthMm = mm; }
    uint16_t corridorHalfWidth() const { return _cfg.corridorHalfWidthMm; }

    void copyDistances(uint16_t* out) const;
    void copyPoints(LidarPoint* out) const;
    Calibration calibration() const { return Calibration(); }
    Stats stats() const;

    // --- simulation ---
    const Config& simConfig() const { return _cfg; }
    // Nouvelle trame : distances (0 = rien) et points dans le repère lidar.
    void simSetScan(const uint16_t* dist, const LidarPoint* pts);

   private:
    void updateObstacle(uint16_t frontMin);

    Config _cfg;
    uint16_t _dist[POINT_COUNT] = {};
    LidarPoint _points[POINT_COUNT] = {};
    bool _hasNearest = false;
    float _nearestX = 0, _nearestY = 0;
    bool _obstacle = false;
    uint16_t _frontMin = 0xFFFF;
    uint32_t _lastFrameMs = 0;
    uint32_t _frames = 0;
    uint8_t _nearCount = 0;
    uint8_t _clearCount = 0;
};
