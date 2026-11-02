// Capteur d'obstacles du PAMI : ce que voit le PAMI devant lui, quel que soit
// le capteur (aujourd'hui des ToF, voir TofArray).
//
// Les points sont dans le repère « capteurs » : origine au point de référence
// des capteurs (SENSOR_OFFSET_X_MM devant l'axe des roues), x vers l'avant,
// y vers la gauche, en mm.
//
// La détection (couloir surveillé devant le PAMI, hystérésis, point le plus
// proche) est commune à tous les capteurs : publish() la fait à chaque
// nouvelle trame de points.
#pragma once

#include <Arduino.h>

struct ObstaclePoint {
    float x;
    float y;
    bool valid;      // une mesure est là
    bool confirmed;  // mesure fiable (pas un point isolé / douteux)
};

class ObstacleSensor {
   public:
    static constexpr uint8_t MAX_POINTS = 16;

    struct Detection {
        // Couloir surveillé : `obstacleMm` de profondeur, 2 x
        // `corridorHalfWidthMm` de large, devant le point de référence.
        uint16_t obstacleMm = 220;
        uint16_t corridorHalfWidthMm = 50;
        uint16_t hysteresisMm = 15;
        uint8_t onFrames = 2;            // trames de suite pour déclarer un obstacle
        uint8_t offFrames = 3;           // trames de suite pour le lever
        uint16_t fastApproachMm = 15;    // rapproché de plus en une trame : 1 seule suffit
        uint32_t staleMs = 300;          // sans trame depuis plus longtemps : capteur muet
    };

    virtual ~ObstacleSensor() = default;

    bool healthy() const;
    bool obstacle() const { return _obstacle; }        // quelque chose dans le couloir
    uint16_t frontMinMm() const { return _frontMin; }  // 0xFFFF = couloir libre
    bool nearest(float& x, float& y) const;            // point le plus proche (tout le champ)
    uint32_t frames() const { return _frames; }

    // Points de la dernière trame ; renvoie leur nombre (<= MAX_POINTS)
    uint8_t copyPoints(ObstaclePoint* out) const;

    void setObstacleThreshold(uint16_t mm) { _det.obstacleMm = mm; }
    uint16_t obstacleThreshold() const { return _det.obstacleMm; }
    void setCorridorHalfWidth(uint16_t mm) { _det.corridorHalfWidthMm = mm; }
    uint16_t corridorHalfWidth() const { return _det.corridorHalfWidthMm; }

   protected:
    void setDetection(const Detection& d) { _det = d; }
    // Nouvelle trame : met à jour points, couloir, obstacle, point le plus proche
    void publish(const ObstaclePoint* points, uint8_t count);

   private:
    void updateObstacle(uint16_t frontMin);

    Detection _det;
    ObstaclePoint _points[MAX_POINTS] = {};
    uint8_t _count = 0;
    bool _hasNearest = false;
    float _nearestX = 0, _nearestY = 0;
    bool _obstacle = false;
    uint16_t _frontMin = 0xFFFF;
    uint16_t _prevFrontMin = 0xFFFF;
    uint8_t _nearCount = 0, _clearCount = 0;
    uint32_t _lastFrameMs = 0;
    uint32_t _frames = 0;
};
