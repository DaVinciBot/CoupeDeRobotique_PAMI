// Rangée de capteurs ToF (VL53L1X) à l'avant du PAMI.
//
// Chaque capteur donne UNE distance (l'objet le plus proche dans son cône
// d'environ 27°). Il devient un point, placé sur l'axe du capteur, dans le
// repère commun des capteurs (voir ObstacleSensor). Monté en éventail
// (ex. -30°, 0°, +30°), l'ensemble couvre l'avant et les côtés avant.
//
// Câblage : tous les capteurs sur le même bus I2C ; chacun a une broche
// XSHUT pour l'allumer seul au démarrage et lui donner sa propre adresse.
//
// Partie portable (TofArray.cpp, utilisée aussi par le simulateur) :
// géométrie + détection. Partie matérielle (TofArrayHw.cpp) : I2C.
#pragma once

#include "ObstacleSensor.h"

class TofArray : public ObstacleSensor {
   public:
    static constexpr uint8_t MAX_SENSORS = MAX_POINTS;

    struct Mount {
        int8_t xshutPin;  // -1 : pas de broche XSHUT (un seul capteur)
        float x, y;       // position dans le repère des capteurs (mm)
        float angleDeg;   // orientation (+ = vers la gauche)
    };

    struct Config {
        const Mount* sensors = nullptr;
        uint8_t count = 0;
        int8_t sdaPin = -1, sclPin = -1;
        uint32_t i2cHz = 400000;
        uint8_t firstAddress = 0x30;     // adresses données : 0x30, 0x31, ...
        uint16_t minMm = 40;             // en dessous : mesure peu fiable
        uint16_t maxMm = 600;            // au-delà : ignoré (rien devant)
        uint32_t timingBudgetMs = 20;    // 50 mesures/s par capteur
        // Zone de mesure du capteur (SPAD, 4..16) : 16 -> cône de ~27°,
        // 4 -> ~15° (ST UM2555). Plus étroit : le PAMI peut longer un bord
        // ou un voisin sans le prendre pour un obstacle.
        uint8_t roiSpads = 16;
        uint32_t maxReadingAgeMs = 100;  // mesure plus vieille : ignorée
        Detection detection;
    };

    // Robot : démarre l'I2C, adresse et lance chaque capteur (TofArrayHw.cpp).
    // Renvoie le nombre de capteurs qui répondent.
    uint8_t begin(const Config& cfg);
    // Robot : lit les capteurs qui ont une mesure, puis process().
    void update();

    // Portable : nouvelle mesure d'un capteur (ok = false : rien / erreur)
    void setReading(uint8_t sensor, uint16_t mm, bool ok);
    // Portable : construit la trame de points avec les dernières mesures
    void process();

    const Config& config() const { return _cfg; }
    uint16_t distanceMm(uint8_t sensor) const { return _mm[sensor]; }
    bool readingOk(uint8_t sensor) const { return _ok[sensor]; }
    bool sensorAlive(uint8_t sensor) const { return _alive[sensor]; }

   protected:
    void configure(const Config& cfg);

    Config _cfg;
    uint16_t _mm[MAX_SENSORS] = {};
    bool _ok[MAX_SENSORS] = {};
    bool _alive[MAX_SENSORS] = {};
    uint32_t _readingMs[MAX_SENSORS] = {};
    bool _fresh = false;
    uint32_t _lastProcessMs = 0;
};
