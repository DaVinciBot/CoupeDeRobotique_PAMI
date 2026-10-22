#pragma once

#include <Arduino.h>

// Point du lidar dans son propre repère : x devant, y à gauche (mm).
struct LidarPoint {
    float x;
    float y;
    bool valid;
};

// Driver du lidar YDLIDAR GS2 : deux caméras, 160 points (0..79 et 80..159),
// environ 100° de champ, 25 à 300 mm.
//
// Au démarrage, le lidar donne ses paramètres de calibration (commande 0x61) :
// chaque point est alors converti en position (x, y), avec la même formule
// que le SDK officiel YDLIDAR (GSLidarDriver::angTransform). On peut ainsi
// surveiller une vraie zone devant le PAMI et repérer l'objet le plus proche
// dans tout le champ. Sans calibration, on revient à des indices fixes.
//
// La lecture se fait dans une tâche FreeRTOS dédiée : loop() ne fait que lire
// les résultats, sans jamais bloquer.
//
// Trame : A5 A5 A5 A5 | adresse | type | longueur (2, LE) | données | checksum
// checksum = somme (adresse, type, longueur, données) sur 8 bits.
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
        uint16_t minMm = 25;   // zone aveugle
        uint16_t maxMm = 300;  // portée max

        // Zone surveillée devant le PAMI (repère lidar) : un couloir de
        // `obstacleMm` de profondeur et 2 x `corridorHalfWidthMm` de large.
        uint16_t obstacleMm = 120;
        uint16_t corridorHalfWidthMm = 90;
        uint16_t hysteresisMm = 15;
        uint8_t onFrames = 2;   // trames consécutives pour déclarer un obstacle
        uint8_t offFrames = 3;  // trames consécutives pour le lever

        uint16_t spikeToleranceMm = 20;  // écart max avec un point voisin
        bool yLeftPositive = true;       // inverser si gauche et droite sont échangées

        // Utilisé seulement si le lidar ne donne pas sa calibration.
        Range fallbackFront[2] = {{0, 20}, {140, 159}};
    };

    struct Calibration {
        bool valid = false;
        float k0 = 0, b0 = 0, k1 = 0, b1 = 0;
        float biasDeg = 0;
    };

    struct Stats {
        uint32_t frames;
        uint32_t badChecksum;
        uint32_t lastFrameAgeMs;
    };

    // Lit la calibration (bloque jusqu'à ~1,5 s au démarrage), puis lance le scan.
    void begin(HardwareSerial& serial, const Config& cfg);
    void startTask(uint8_t core = 0);

    bool healthy() const;
    bool obstacle() const { return _obstacle; }       // quelque chose dans le couloir
    uint16_t frontMinMm() const { return _frontMin; }  // 0xFFFF = couloir libre

    // Point le plus proche dans tout le champ (repère lidar). false si rien.
    bool nearest(float& x, float& y) const;

    void setObstacleThreshold(uint16_t mm) { _cfg.obstacleMm = mm; }
    uint16_t obstacleThreshold() const { return _cfg.obstacleMm; }
    void setCorridorHalfWidth(uint16_t mm) { _cfg.corridorHalfWidthMm = mm; }
    uint16_t corridorHalfWidth() const { return _cfg.corridorHalfWidthMm; }

    void copyDistances(uint16_t* out) const;  // distances brutes, POINT_COUNT valeurs
    void copyPoints(LidarPoint* out) const;   // points (x, y), POINT_COUNT valeurs
    Calibration calibration() const { return _cal; }
    Stats stats() const;

   private:
    static constexpr uint16_t MAX_DATA = 400;
    static constexpr uint8_t CMD_GET_PARAMETER = 0x61;
    static constexpr uint8_t CMD_SCAN = 0x63;
    static constexpr uint8_t CMD_STOP = 0x64;
    static constexpr uint16_t SCAN_DATA_LEN = 2 + POINT_COUNT * 2;  // env + points

    static void taskEntry(void* self);
    void taskLoop();
    void sendCommand(uint8_t cmd);
    void feed(uint8_t b);
    void handleFrame(uint8_t type, const uint8_t* data, uint16_t len);
    void handleParameters(const uint8_t* data, uint16_t len);
    void handleScan(const uint8_t* data);
    bool toXY(uint16_t dist, int index, float& x, float& y) const;
    void updateObstacle(uint16_t frontMin);
    bool validDist(uint16_t d) const { return d > _cfg.minMm && d < _cfg.maxMm; }

    HardwareSerial* _serial = nullptr;
    Config _cfg;
    Calibration _cal;

    // Analyse des trames
    uint8_t _syncCount = 0;
    uint16_t _pos = 0;  // octets reçus après les 4 x A5
    uint8_t _addr = 0, _type = 0;
    uint16_t _len = 0;
    uint8_t _sum = 0;
    uint8_t _data[MAX_DATA];

    // Résultats (écrits par la tâche lidar, lus par loop())
    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    uint16_t _dist[POINT_COUNT] = {};
    LidarPoint _points[POINT_COUNT] = {};
    bool _hasNearest = false;
    float _nearestX = 0, _nearestY = 0;

    volatile bool _obstacle = false;
    volatile uint16_t _frontMin = 0xFFFF;
    volatile uint32_t _lastFrameMs = 0;
    volatile uint32_t _frames = 0;
    volatile uint32_t _badChecksum = 0;
    uint8_t _nearCount = 0;
    uint8_t _clearCount = 0;
    uint32_t _lastCommandMs = 0;
};
