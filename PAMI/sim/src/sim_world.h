// Monde simulé : table, carrières, murailles, PAMI des deux équipes (tous
// avec le vrai code), lidar simulé, détection des collisions, enregistrement.
#pragma once

#include <set>
#include <string>
#include <vector>

#include "app.h"

struct SimOptions {
    Team team = Team::Blue;  // "notre" équipe (affichée en premier)
    bool enemies = true;     // les 6 PAMI de l'autre équipe
    float pamiLengthMm = PAMI_LENGTH_MM;  // gabarit de config.h
    float pamiWidthMm = PAMI_WIDTH_MM;
    float speedMmS = 0.0f;  // 0 = vitesse de config.h
    uint32_t recordFromMs = 84000;
    uint32_t recordPeriodMs = 20;
};

// Rectangle orienté (empreinte d'un PAMI, muraille, carrière)
struct Box {
    double cx, cy, th, hl, hw;  // centre, cap, demi-longueur, demi-largeur
};

struct SimPami {
    App* app;
    int logId;   // identifiant du journal : n° du PAMI, + 100 pour l'adversaire
    bool ours;   // notre équipe ou l'adversaire
    Team team;
    Pose target;  // douve ou zone d'attaque (pour l'affichage)

    // Vraie position sur la table (tenue par le simulateur). Le PAMI, lui,
    // ne connaît que son odométrie (app->drive.pose()), calculée à partir
    // des pas des moteurs : bloqué contre un autre PAMI, ses roues patinent,
    // il croit avancer et son odométrie dérive.
    Pose truePose;
    Pose lastOdom;  // odométrie au pas précédent
    bool started = false;
    bool blocked = false;     // mouvement refusé à ce pas (contact)
    float maxDriftMm = 0.0f;  // plus grand écart odométrie / vraie position
};

struct Frame {
    uint32_t t;
    // par PAMI : x, y, theta, drapeaux, action, servo
    std::vector<float> values;
};

class SimWorld {
   public:
    explicit SimWorld(const SimOptions& opt) : _opt(opt) {}

    void addPami(App* app, int logId, bool ours, Team team);

    void step(uint32_t tMs);  // après le pas de temps des moteurs et des PAMI
    // Écrit le déroulé dans window.SIM_RUNS[name] (lu par viewer.html)
    bool writeOutput(const char* path, const char* name) const;
    void printSummary() const;

    std::vector<SimPami>& pamis() { return _pamis; }

    enum Flags : int {
        FLAG_OBSTACLE = 1,   // le lidar voit quelque chose dans le couloir
        FLAG_COLLISION = 2,  // touche un obstacle, un bord ou un autre PAMI
        FLAG_PAUSED = 4,     // évitement en cours
        FLAG_MOVING = 8,
        FLAG_ENDED = 16,  // match terminé pour ce PAMI
    };

   private:
    Box pamiBox(const SimPami& p) const;
    Box boxAt(const Pose& axle) const;
    void moveBodies();
    void scanLidar(SimPami& p, size_t index);
    double castRay(size_t selfIndex, double ox, double oy, double dx, double dy) const;
    void checkCollisions(uint32_t tMs);
    void checkStables(uint32_t tMs);
    void record(uint32_t tMs);
    int actionIndex(const char* name);

    SimOptions _opt;
    std::vector<SimPami> _pamis;
    std::vector<int> _collisionFlags;  // par PAMI
    std::set<std::string> _activeContacts;
    std::set<int> _leftStableEarly;
    std::vector<std::string> _actionNames;
    std::vector<Frame> _frames;
};
