// Monde simulé : table, obstacles fixes, nos PAMI (le vrai code), PAMI
// adverses scriptés, lidar simulé, détection des collisions, enregistrement.
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "app.h"

struct SimOptions {
    Team team = Team::Blue;
    bool enemies = true;
    float pamiLengthMm = 190.0f;  // dans le sens de marche
    float pamiWidthMm = 95.0f;
    uint32_t recordFromMs = 84000;
    uint32_t recordPeriodMs = 20;
};

// Rectangle orienté (empreinte d'un PAMI)
struct Box {
    double cx, cy, th, hl, hw;  // centre, cap, demi-longueur, demi-largeur
};

struct OurPami {
    App* app;
    int id;
    Pose target;  // douve ou zone d'attaque (pour l'affichage)
};

struct EnemyPami {
    std::string name;
    Pose pose;
    Pose target;
    Pose start;  // rempli par addEnemy
    uint32_t startMs;
    float speedMmS;
};

struct Frame {
    uint32_t t;
    // par corps : x, y, theta, drapeaux, action, servo
    std::vector<float> values;
};

class SimWorld {
   public:
    explicit SimWorld(const SimOptions& opt) : _opt(opt) {}

    void addOurPami(App* app, int id);
    void addEnemy(EnemyPami e) {
        e.start = e.pose;
        _enemies.push_back(e);
    }

    void step(uint32_t tMs);  // après le pas de temps des moteurs et des PAMI
    bool writeOutput(const char* path) const;
    void printSummary() const;

    std::vector<OurPami>& ours() { return _ours; }

    enum Flags : int {
        FLAG_OBSTACLE = 1,   // le lidar voit quelque chose dans le couloir
        FLAG_COLLISION = 2,  // touche un obstacle, un bord ou un autre PAMI
        FLAG_PAUSED = 4,     // évitement en cours
        FLAG_MOVING = 8,
        FLAG_ENDED = 16,  // match terminé pour ce PAMI
    };

   private:
    Box ourBox(const OurPami& p) const;
    Box enemyBox(const EnemyPami& e) const;
    void moveEnemies(uint32_t tMs);
    void scanLidar(OurPami& p, size_t index);
    double castRay(size_t selfIndex, double ox, double oy, double dx, double dy) const;
    void checkCollisions(uint32_t tMs);
    void checkStables(uint32_t tMs);
    void record(uint32_t tMs);
    int actionIndex(const char* name);

    SimOptions _opt;
    std::vector<OurPami> _ours;
    std::vector<EnemyPami> _enemies;
    std::vector<int> _collisionFlags;  // par corps (nos PAMI puis adversaires)
    std::set<std::string> _activeContacts;
    std::set<int> _leftStableEarly;
    std::vector<std::string> _actionNames;
    std::vector<Frame> _frames;
};
