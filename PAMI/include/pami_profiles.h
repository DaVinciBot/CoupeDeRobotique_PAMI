// Ce qui différencie les PAMI : rôle, micropas, départ, trajet.
// Le numéro du PAMI est choisi à la compilation (pio run -e pamiN).
#pragma once

#include <Arduino.h>

#include "Pose.h"
#include "config.h"
#include "table_2027.h"

enum class Role : uint8_t {
    Test = 0,    // carré de test, pour calibrer
    Douve = 1,   // occuper une douve adverse (5 pts)
    Attack = 2,  // toucher un PAMI adverse avec l'actionneur (5 pts)
};

// Point de passage (repère bleu) et comportement en y allant :
//   HOME    : notre couloir de sortie. On y fait la queue derrière les
//             coéquipiers : ni contournement, ni décalage.
//   PASSAGE : passage étroit où l'on peut croiser l'adversaire : on contourne
//             si l'on reste bloqué (en reculant d'abord pour sortir du passage).
//   OPEN    : terrain ouvert : on contourne et on tient sa droite.
enum class Zone : uint8_t { HOME, PASSAGE, OPEN };
struct Waypoint {
    float x, y;
    Zone zone;
};

struct PamiProfile {
    uint8_t id;
    Role role;
    uint8_t microstepping;  // réglage matériel des A4988 (MS1..MS3)
    uint8_t targetIndex;    // Douve : index dans table::ENEMY_DOUVES
                            // Attack : index dans table::ATTACK_ZONES
    Pose startBlue;         // pose de départ de l'axe des roues (repère bleu)
    uint16_t startDelayMs;  // décalage après 85 s (0 : tous en même temps)
    float exitMm;           // sortie d'écurie : tout droit, au plus cette distance
    uint16_t exitMs;        // ... et pendant ce temps (arrêt si obstacle devant)
    // Points de passage (repère bleu) entre la sortie et la cible
    const Waypoint* route;
    uint8_t routeLength;
};

template <size_t N>
constexpr uint8_t countOf(const Waypoint (&)[N]) {
    return (uint8_t)N;
}

// ------------------------------------------------------------------
// Écuries : 200 x 300 mm chacune, 2 par équipe (table::STABLES), 3 PAMI par
// écurie côte à côte, tournés vers l'adversaire, centres espacés de 100 mm.
//
// Sortie : les 3 PAMI d'une écurie partent en même temps, tout droit. Chacun
// roule un peu plus longtemps que son voisin côté intérieur (2,5 s, 3 s,
// 3,5 s) : quand il tourne, son voisin a déjà tourné et s'éloigne, et son
// avant ne balaie pas la place d'un PAMI qu'il ne voit pas (hors du champ
// du lidar). Ensuite, chacun va directement à sa cible : sans les carrières,
// rien ne gêne entre l'écurie et le château adverse.
// ------------------------------------------------------------------
constexpr float EXIT_MAX_MM = 1200.0f;
constexpr uint16_t INNER_SLOT_EXIT_MS = 2500;   // y 250 / 1750
constexpr uint16_t MIDDLE_SLOT_EXIT_MS = 3000;  // y 150 / 1850
constexpr uint16_t EDGE_SLOT_EXIT_MS = 3500;    // y  50 / 1950

constexpr float STABLE_SLOT_X = 2900.0f - PAMI_AXLE_FROM_CENTER_MM;  // axe des roues

// Trajets directs : aucun point de passage entre la sortie et la cible.
// (Des points de passage peuvent être ajoutés ici : Waypoint{x, y, zone}.)
// Rangement dans l'écurie : le PAMI dont la cible est la plus « en biais »
// est côté intérieur (il tourne le plus, et le premier) ; celui qui va le
// plus droit est contre le bord. Les trajets s'écartent en éventail sans se
// croiser.
constexpr PamiProfile PAMI_PROFILES[] = {
    // id  rôle       micropas cible  départ (x, y, theta)          décalage sortie   durée
    // écurie 0 : intérieur (y 250), milieu (150), bord (50)
    {5, Role::Attack, 8, 0, {STABLE_SLOT_X, 250.0f, table::FACING_ENEMY}, 0, EXIT_MAX_MM,
     INNER_SLOT_EXIT_MS, nullptr, 0},  // -> zone 0 (la plus en biais)
    {3, Role::Douve, 8, 1, {STABLE_SLOT_X, 150.0f, table::FACING_ENEMY}, 0, EXIT_MAX_MM,
     MIDDLE_SLOT_EXIT_MS, nullptr, 0},  // -> pont
    {1, Role::Douve, 8, 0, {STABLE_SLOT_X, 50.0f, table::FACING_ENEMY}, 0, EXIT_MAX_MM,
     EDGE_SLOT_EXIT_MS, nullptr, 0},  // -> douve 0 (presque tout droit), ex-"Normal"
    // écurie 1 : intérieur (y 1750), milieu (1850), bord (1950)
    {6, Role::Attack, 8, 1, {STABLE_SLOT_X, 1750.0f, table::FACING_ENEMY}, 0, EXIT_MAX_MM,
     INNER_SLOT_EXIT_MS, nullptr, 0},  // -> zone 1 (la plus en biais)
    {4, Role::Attack, 8, 2, {STABLE_SLOT_X, 1850.0f, table::FACING_ENEMY}, 0, EXIT_MAX_MM,
     MIDDLE_SLOT_EXIT_MS, nullptr, 0},  // -> zone 2
    {2, Role::Douve, 4, 2, {STABLE_SLOT_X, 1950.0f, table::FACING_ENEMY}, 0, EXIT_MAX_MM,
     EDGE_SLOT_EXIT_MS, nullptr, 0},  // -> douve 2 (presque tout droit), ex-"Ninja"
};

// Attaque : le PAMI s'arrête quand la cible est à cette distance de l'axe
// des roues (lidar devant + couloir surveillé), puis avance encore de
// ATTACK_CREEP_MM pour la toucher avant de déployer.
constexpr float ATTACK_STOP_WITHIN_MM = LIDAR_OFFSET_X_MM + LIDAR_OBSTACLE_MM;
constexpr float ATTACK_CREEP_MM = 80.0f;
// Les PAMI d'attaque ne chassent qu'à partir de cet instant du match : nos
// PAMI douve sont passés, ce que voit le lidar est alors plutôt adverse.
// (La vraie solution : la Jetson dit qui est qui, en Wi-Fi.)
constexpr uint32_t ATTACK_HUNT_FROM_MS = 94000;
// Marge autour des obstacles fixes : un point vu à moins de cette distance
// d'une carrière ou d'un rempart n'est pas pris pour un PAMI adverse
// (positions relevées à +/- 20 mm, erreur du lidar et de l'odométrie).
constexpr float ATTACK_FIXED_OBSTACLE_MARGIN_MM = 40.0f;
// Bloqué par un obstacle immobile depuis ce temps : contourner (reculer, se
// décaler à droite de préférence, repartir). Court, pour ne pas perdre de
// temps ; un obstacle qui VIENT VERS NOUS est esquivé sans s'arrêter.
constexpr uint32_t DETOUR_AFTER_MS = 150;  // s'il ne s'éloigne pas : contourner vite
constexpr uint32_t PASSAGE_DETOUR_AFTER_MS = 1500;  // passage étroit
// Points de passage décalés de cette distance vers la droite du trajet.
constexpr float KEEP_RIGHT_MM = 60.0f;
// Un point de passage est « atteint » à cette distance : le PAMI enchaîne
// alors vers le suivant en courbe, sans freiner.
constexpr float PASS_THROUGH_MM = 100.0f;
// Douve : à l'approche, on ignore le lidar (rempart, pierres autour des douves).
constexpr float DOUVE_IGNORE_OBSTACLE_WITHIN_MM = 150.0f;

inline const PamiProfile& pamiProfile(uint8_t id) {
    for (const PamiProfile& p : PAMI_PROFILES) {
        if (p.id == id) return p;
    }
    return PAMI_PROFILES[0];
}

inline const char* roleName(Role r) {
    switch (r) {
        case Role::Test: return "test";
        case Role::Douve: return "douve";
        case Role::Attack: return "attaque";
    }
    return "?";
}

// Angles de l'actionneur d'attaque (servo 1). À régler sur le robot.
constexpr uint8_t ATTACK_SERVO_REST_DEG = 0;
constexpr uint8_t ATTACK_SERVO_DEPLOY_DEG = 120;
