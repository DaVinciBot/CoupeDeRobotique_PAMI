// Ce qui différencie les PAMI : rôle, micropas, position de départ.
// Le numéro du PAMI est choisi à la compilation (pio run -e pamiN).
#pragma once

#include <Arduino.h>

#include "Pose.h"
#include "table_2027.h"

enum class Role : uint8_t {
    Test = 0,    // carré de test, pour calibrer
    Douve = 1,   // occuper une douve adverse (5 pts)
    Attack = 2,  // toucher un PAMI adverse avec l'actionneur (5 pts)
};

struct PamiProfile {
    uint8_t id;
    Role role;
    uint8_t microstepping;  // réglage matériel des A4988 (MS1..MS3)
    uint8_t targetIndex;    // Douve : index dans table::ENEMY_DOUVES
                            // Attack : index dans table::ATTACK_ZONES
    Pose startBlue;         // pose de départ (repère table, équipe bleue)
    uint16_t startDelayMs;  // décalage après 85 s, pour ne pas se gêner à la sortie
};

// Écuries : 200 x 300 mm chacune, 2 par équipe (table::STABLES).
// 6 PAMI = 3 par écurie, côte à côte, tournés vers l'adversaire : centres
// espacés de 100 mm. /!\ Il faut donc des PAMI de moins de ~95 mm de large
// (dimension perpendiculaire au sens de marche) et 200 mm de long.
// Dans chaque écurie, le PAMI douve qui sort en premier est au milieu.
constexpr PamiProfile PAMI_PROFILES[] = {
    // id  rôle        micropas cible  départ (x, y, theta)                 décalage
    // écurie 0 (y 0..300)
    {1, Role::Douve, 8, 0, {2900.0f, 150.0f, table::FACING_ENEMY}, 0},     // ex-"Normal"
    {3, Role::Douve, 8, 1, {2900.0f, 250.0f, table::FACING_ENEMY}, 600},
    {5, Role::Attack, 8, 0, {2900.0f, 50.0f, table::FACING_ENEMY}, 1200},
    // écurie 1 (y 1700..2000)
    {2, Role::Douve, 4, 2, {2900.0f, 1850.0f, table::FACING_ENEMY}, 0},    // ex-"Ninja"
    {4, Role::Attack, 8, 2, {2900.0f, 1750.0f, table::FACING_ENEMY}, 600},
    {6, Role::Attack, 8, 1, {2900.0f, 1950.0f, table::FACING_ENEMY}, 1200},
};

// Attaque : le PAMI s'arrête quand la cible est à cette distance du centre
// des roues (lidar devant + couloir surveillé), puis avance encore de
// ATTACK_CREEP_MM pour la toucher avant de déployer.
constexpr float ATTACK_STOP_WITHIN_MM = 180.0f;
constexpr float ATTACK_CREEP_MM = 80.0f;
// Marge autour des obstacles fixes : un point vu à moins de cette distance
// d'une tour ou d'une carrière n'est pas pris pour un PAMI adverse
// (positions relevées à +/- 20 mm, erreur du lidar et de l'odométrie).
constexpr float ATTACK_FIXED_OBSTACLE_MARGIN_MM = 40.0f;
// Douve : à l'approche, on ignore le lidar (murs, pierres autour des douves).
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
