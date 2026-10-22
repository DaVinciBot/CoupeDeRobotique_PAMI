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
    Boulet = 3,  // lancer le boulet dans la cour adverse (5 pts, +15 si 3)
};

struct PamiProfile {
    uint8_t id;
    Role role;
    uint8_t microstepping;  // réglage matériel des A4988 (MS1..MS3)
    uint8_t douveIndex;     // pour Role::Douve : index dans table::ENEMY_DOUVES
    Pose startBlue;         // pose de départ (repère table, équipe bleue)
    uint16_t startDelayMs;  // décalage après 85 s, pour ne pas se gêner à la sortie
};

// Écuries : 200 x 300 mm chacune, 2 par équipe (table::STABLES).
// /!\ Combien de PAMI tiennent dans une écurie ? Les départs des PAMI 3 à 6
// supposent 2 PAMI côte à côte par écurie (PAMI de moins de 150 mm de large).
constexpr PamiProfile PAMI_PROFILES[] = {
    // id  rôle        micropas douve  départ (x, y, theta)                 décalage
    {1, Role::Douve, 8, 0, {2900.0f, 150.0f, table::FACING_ENEMY}, 0},     // ex-"Normal"
    {2, Role::Douve, 4, 2, {2900.0f, 1850.0f, table::FACING_ENEMY}, 0},    // ex-"Ninja"
    {3, Role::Douve, 8, 1, {2900.0f, 225.0f, table::FACING_ENEMY}, 600},
    {4, Role::Attack, 8, 0, {2900.0f, 1775.0f, table::FACING_ENEMY}, 600},
    {5, Role::Boulet, 8, 0, {2900.0f, 75.0f, table::FACING_ENEMY}, 1200},
    {6, Role::Douve, 8, 1, {2900.0f, 1925.0f, table::FACING_ENEMY}, 1200},
};

// Attaque : le lidar arrête le PAMI à ~LIDAR_OBSTACLE_MM de la cible, trop
// loin pour la toucher. Il avance encore de cette distance avant de déployer.
constexpr float ATTACK_CREEP_MM = 80.0f;
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
        case Role::Boulet: return "boulet";
    }
    return "?";
}

// Angles de l'actionneur (servo 1) selon le rôle. À régler sur le robot.
constexpr uint8_t ATTACK_SERVO_REST_DEG = 0;
constexpr uint8_t ATTACK_SERVO_DEPLOY_DEG = 120;
constexpr uint8_t BOULET_SERVO_HOLD_DEG = 0;
constexpr uint8_t BOULET_SERVO_RELEASE_DEG = 90;
