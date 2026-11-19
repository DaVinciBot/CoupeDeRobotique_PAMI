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
// roule 150 mm de plus que son voisin côté intérieur (750, 900, 1050 mm) :
// quand il tourne, son voisin a déjà tourné et s'éloigne, et son avant ne
// balaie pas la place d'un PAMI qu'il ne voit pas (hors du champ des
// capteurs). Ensuite, chacun va directement à sa cible : sans les carrières,
// rien ne gêne entre l'écurie et le château adverse.
// La sortie se règle en distance, pas en temps : avec une durée fixe, à
// 200 mm/s les PAMI tournaient trop tôt (tour de notre château renversée,
// PAMI d'attaque dans le PAMI pont, vu en simulation). La durée n'est plus
// qu'une sécurité (obstacle devant : on finit par passer à la suite).
// ------------------------------------------------------------------
constexpr float INNER_SLOT_EXIT_MM = 750.0f;   // y 250 / 1750
constexpr float MIDDLE_SLOT_EXIT_MM = 900.0f;  // y 150 / 1850
constexpr float EDGE_SLOT_EXIT_MM = 1050.0f;   // y  50 / 1950
constexpr uint16_t EXIT_MAX_MS = 7000;

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
    {5, Role::Attack, 8, 0, {STABLE_SLOT_X, 250.0f, table::FACING_ENEMY}, 0, INNER_SLOT_EXIT_MM,
     EXIT_MAX_MS, nullptr, 0},  // -> zone 0 (la plus en biais)
    {3, Role::Douve, 8, 1, {STABLE_SLOT_X, 150.0f, table::FACING_ENEMY}, 0, MIDDLE_SLOT_EXIT_MM,
     EXIT_MAX_MS, nullptr, 0},  // -> pont
    {1, Role::Douve, 8, 0, {STABLE_SLOT_X, 50.0f, table::FACING_ENEMY}, 0, EDGE_SLOT_EXIT_MM,
     EXIT_MAX_MS, nullptr, 0},  // -> douve 0 (presque tout droit), ex-"Normal"
    // écurie 1 : intérieur (y 1750), milieu (1850), bord (1950)
    {6, Role::Attack, 8, 1, {STABLE_SLOT_X, 1750.0f, table::FACING_ENEMY}, 0, INNER_SLOT_EXIT_MM,
     EXIT_MAX_MS, nullptr, 0},  // -> zone 1 (la plus en biais)
    {4, Role::Attack, 8, 2, {STABLE_SLOT_X, 1850.0f, table::FACING_ENEMY}, 0, MIDDLE_SLOT_EXIT_MM,
     EXIT_MAX_MS, nullptr, 0},  // -> zone 2
    {2, Role::Douve, 4, 2, {STABLE_SLOT_X, 1950.0f, table::FACING_ENEMY}, 0, EDGE_SLOT_EXIT_MM,
     EXIT_MAX_MS, nullptr, 0},  // -> douve 2 (presque tout droit), ex-"Ninja"
};

// Attaque. Règlement 2027 (E.4.b) : l'attaque compte si l'ACTIONNEUR touche
// un PAMI adverse en fin de match, et seul l'actionneur a le droit de le
// toucher (pas la carrosserie). Le PAMI s'approche donc à
// ATTACK_APPROACH_GAP_MM, déploie l'actionneur, puis se tient à
// ATTACK_HOLD_GAP_MM de la face adverse (AttackHold) jusqu'à la fin du
// match, en suivant sa cible si elle bouge.
constexpr float FRONT_AHEAD_OF_AXLE_MM = PAMI_LENGTH_MM / 2 - PAMI_AXLE_FROM_CENTER_MM;
// /!\ À MESURER sur le PAMI : de combien l'actionneur déployé dépasse de
// l'avant du PAMI, et sa largeur. Valeurs provisoires.
constexpr float ATTACK_ACTUATOR_REACH_MM = 60.0f;
constexpr float ATTACK_ACTUATOR_WIDTH_MM = 30.0f;  // (simulation)
// Écart avant du PAMI <-> face adverse tenu pendant l'attaque : l'actionneur
// (REACH) dépasse de 20 mm sur l'adversaire, la carrosserie reste à distance.
constexpr float ATTACK_HOLD_GAP_MM = ATTACK_ACTUATOR_REACH_MM - 20.0f;
constexpr float ATTACK_HOLD_TOLERANCE_MM = 12.0f;  // +/- avant de corriger
// Écart à la fin de l'approche (avant de déployer) : un peu plus large.
constexpr float ATTACK_APPROACH_GAP_MM = ATTACK_HOLD_GAP_MM + 40.0f;
constexpr float ATTACK_STOP_WITHIN_MM = FRONT_AHEAD_OF_AXLE_MM + ATTACK_APPROACH_GAP_MM;
// Coin d'un PAMI adverse depuis son centre : pire cas de sa surface vers
// nous (son orientation n'est pas connue). Gabarit supposé égal au nôtre.
constexpr float ENEMY_CORNER_FROM_CENTER_MM = 106.0f;  // hypot(190 / 2, 95 / 2)
// Cap à tenir face à la cible : au-delà, pivoter ; en dessous, corriger en courbe.
constexpr float ATTACK_HOLD_MAX_HEADING_RAD = 0.35f;  // 20°
// Zone d'attaque d'un coéquipier (table::ATTACK_ZONES) : disque de ce rayon,
// où il stationne et pivote. Les autres PAMI n'y font ni esquive ni
// contournement, et ne prennent pas pour cible ce qu'ils y voient.
constexpr float TEAMMATE_ZONE_RADIUS_MM = 200.0f;
// Un point vu par les capteurs à moins de cette distance du centre d'un PAMI
// adverse (positions de la Jetson) est ce PAMI : demi-longueur + retard et
// erreurs de position.
constexpr float ENEMY_MATCH_RADIUS_MM = PAMI_LENGTH_MM / 2 + 60.0f;
// Avec la position de nos robots (Jetson) : un adversaire à moins de cette
// distance d'un coéquipier est déjà « pris » (contact, blocage) ; un
// coéquipier à moins de ALLY_PATH_CLEARANCE_MM du trajet vers un adversaire
// est sur le chemin. Dans les deux cas, on vise un autre adversaire.
constexpr float ENGAGED_RADIUS_MM = PAMI_LENGTH_MM + 60.0f;
constexpr float ALLY_PATH_CLEARANCE_MM = PAMI_LENGTH_MM / 2 + PAMI_WIDTH_MM / 2 + 20.0f;
// Sans Wi-Fi, les PAMI d'attaque ne chassent aux capteurs qu'à partir de cet
// instant : nos PAMI douve sont passés, ce que voient les capteurs est alors
// plutôt adverse.
constexpr uint32_t ATTACK_HUNT_FROM_MS = 94000;
// ... et seulement de notre côté de la table (x >= cette valeur, repère bleu) :
// les adversaires y passent pour rejoindre nos douves ; plus loin, ce que
// voient les capteurs est plutôt un de nos PAMI douve, arrivé ou bloqué.
constexpr float ATTACK_SENSOR_HUNT_MIN_X_MM = 1350.0f;
// ... et pas sur le trajet de nos PAMI douve (à moins de cette distance de la
// ligne sortie d'écurie -> douve) : un PAMI douve bloqué en route y est.
constexpr float OWN_ROUTE_CLEARANCE_MM = 150.0f;
// Avec les positions adverses de la Jetson (Wi-Fi) : on sait qui est adverse,
// la chasse commence dès l'arrivée dans la zone, jusqu'à cette distance.
constexpr float ATTACK_REMOTE_RANGE_MM = 700.0f;
constexpr uint32_t REMOTE_ENEMIES_MAX_AGE_MS = 300;  // au-delà : positions trop vieilles
// Marge autour des obstacles fixes : un point vu à moins de cette distance
// d'une carrière ou d'un rempart n'est pas pris pour un PAMI adverse
// (positions relevées à +/- 20 mm, erreur des capteurs et de l'odométrie).
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
// Courbes : vers une cible jusqu'à cet écart d'angle, le PAMI part en arc
// (avance et tourne en même temps) au lieu de pivoter sur place puis avancer.
// Réglages comparés en simulation (36 matchs en 6 contre 6, 9 vitesses) :
//   - trajets en courbe jusqu'à 60° : 3 fois plus de douves atteintes qu'à
//     30°, mais plus de frottements entre voisins d'écurie quand ils
//     manœuvrent face à un adversaire (aucun sans adversaire) ;
//   - chasse de l'attaque : 30° (plus large, la cible est moins souvent atteinte) ;
//   - reprises après esquive, contournement ou attente : pivot (en courbe,
//     les voisins d'écurie se gênent davantage).
constexpr float TRAVEL_SMOOTH_MAX_RAD = 1.05f;        // 60°
constexpr float HUNT_SMOOTH_MAX_RAD = 0.52f;          // 30°
constexpr float TRAVEL_REPLAN_SMOOTH_MAX_RAD = 0.0f;  // 0 = pivot
// Douve : à l'approche, on ignore les capteurs (rempart, pierres autour des douves).
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
