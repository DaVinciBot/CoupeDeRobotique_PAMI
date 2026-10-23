// Positions sur la table Eurobot 2027 (3000 x 2000 mm), relevées sur la
// figure 8 du règlement officiel 1.0 (précision ~ +/- 20 mm).
//
// Repère du plan : x sur la longueur (0..3000), y sur la largeur (0..2000),
// theta en radians, sens trigo. Sur la figure 8, la table est dessinée
// tournée : x vers le haut, y vers la gauche.
//
//   - équipe JAUNE : côté x = 0     (salle du trône centrée en (250, 1000))
//   - équipe BLEUE : côté x = 3000  (salle du trône centrée en (2750, 1000))
//
// Les positions ci-dessous sont décrites pour l'équipe BLEUE (qui attaque le
// château jaune, côté x petit). forTeam() fait le miroir pour l'équipe jaune.
#pragma once

#include <stdint.h>

#include "Pose.h"

enum class Team : uint8_t { Blue, Yellow };

inline const char* teamName(Team t) {
    return t == Team::Yellow ? "JAUNE" : "BLEU";
}

namespace table {

constexpr float LENGTH_MM = 3000.0f;
constexpr float WIDTH_MM = 2000.0f;

inline Pose mirror(const Pose& p) {
    return {LENGTH_MM - p.x, p.y, wrapPi((float)M_PI - p.theta)};
}

inline Pose forTeam(const Pose& bluePose, Team team) {
    return team == Team::Yellow ? mirror(bluePose) : bluePose;
}

constexpr float FACING_ENEMY = (float)M_PI;  // bleu : vers les x décroissants

// Salle du trône bleue (50 x 50 cm), centre.
constexpr Pose THRONE_CENTER = {2750.0f, 1000.0f, 0.0f};

// Écuries bleues (200 x 300 mm) : dans les deux coins du côté bleu.
//   écurie 0 : x 2800..3000, y    0..300
//   écurie 1 : x 2800..3000, y 1700..2000
constexpr Pose STABLES[] = {
    {2900.0f, 150.0f, FACING_ENEMY},
    {2900.0f, 1850.0f, FACING_ENEMY},
};

// Douves du château JAUNE (5 pts par douve occupée, une partie du PAMI suffit).
// On vise leur centre.
//   0 : douve diagonale côté y petit (rectangle penché, x ~610..1020)
//   1 : douve du pont (x 955..1145, y 795..1190)
//   2 : douve diagonale côté y grand
constexpr Pose ENEMY_DOUVES[] = {
    {815.0f, 425.0f, 0.0f},
    {1050.0f, 1000.0f, 0.0f},
    {815.0f, 1575.0f, 0.0f},
};
constexpr uint8_t ENEMY_DOUVE_COUNT = sizeof(ENEMY_DOUVES) / sizeof(ENEMY_DOUVES[0]);

// Attaque : où chercher un PAMI adverse tant que le lidar ne voit rien
// (centre table).
constexpr Pose ATTACK_DEFAULT_TARGET = {1500.0f, 1000.0f, 0.0f};

// Obstacles fixes au départ (pour info / simulateur) :
//   carrières centrales : x 1445..1555, y 340..660 et y 1340..1660
//   carrières en bordure (y 0..110 et y 1890..2000) :
//       x 585..910, 1090..1410, 1590..1910, 2090..2415
//   tours (cercles d = 200) côté jaune : (495, 400), (495, 1600),
//       (850, 755), (850, 1245) ; miroir côté bleu.

}  // namespace table
