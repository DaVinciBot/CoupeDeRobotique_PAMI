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

// Attaque : zones où chaque PAMI d'attaque attend les PAMI adverses, sur leur
// chemin vers nos douves (côté bleu, x ~ 1950..2400). Une zone par PAMI,
// espacées de 500 mm : plus que la portée du lidar (300 mm), pour qu'un PAMI
// ne prenne pas son voisin pour une cible. À vérifier sur la table.
constexpr Pose ATTACK_ZONES[] = {
    {1800.0f, 500.0f, FACING_ENEMY},
    {1800.0f, 1000.0f, FACING_ENEMY},
    {1800.0f, 1500.0f, FACING_ENEMY},
};
constexpr uint8_t ATTACK_ZONE_COUNT = sizeof(ATTACK_ZONES) / sizeof(ATTACK_ZONES[0]);

// ------------------------------------------------------------------
// Obstacles fixes au départ, en coordonnées absolues (les deux côtés).
// Sert à ne pas prendre une tour ou une carrière pour un PAMI adverse.
// Le château n'y est pas encore : ses murs ne sont pas relevés.
// ------------------------------------------------------------------
struct Rect {
    float x0, x1, y0, y1;
};
struct Circle {
    float x, y, r;
};

constexpr Rect CARRIERES[] = {
    // centrales
    {1445.0f, 1555.0f, 340.0f, 660.0f},
    {1445.0f, 1555.0f, 1340.0f, 1660.0f},
    // en bordure, côté y = 0
    {585.0f, 910.0f, 0.0f, 110.0f},
    {1090.0f, 1410.0f, 0.0f, 110.0f},
    {1590.0f, 1910.0f, 0.0f, 110.0f},
    {2090.0f, 2415.0f, 0.0f, 110.0f},
    // en bordure, côté y = 2000
    {585.0f, 910.0f, 1890.0f, 2000.0f},
    {1090.0f, 1410.0f, 1890.0f, 2000.0f},
    {1590.0f, 1910.0f, 1890.0f, 2000.0f},
    {2090.0f, 2415.0f, 1890.0f, 2000.0f},
};

constexpr Circle TOURS[] = {
    // côté jaune
    {495.0f, 400.0f, 100.0f},
    {495.0f, 1600.0f, 100.0f},
    {850.0f, 755.0f, 100.0f},
    {850.0f, 1245.0f, 100.0f},
    // côté bleu (miroir)
    {2505.0f, 400.0f, 100.0f},
    {2505.0f, 1600.0f, 100.0f},
    {2150.0f, 755.0f, 100.0f},
    {2150.0f, 1245.0f, 100.0f},
};

// Point à moins de `margin` mm du bord de la table ?
inline bool nearEdge(float x, float y, float margin) {
    return x < margin || x > LENGTH_MM - margin || y < margin || y > WIDTH_MM - margin;
}

// Point sur (ou à moins de `margin` mm d') une carrière ou une tour ?
inline bool isFixedObstacle(float x, float y, float margin) {
    for (const Rect& r : CARRIERES) {
        if (x >= r.x0 - margin && x <= r.x1 + margin && y >= r.y0 - margin &&
            y <= r.y1 + margin) {
            return true;
        }
    }
    for (const Circle& c : TOURS) {
        float dx = x - c.x, dy = y - c.y, r = c.r + margin;
        if (dx * dx + dy * dy <= r * r) return true;
    }
    return false;
}

}  // namespace table
