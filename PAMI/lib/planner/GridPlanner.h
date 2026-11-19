// Recherche de chemin sur la table (A* sur une grille), pour les PAMI qui
// connaissent la position des autres robots (Wi-Fi 6E, Jetson connectée).
//
// Le chemin est calculé pour le CENTRE du PAMI :
//   - obstacles fixes (murailles, bords) : carte calculée une fois (init) ;
//   - robots vus par la Jetson : disques autour de leur position actuelle ET
//     de leurs positions prévues (vitesse x horizon), pour passer là où ils
//     ne seront plus, pas là où ils sont ;
//   - une marge « molle » autour de tout ça : le chemin s'en écarte s'il peut.
// Les cases occupées par un robot restent franchissables à très grand coût
// (le PAMI peut démarrer collé à un robot : il faut pouvoir s'en dégager).
// Le chemin brut (cases) est ensuite tendu (lignes droites tant que la vue
// est dégagée) : il ne reste que quelques points de passage.
//
// Sans dépendance à Arduino (simulateur, tests). Mémoire : ~40 ko, réservés
// une fois.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <functional>

class GridPlanner {
   public:
    static constexpr float CELL_MM = 50.0f;
    static constexpr int COLS = 60;  // 3000 mm
    static constexpr int ROWS = 40;  // 2000 mm
    static constexpr int CELLS = COLS * ROWS;
    static constexpr int MAX_WAYPOINTS = 24;
    static constexpr int MAX_OBSTACLES = 16;

    struct Point {
        float x, y;
    };

    // Robot à éviter : position, vitesse (mm/s), rayon (mm, déjà agrandi de
    // la demi-taille de notre PAMI).
    struct Obstacle {
        float x, y, vx, vy, radius;
    };

    // `isFree(x, y)` : le centre du PAMI peut-il être en (x, y) sans toucher
    // un obstacle fixe ? Appelé une fois par case.
    void init(const std::function<bool(float x, float y)>& isFree);
    bool ready() const { return _ready; }

    void clearObstacles() { _obstacleCount = 0; }
    void addObstacle(const Obstacle& o);
    // Prévision : chaque obstacle occupe aussi ses positions jusqu'à
    // `horizonS` secondes dans le futur.
    void setHorizon(float horizonS) { _horizonS = horizonS; }
    void setSoftMargin(float mm) { _softMarginMm = mm; }

    // Chemin de `from` à `to` (centres du PAMI). Renvoie le nombre de points
    // (le dernier est `to`, ou la case libre la plus proche), 0 si aucun.
    int plan(Point from, Point to);
    const Point* waypoints() const { return _wp; }
    int waypointCount() const { return _wpCount; }

    // Ligne droite sans obstacle fixe ni robot (en ignorant les `skipMm`
    // premiers mm : le PAMI peut y être déjà collé) ?
    bool lineFree(Point a, Point b, float skipMm = 0.0f) const;

   private:
    int idx(int c, int r) const { return r * COLS + c; }
    bool cellOf(Point p, int& c, int& r) const;
    Point centerOf(int i) const;
    bool blockedAt(float x, float y) const;
    void buildDynamic();
    bool nearestFree(int& c, int& r) const;

    void heapPush(int16_t cell);
    void heapUp(int pos);
    void heapDown(int pos);
    int16_t heapPop();

    bool _ready = false;
    uint8_t _static[CELLS] = {};   // 1 = obstacle fixe
    uint8_t _dynamic[CELLS] = {};  // 0 libre, 1 marge, 2 occupé (robot)
    Obstacle _obstacles[MAX_OBSTACLES] = {};
    int _obstacleCount = 0;
    float _horizonS = 1.0f;
    float _softMarginMm = 80.0f;

    // A* (tas indexé)
    float _g[CELLS];
    float _f[CELLS];
    int16_t _parent[CELLS];
    uint8_t _closed[CELLS];
    int16_t _heap[CELLS];
    int16_t _heapPos[CELLS];  // -1 : pas dans le tas
    int _heapSize = 0;
    int16_t _pathCells[CELLS];  // cases du chemin trouvé (but -> départ)

    Point _wp[MAX_WAYPOINTS] = {};
    int _wpCount = 0;
};
