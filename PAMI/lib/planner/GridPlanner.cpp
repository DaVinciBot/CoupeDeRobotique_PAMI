#include "GridPlanner.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr float SQRT2 = 1.41421356f;
// Surcoût d'une case (multiplie la longueur du pas) : marge, robot.
constexpr float MARGIN_COST = 2.0f;
constexpr float OCCUPIED_COST = 60.0f;

}  // namespace

void GridPlanner::init(const std::function<bool(float x, float y)>& isFree) {
    for (int i = 0; i < CELLS; ++i) {
        Point p = centerOf(i);
        _static[i] = isFree(p.x, p.y) ? 0 : 1;
    }
    _ready = true;
}

void GridPlanner::addObstacle(const Obstacle& o) {
    if (_obstacleCount < MAX_OBSTACLES) _obstacles[_obstacleCount++] = o;
}

bool GridPlanner::cellOf(Point p, int& c, int& r) const {
    c = (int)floorf(p.x / CELL_MM);
    r = (int)floorf(p.y / CELL_MM);
    if (c < 0) c = 0;
    if (r < 0) r = 0;
    if (c >= COLS) c = COLS - 1;
    if (r >= ROWS) r = ROWS - 1;
    return true;
}

GridPlanner::Point GridPlanner::centerOf(int i) const {
    return {((i % COLS) + 0.5f) * CELL_MM, ((i / COLS) + 0.5f) * CELL_MM};
}

// Robots : disque autour de chaque position, de maintenant à l'horizon
// (pas de 0,2 s) ; marge autour.
void GridPlanner::buildDynamic() {
    memset(_dynamic, 0, sizeof(_dynamic));
    for (int k = 0; k < _obstacleCount; ++k) {
        const Obstacle& o = _obstacles[k];
        int steps = (int)ceilf(_horizonS / 0.2f);
        for (int s = 0; s <= steps; ++s) {
            float t = s * 0.2f;
            float ox = o.x + o.vx * t, oy = o.y + o.vy * t;
            float outer = o.radius + _softMarginMm;
            int c0 = (int)floorf((ox - outer) / CELL_MM), c1 = (int)floorf((ox + outer) / CELL_MM);
            int r0 = (int)floorf((oy - outer) / CELL_MM), r1 = (int)floorf((oy + outer) / CELL_MM);
            for (int r = r0 < 0 ? 0 : r0; r <= r1 && r < ROWS; ++r) {
                for (int c = c0 < 0 ? 0 : c0; c <= c1 && c < COLS; ++c) {
                    Point p = centerOf(idx(c, r));
                    float d = hypotf(p.x - ox, p.y - oy);
                    uint8_t level = d <= o.radius ? 2 : (d <= outer ? 1 : 0);
                    uint8_t& cell = _dynamic[idx(c, r)];
                    if (level > cell) cell = level;
                }
            }
        }
    }
}

bool GridPlanner::blockedAt(float x, float y) const {
    if (x < 0 || y < 0 || x >= COLS * CELL_MM || y >= ROWS * CELL_MM) return true;
    int i = idx((int)(x / CELL_MM), (int)(y / CELL_MM));
    return _static[i] || _dynamic[i] == 2;
}

bool GridPlanner::lineFree(Point a, Point b, float skipMm) const {
    float d = hypotf(b.x - a.x, b.y - a.y);
    int n = (int)ceilf(d / (CELL_MM / 2));
    for (int k = 1; k <= n; ++k) {
        float f = (float)k / n;
        if (f * d < skipMm) continue;
        if (blockedAt(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y))) return false;
    }
    return true;
}

// Case libre (sans obstacle fixe) la plus proche, en anneaux.
bool GridPlanner::nearestFree(int& c, int& r) const {
    if (!_static[idx(c, r)]) return true;
    for (int ring = 1; ring < 10; ++ring) {
        for (int dr = -ring; dr <= ring; ++dr) {
            for (int dc = -ring; dc <= ring; ++dc) {
                if (abs(dr) != ring && abs(dc) != ring) continue;
                int cc = c + dc, rr = r + dr;
                if (cc < 0 || rr < 0 || cc >= COLS || rr >= ROWS) continue;
                if (!_static[idx(cc, rr)]) {
                    c = cc;
                    r = rr;
                    return true;
                }
            }
        }
    }
    return false;
}

// ---------------- tas indexé (plus petit f en haut)
void GridPlanner::heapUp(int pos) {
    int16_t cell = _heap[pos];
    while (pos > 0) {
        int parent = (pos - 1) / 2;
        if (_f[_heap[parent]] <= _f[cell]) break;
        _heap[pos] = _heap[parent];
        _heapPos[_heap[pos]] = (int16_t)pos;
        pos = parent;
    }
    _heap[pos] = cell;
    _heapPos[cell] = (int16_t)pos;
}

void GridPlanner::heapDown(int pos) {
    int16_t cell = _heap[pos];
    for (;;) {
        int child = 2 * pos + 1;
        if (child >= _heapSize) break;
        if (child + 1 < _heapSize && _f[_heap[child + 1]] < _f[_heap[child]]) ++child;
        if (_f[_heap[child]] >= _f[cell]) break;
        _heap[pos] = _heap[child];
        _heapPos[_heap[pos]] = (int16_t)pos;
        pos = child;
    }
    _heap[pos] = cell;
    _heapPos[cell] = (int16_t)pos;
}

void GridPlanner::heapPush(int16_t cell) {
    if (_heapPos[cell] >= 0) {
        heapUp(_heapPos[cell]);  // f a baissé
        return;
    }
    _heap[_heapSize] = cell;
    heapUp(_heapSize++);
}

int16_t GridPlanner::heapPop() {
    int16_t top = _heap[0];
    _heapPos[top] = -1;
    if (--_heapSize > 0) {
        _heap[0] = _heap[_heapSize];
        heapDown(0);
    }
    return top;
}

int GridPlanner::plan(Point from, Point to) {
    _wpCount = 0;
    if (!_ready) return 0;
    buildDynamic();

    int sc, sr, gc, gr;
    cellOf(from, sc, sr);
    cellOf(to, gc, gr);
    const bool goalOnObstacle = _static[idx(gc, gr)] != 0;
    if (!nearestFree(gc, gr)) return 0;
    const int start = idx(sc, sr), goal = idx(gc, gr);
    // But sur un obstacle fixe : on vise la case libre la plus proche
    const Point goalPoint = goalOnObstacle ? centerOf(goal) : to;

    for (int i = 0; i < CELLS; ++i) {
        _g[i] = INFINITY;
        _parent[i] = -1;
        _closed[i] = 0;
        _heapPos[i] = -1;
    }
    _heapSize = 0;
    auto h = [&](int i) {
        int dc = abs(i % COLS - gc), dr = abs(i / COLS - gr);
        int lo = dc < dr ? dc : dr, hi = dc < dr ? dr : dc;
        return (lo * SQRT2 + (hi - lo)) * CELL_MM;
    };
    _g[start] = 0.0f;
    _f[start] = h(start);
    heapPush((int16_t)start);

    bool found = false;
    while (_heapSize > 0) {
        int cur = heapPop();
        if (cur == goal) {
            found = true;
            break;
        }
        _closed[cur] = 1;
        int cc = cur % COLS, cr = cur / COLS;
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                if (!dr && !dc) continue;
                int nc = cc + dc, nr = cr + dr;
                if (nc < 0 || nr < 0 || nc >= COLS || nr >= ROWS) continue;
                int n = idx(nc, nr);
                if (_closed[n] || _static[n]) continue;
                // pas de coin coupé le long d'un obstacle fixe
                if (dc && dr && (_static[idx(cc + dc, cr)] || _static[idx(cc, cr + dr)])) continue;
                float cost = (dc && dr ? SQRT2 : 1.0f) * CELL_MM;
                if (_dynamic[n] == 2) cost *= 1.0f + OCCUPIED_COST;
                else if (_dynamic[n] == 1) cost *= 1.0f + MARGIN_COST;
                float g = _g[cur] + cost;
                if (g >= _g[n]) continue;
                _g[n] = g;
                _f[n] = g + h(n);
                _parent[n] = (int16_t)cur;
                heapPush((int16_t)n);
            }
        }
    }
    if (!found) return 0;

    // Cases du chemin, du but vers le départ
    int16_t* cells = _pathCells;
    int count = 0;
    for (int i = goal; i != -1 && count < CELLS; i = _parent[i]) cells[count++] = (int16_t)i;

    // Tendre le chemin : depuis le point courant, aller le plus loin
    // possible en ligne droite (vue dégagée), et ainsi de suite.
    Point anchor = from;
    int k = count - 1;  // cells[count - 1] = départ
    float skip = CELL_MM * 1.5f;  // départ : peut être collé à un robot
    while (k > 0 && _wpCount < MAX_WAYPOINTS - 1) {
        int best = k - 1;
        for (int j = 0; j < k; ++j) {  // le plus loin d'abord
            Point p = j == 0 ? goalPoint : centerOf(cells[j]);
            if (lineFree(anchor, p, skip)) {
                best = j;
                break;
            }
        }
        Point p = best == 0 ? goalPoint : centerOf(cells[best]);
        _wp[_wpCount++] = p;
        anchor = p;
        skip = 0.0f;
        k = best;
    }
    if (_wpCount == 0 || _wp[_wpCount - 1].x != goalPoint.x || _wp[_wpCount - 1].y != goalPoint.y) {
        _wp[_wpCount < MAX_WAYPOINTS ? _wpCount++ : MAX_WAYPOINTS - 1] = goalPoint;
    }
    return _wpCount;
}
