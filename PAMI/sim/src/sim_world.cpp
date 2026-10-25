#include "sim_world.h"

#include <stdio.h>

#include <limits>

#include "sim_log.h"

namespace {

const double INF = std::numeric_limits<double>::infinity();
const double LIDAR_FOV_DEG = 100.0;  // champ du GS2
const uint32_t LIDAR_PERIOD_MS = 35;  // ~28 trames/s
const uint32_t COLLISION_PERIOD_MS = 10;

struct Pt {
    double x, y;
};

void corners(const Box& b, Pt out[4]) {
    double c = cos(b.th), s = sin(b.th);
    const double sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
    for (int i = 0; i < 4; ++i) {
        double lx = sx[i] * b.hl, ly = sy[i] * b.hw;
        out[i] = {b.cx + c * lx - s * ly, b.cy + s * lx + c * ly};
    }
}

Box aabb(const table::Rect& r) {
    return {(r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2, 0.0, (r.x1 - r.x0) / 2, (r.y1 - r.y0) / 2};
}

// Séparation selon les axes des deux rectangles (théorème des axes séparateurs)
bool overlapBoxBox(const Box& a, const Box& b) {
    Pt ca[4], cb[4];
    corners(a, ca);
    corners(b, cb);
    const double axes[4] = {a.th, a.th + M_PI_2, b.th, b.th + M_PI_2};
    for (double th : axes) {
        double ax = cos(th), ay = sin(th);
        double minA = INF, maxA = -INF, minB = INF, maxB = -INF;
        for (int i = 0; i < 4; ++i) {
            double pa = ca[i].x * ax + ca[i].y * ay;
            double pb = cb[i].x * ax + cb[i].y * ay;
            minA = std::min(minA, pa);
            maxA = std::max(maxA, pa);
            minB = std::min(minB, pb);
            maxB = std::max(maxB, pb);
        }
        if (maxA < minB || maxB < minA) return false;
    }
    return true;
}

bool overlapBoxCircle(const Box& b, double cx, double cy, double r) {
    double c = cos(b.th), s = sin(b.th);
    double dx = cx - b.cx, dy = cy - b.cy;
    double lx = c * dx + s * dy, ly = -s * dx + c * dy;
    double qx = std::max(-b.hl, std::min(b.hl, lx));
    double qy = std::max(-b.hw, std::min(b.hw, ly));
    return (lx - qx) * (lx - qx) + (ly - qy) * (ly - qy) <= r * r;
}

bool boxOutsideTable(const Box& b) {
    Pt c[4];
    corners(b, c);
    for (const Pt& p : c) {
        if (p.x < 0 || p.x > table::LENGTH_MM || p.y < 0 || p.y > table::WIDTH_MM) return true;
    }
    return false;
}

// Distance le long d'un rayon (direction unitaire) jusqu'à un rectangle orienté.
double rayBox(double ox, double oy, double dx, double dy, const Box& b) {
    double c = cos(b.th), s = sin(b.th);
    double rx = ox - b.cx, ry = oy - b.cy;
    double lox = c * rx + s * ry, loy = -s * rx + c * ry;
    double ldx = c * dx + s * dy, ldy = -s * dx + c * dy;
    double tMin = -INF, tMax = INF;
    const double o[2] = {lox, loy}, d[2] = {ldx, ldy}, h[2] = {b.hl, b.hw};
    for (int i = 0; i < 2; ++i) {
        if (fabs(d[i]) < 1e-12) {
            if (o[i] < -h[i] || o[i] > h[i]) return INF;
        } else {
            double t1 = (-h[i] - o[i]) / d[i], t2 = (h[i] - o[i]) / d[i];
            if (t1 > t2) std::swap(t1, t2);
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            if (tMin > tMax) return INF;
        }
    }
    if (tMax < 0) return INF;
    return tMin > 0 ? tMin : 0.0;  // 0 : le lidar est dedans
}

double rayCircle(double ox, double oy, double dx, double dy, double cx, double cy, double r) {
    double fx = ox - cx, fy = oy - cy;
    double b = fx * dx + fy * dy;
    double c = fx * fx + fy * fy - r * r;
    double disc = b * b - c;
    if (disc < 0) return INF;
    double t = -b - sqrt(disc);
    if (t < 0) t = -b + sqrt(disc);
    return t < 0 ? INF : t;
}

// Bordures de la table (vues de l'intérieur)
double rayEdges(double ox, double oy, double dx, double dy) {
    double t = INF;
    if (dx > 1e-12) t = std::min(t, (table::LENGTH_MM - ox) / dx);
    if (dx < -1e-12) t = std::min(t, -ox / dx);
    if (dy > 1e-12) t = std::min(t, (table::WIDTH_MM - oy) / dy);
    if (dy < -1e-12) t = std::min(t, -oy / dy);
    return t < 0 ? 0.0 : t;
}

std::string fmt(const char* f, double a = 0, double b = 0, double c = 0) {
    char buf[160];
    snprintf(buf, sizeof(buf), f, a, b, c);
    return buf;
}

void jsonString(FILE* f, const std::string& s) {
    fputc('"', f);
    for (char ch : s) {
        if (ch == '"' || ch == '\\') {
            fputc('\\', f);
            fputc(ch, f);
        } else if ((unsigned char)ch < 0x20) {
            fputc(' ', f);
        } else {
            fputc(ch, f);
        }
    }
    fputc('"', f);
}

}  // namespace

// ------------------------------------------------------------------

void SimWorld::addOurPami(App* app, int id) {
    const PamiProfile& p = *app->profile;
    Pose targetBlue = p.role == Role::Attack
                          ? table::ATTACK_ZONES[p.targetIndex % table::ATTACK_ZONE_COUNT]
                      : p.role == Role::Douve
                          ? table::ENEMY_DOUVES[p.targetIndex % table::ENEMY_DOUVE_COUNT]
                          : p.startBlue;
    _ours.push_back({app, id, table::forTeam(targetBlue, _opt.team)});
}

Box SimWorld::ourBox(const OurPami& p) const {
    Pose q = p.app->drive.pose();
    return {q.x, q.y, q.theta, _opt.pamiLengthMm / 2, _opt.pamiWidthMm / 2};
}

Box SimWorld::enemyBox(const EnemyPami& e) const {
    return {e.pose.x, e.pose.y, e.pose.theta, _opt.pamiLengthMm / 2, _opt.pamiWidthMm / 2};
}

void SimWorld::step(uint32_t tMs) {
    if (_collisionFlags.empty()) _collisionFlags.assign(_ours.size() + _enemies.size(), 0);

    moveEnemies(tMs);
    if (tMs % LIDAR_PERIOD_MS == 0) {
        for (size_t i = 0; i < _ours.size(); ++i) {
            if (_ours[i].app->lidarEnabled) scanLidar(_ours[i], i);
        }
    }
    if (tMs % COLLISION_PERIOD_MS == 0) {
        checkCollisions(tMs);
        checkStables(tMs);
    }
    if (tMs >= _opt.recordFromMs && tMs % _opt.recordPeriodMs == 0) record(tMs);
}

// Adversaires : ligne droite vers leur cible à vitesse constante, sans
// évitement (ils traversent ce qu'ils rencontrent : ce sont des leurres).
void SimWorld::moveEnemies(uint32_t tMs) {
    for (EnemyPami& e : _enemies) {
        if (tMs < e.startMs) continue;
        double dx = e.target.x - e.pose.x, dy = e.target.y - e.pose.y;
        double d = hypot(dx, dy);
        if (d < 1.0) continue;
        e.pose.theta = (float)atan2(dy, dx);
        double stepMm = std::min(d, e.speedMmS * 0.001);
        e.pose.x += (float)(dx / d * stepMm);
        e.pose.y += (float)(dy / d * stepMm);
    }
}

double SimWorld::castRay(size_t self, double ox, double oy, double dx, double dy) const {
    double best = rayEdges(ox, oy, dx, dy);
    for (const table::Circle& c : table::TOURS) {
        best = std::min(best, rayCircle(ox, oy, dx, dy, c.x, c.y, c.r));
    }
    for (const table::Rect& r : table::CARRIERES) {
        best = std::min(best, rayBox(ox, oy, dx, dy, aabb(r)));
    }
    for (size_t j = 0; j < _ours.size(); ++j) {
        if (j != self) best = std::min(best, rayBox(ox, oy, dx, dy, ourBox(_ours[j])));
    }
    for (const EnemyPami& e : _enemies) {
        best = std::min(best, rayBox(ox, oy, dx, dy, enemyBox(e)));
    }
    return best;
}

void SimWorld::scanLidar(OurPami& p, size_t index) {
    const LidarGS2::Config& cfg = p.app->lidar.simConfig();
    const uint16_t N = LidarGS2::POINT_COUNT;
    Pose q = p.app->drive.pose();
    double ox = q.x + LIDAR_OFFSET_X_MM * cos(q.theta);
    double oy = q.y + LIDAR_OFFSET_X_MM * sin(q.theta);

    uint16_t dist[N];
    LidarPoint pts[N];
    for (uint16_t k = 0; k < N; ++k) {
        // indice 0 = côté droit (y < 0), comme la caméra 0 du GS2
        double a = (-LIDAR_FOV_DEG / 2 + k * LIDAR_FOV_DEG / (N - 1)) * DEG_TO_RAD;
        double d = castRay(index, ox, oy, cos(q.theta + a), sin(q.theta + a));
        bool inRange = d > cfg.minMm && d < cfg.maxMm;
        dist[k] = inRange ? (uint16_t)d : 0;
        pts[k] = {(float)(d * cos(a)), (float)(d * sin(a)), inRange, false};
    }
    // Point confirmé si un voisin est proche (comme le filtre du vrai driver)
    const float tol2 = (float)cfg.spikeToleranceMm * cfg.spikeToleranceMm;
    for (uint16_t k = 0; k < N; ++k) {
        if (!pts[k].valid) continue;
        for (int j : {(int)k - 1, (int)k + 1}) {
            if (j < 0 || j >= N || !pts[j].valid) continue;
            float dx = pts[k].x - pts[j].x, dy = pts[k].y - pts[j].y;
            if (dx * dx + dy * dy <= tol2) pts[k].confirmed = true;
        }
    }
    p.app->lidar.simSetScan(dist, pts);
}

void SimWorld::checkCollisions(uint32_t) {
    std::fill(_collisionFlags.begin(), _collisionFlags.end(), 0);
    std::set<std::string> before = _activeContacts;
    std::set<std::string> now;
    auto hit = [&](const std::string& key, const std::string& who, const std::string& kind,
                   const std::string& text) {
        now.insert(key);
        if (before.count(key) == 0) simLog(who, kind, text);
    };

    for (size_t i = 0; i < _ours.size(); ++i) {
        const OurPami& p = _ours[i];
        std::string who = pamiName(p.id);
        Box b = ourBox(p);

        if (boxOutsideTable(b)) {
            _collisionFlags[i] = 1;
            hit(who + "|bord", who, "collision", "touche le bord de la table");
        }
        for (const table::Circle& c : table::TOURS) {
            if (overlapBoxCircle(b, c.x, c.y, c.r)) {
                _collisionFlags[i] = 1;
                hit(who + fmt("|tour%.0f,%.0f", c.x, c.y), who, "collision",
                    fmt("touche la tour (%.0f, %.0f)", c.x, c.y));
            }
        }
        for (const table::Rect& r : table::CARRIERES) {
            if (overlapBoxBox(b, aabb(r))) {
                _collisionFlags[i] = 1;
                hit(who + fmt("|carriere%.0f,%.0f", r.x0, r.y0), who, "collision",
                    fmt("touche la carriere x %.0f..%.0f, y %.0f..", r.x0, r.x1, r.y0) +
                        fmt("%.0f", r.y1));
            }
        }
        for (size_t j = i + 1; j < _ours.size(); ++j) {
            if (overlapBoxBox(b, ourBox(_ours[j]))) {
                _collisionFlags[i] = _collisionFlags[j] = 1;
                std::string other = pamiName(_ours[j].id);
                hit(who + "|" + other, who, "collision", "touche " + other + " (coequipier)");
            }
        }
        for (size_t e = 0; e < _enemies.size(); ++e) {
            if (overlapBoxBox(b, enemyBox(_enemies[e]))) {
                _collisionFlags[i] = 1;
                _collisionFlags[_ours.size() + e] = 1;
                hit(who + "|" + _enemies[e].name, who, "contact",
                    "touche " + _enemies[e].name);
            }
        }
    }
    _activeContacts = now;
}

// Règle : sortir de son écurie avant 85 s = retiré du match.
void SimWorld::checkStables(uint32_t tMs) {
    if (tMs >= ATTACK_PHASE_START_MS) return;
    for (const OurPami& p : _ours) {
        if (_leftStableEarly.count(p.id)) continue;
        Pose q = p.app->drive.pose();
        bool inside = false;
        for (const Pose& s : table::STABLES) {
            Pose c = table::forTeam(s, _opt.team);
            if (fabs(q.x - c.x) <= 100.0 && fabs(q.y - c.y) <= 150.0) inside = true;
        }
        if (!inside) {
            _leftStableEarly.insert(p.id);
            simLog(pamiName(p.id), "alerte",
                   fmt("sort de l'ecurie avant 85 s (t = %.1f s) : retire du match", tMs / 1000.0));
        }
    }
}

int SimWorld::actionIndex(const char* name) {
    if (name == nullptr) return -1;
    for (size_t i = 0; i < _actionNames.size(); ++i) {
        if (_actionNames[i] == name) return (int)i;
    }
    _actionNames.push_back(name);
    return (int)_actionNames.size() - 1;
}

void SimWorld::record(uint32_t tMs) {
    Frame f;
    f.t = tMs;
    for (size_t i = 0; i < _ours.size(); ++i) {
        App& a = *_ours[i].app;
        Pose q = a.drive.pose();
        int flags = 0;
        if (a.lidarEnabled && a.lidar.obstacle()) flags |= FLAG_OBSTACLE;
        if (_collisionFlags[i]) flags |= FLAG_COLLISION;
        if (a.drive.isPaused()) flags |= FLAG_PAUSED;
        if (a.drive.isMoving()) flags |= FLAG_MOVING;
        if (a.state == MatchState::Ended) flags |= FLAG_ENDED;
        float values[6] = {q.x, q.y, q.theta, (float)flags,
                           (float)actionIndex(a.strategy.currentActionName()),
                           (float)a.servo.angle()};
        f.values.insert(f.values.end(), values, values + 6);
    }
    for (size_t e = 0; e < _enemies.size(); ++e) {
        const EnemyPami& en = _enemies[e];
        int flags = _collisionFlags[_ours.size() + e] ? FLAG_COLLISION : 0;
        float values[6] = {en.pose.x, en.pose.y, en.pose.theta, (float)flags, -1.0f, 0.0f};
        f.values.insert(f.values.end(), values, values + 6);
    }
    _frames.push_back(f);
}

bool SimWorld::writeOutput(const char* path) const {
    FILE* f = fopen(path, "w");
    if (f == nullptr) return false;

    fprintf(f, "window.SIM_DATA = {\n");
    fprintf(f, "\"team\": \"%s\",\n", _opt.team == Team::Blue ? "bleu" : "jaune");
    fprintf(f, "\"table\": {\"length\": %.0f, \"width\": %.0f},\n", table::LENGTH_MM,
            table::WIDTH_MM);
    fprintf(f,
            "\"pami\": {\"length\": %.0f, \"width\": %.0f, \"lidarOffset\": %.0f, "
            "\"lidarRange\": %u, \"lidarFovDeg\": %.0f, \"corridorDepth\": %u, "
            "\"corridorHalf\": %u},\n",
            _opt.pamiLengthMm, _opt.pamiWidthMm, LIDAR_OFFSET_X_MM, 300u, LIDAR_FOV_DEG,
            (unsigned)LIDAR_OBSTACLE_MM, (unsigned)LIDAR_CORRIDOR_HALF_WIDTH_MM);
    fprintf(f, "\"attackPhaseMs\": %u, \"matchMs\": %u,\n", (unsigned)ATTACK_PHASE_START_MS,
            (unsigned)MATCH_DURATION_MS);

    fprintf(f, "\"carrieres\": [");
    bool first = true;
    for (const table::Rect& r : table::CARRIERES) {
        fprintf(f, "%s[%.0f,%.0f,%.0f,%.0f]", first ? "" : ",", r.x0, r.x1, r.y0, r.y1);
        first = false;
    }
    fprintf(f, "],\n\"tours\": [");
    first = true;
    for (const table::Circle& c : table::TOURS) {
        fprintf(f, "%s[%.0f,%.0f,%.0f]", first ? "" : ",", c.x, c.y, c.r);
        first = false;
    }
    fprintf(f, "],\n");

    fprintf(f, "\"bodies\": [\n");
    const char* sep = "";
    for (size_t i = 0; i < _ours.size(); ++i) {
        const OurPami& p = _ours[i];
        Pose s = table::forTeam(p.app->profile->startBlue, _opt.team);
        fprintf(f,
                "%s{\"name\": \"PAMI %d\", \"side\": \"ours\", \"role\": \"%s\", "
                "\"start\": [%.0f,%.0f], \"target\": [%.0f,%.0f]}",
                sep, p.id, roleName(p.app->role), s.x, s.y, p.target.x, p.target.y);
        sep = ",\n";
    }
    for (const EnemyPami& e : _enemies) {
        fprintf(f,
                "%s{\"name\": \"%s\", \"side\": \"enemy\", \"role\": \"adversaire\", "
                "\"start\": [%.0f,%.0f], \"target\": [%.0f,%.0f]}",
                sep, e.name.c_str(), e.start.x, e.start.y, e.target.x, e.target.y);
        sep = ",\n";
    }
    fprintf(f, "\n],\n");

    fprintf(f, "\"actions\": [");
    for (size_t i = 0; i < _actionNames.size(); ++i) {
        if (i) fputc(',', f);
        jsonString(f, _actionNames[i]);
    }
    fprintf(f, "],\n");

    fprintf(f, "\"frames\": [\n");
    for (size_t k = 0; k < _frames.size(); ++k) {
        const Frame& fr = _frames[k];
        fprintf(f, "[%u", fr.t);
        for (size_t v = 0; v < fr.values.size(); v += 6) {
            fprintf(f, ",%.1f,%.1f,%.4f,%.0f,%.0f,%.0f", fr.values[v], fr.values[v + 1],
                    fr.values[v + 2], fr.values[v + 3], fr.values[v + 4], fr.values[v + 5]);
        }
        fprintf(f, "]%s\n", k + 1 < _frames.size() ? "," : "");
    }
    fprintf(f, "],\n");

    fprintf(f, "\"events\": [\n");
    for (size_t k = 0; k < g_events.size(); ++k) {
        const SimEvent& e = g_events[k];
        fprintf(f, "[%u,", e.t);
        jsonString(f, e.who);
        fputc(',', f);
        jsonString(f, e.kind);
        fputc(',', f);
        jsonString(f, e.text);
        fprintf(f, "]%s\n", k + 1 < g_events.size() ? "," : "");
    }
    fprintf(f, "]\n};\n");
    fclose(f);
    return true;
}

void SimWorld::printSummary() const {
    printf("\n=== Resultat (equipe %s) ===\n", _opt.team == Team::Blue ? "bleue" : "jaune");
    for (const OurPami& p : _ours) {
        Pose q = p.app->drive.pose();
        double d = hypot(q.x - p.target.x, q.y - p.target.y);
        printf("PAMI %d (%-7s) : fin en (%4.0f, %4.0f), a %4.0f mm de sa cible\n", p.id,
               roleName(p.app->role), q.x, q.y, d);
    }
    int collisions = 0, contacts = 0, alerts = 0;
    for (const SimEvent& e : g_events) {
        if (e.kind == "collision") collisions++;
        if (e.kind == "contact") contacts++;
        if (e.kind == "alerte") alerts++;
    }
    printf("Collisions : %d, contacts adverses : %d, alertes : %d\n", collisions, contacts,
           alerts);
    for (const SimEvent& e : g_events) {
        if (e.kind == "log") continue;
        printf("  %6.2f s  %-8s %s\n", e.t / 1000.0, e.who.c_str(), e.text.c_str());
    }
}
