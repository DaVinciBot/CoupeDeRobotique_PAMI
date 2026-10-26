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

// Bordures de la table (vues de l'intérieur)
double rayEdges(double ox, double oy, double dx, double dy) {
    double t = INF;
    if (dx > 1e-12) t = std::min(t, (table::LENGTH_MM - ox) / dx);
    if (dx < -1e-12) t = std::min(t, -ox / dx);
    if (dy > 1e-12) t = std::min(t, (table::WIDTH_MM - oy) / dy);
    if (dy < -1e-12) t = std::min(t, -oy / dy);
    return t < 0 ? 0.0 : t;
}

// Murailles des deux châteaux : d'abord celles du château jaune, puis leur
// miroir (château bleu).
const std::vector<Box>& wallBoxes() {
    static std::vector<Box> walls;
    if (walls.empty()) {
        for (int side = 0; side < 2; ++side) {
            for (const table::Wall& w : table::MURAILLES) {
                double cx = side ? table::LENGTH_MM - w.cx : w.cx;
                double th = side ? M_PI - w.angle : w.angle;
                walls.push_back({cx, w.cy, th, w.length / 2.0, w.width / 2.0});
            }
        }
    }
    return walls;
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

void SimWorld::addPami(App* app, int logId, bool ours, Team team) {
    const PamiProfile& p = *app->profile;
    Pose targetBlue = p.role == Role::Attack
                          ? table::ATTACK_ZONES[p.targetIndex % table::ATTACK_ZONE_COUNT]
                      : p.role == Role::Douve
                          ? table::ENEMY_DOUVES[p.targetIndex % table::ENEMY_DOUVE_COUNT]
                          : p.startBlue;
    SimPami pami = {};
    pami.app = app;
    pami.logId = logId;
    pami.ours = ours;
    pami.team = team;
    pami.target = table::forTeam(targetBlue, team);
    _pamis.push_back(pami);
}

// Pose = axe des roues ; le centre du rectangle est à
// PAMI_AXLE_FROM_CENTER_MM derrière.
Box SimWorld::boxAt(const Pose& q) const {
    double cx = q.x - PAMI_AXLE_FROM_CENTER_MM * cos(q.theta);
    double cy = q.y - PAMI_AXLE_FROM_CENTER_MM * sin(q.theta);
    return {cx, cy, q.theta, _opt.pamiLengthMm / 2, _opt.pamiWidthMm / 2};
}

// Rectangle à la VRAIE position du PAMI.
Box SimWorld::pamiBox(const SimPami& p) const {
    return boxAt(p.truePose);
}

// Applique à la vraie position de chaque PAMI le déplacement mesuré par son
// odométrie depuis le pas précédent, sauf s'il le ferait entrer dans un autre
// PAMI ou sortir de la table : il reste alors bloqué (les roues patinent).
// Les murailles, elles, ne bloquent pas : elles tombent (signalé ailleurs).
void SimWorld::moveBodies() {
    for (size_t i = 0; i < _pamis.size(); ++i) {
        SimPami& p = _pamis[i];
        Pose odom = p.app->drive.pose();
        if (!p.started) {  // départ : vraie position = position annoncée
            p.truePose = odom;
            p.lastOdom = odom;
            p.started = true;
            continue;
        }

        // Déplacement dans le repère du PAMI (avance, côté, rotation)
        double c0 = cos(p.lastOdom.theta), s0 = sin(p.lastOdom.theta);
        double dx = odom.x - p.lastOdom.x, dy = odom.y - p.lastOdom.y;
        double ahead = c0 * dx + s0 * dy, side = -s0 * dx + c0 * dy;
        double dTheta = wrapPi(odom.theta - p.lastOdom.theta);
        p.lastOdom = odom;
        p.blocked = false;
        if (ahead == 0.0 && side == 0.0 && dTheta == 0.0) continue;

        double c = cos(p.truePose.theta), s = sin(p.truePose.theta);
        Pose next = {(float)(p.truePose.x + c * ahead - s * side),
                     (float)(p.truePose.y + s * ahead + c * side),
                     wrapPi(p.truePose.theta + (float)dTheta)};
        Box nb = boxAt(next);
        bool free = !boxOutsideTable(nb) || boxOutsideTable(pamiBox(p));
        for (size_t j = 0; free && j < _pamis.size(); ++j) {
            if (j != i && overlapBoxBox(nb, pamiBox(_pamis[j]))) free = false;
        }
        if (free) {
            p.truePose = next;
        } else {
            p.blocked = true;
        }
        float drift = hypotf(odom.x - p.truePose.x, odom.y - p.truePose.y);
        if (drift > p.maxDriftMm) p.maxDriftMm = drift;
    }
}

// Ce que la Jetson enverrait en Wi-Fi : appelle les mêmes fonctions que
// src/wifi.cpp sur le robot (remote.cpp).
void SimWorld::jetson(uint32_t tMs) {
    if (tMs < ATTACK_PHASE_START_MS) return;  // les PAMI sont dans les écuries
    bool enemies = tMs % _opt.jetsonEnemiesPeriodMs == 0;
    bool odometry = tMs % _opt.jetsonOdometryPeriodMs == 0;
    if (!enemies && !odometry) return;

    for (SimPami& p : _pamis) {
        if (!p.started) continue;
        g_currentPami = p.logId;
        if (enemies) {  // centres des PAMI de l'autre équipe
            float xs[RemoteEnemies::MAX], ys[RemoteEnemies::MAX];
            uint8_t n = 0;
            for (const SimPami& o : _pamis) {
                if (o.ours == p.ours || n >= RemoteEnemies::MAX) continue;
                Box b = pamiBox(o);
                xs[n] = (float)b.cx;
                ys[n] = (float)b.cy;
                ++n;
            }
            remoteEnemyPositions(*p.app, xs, ys, n);
        }
        if (odometry) {  // recalage sur la vraie position
            remoteSetOdometry(*p.app, p.truePose);
            p.lastOdom = p.truePose;  // l'odométrie repart d'ici : pas de saut
        }
    }
}

void SimWorld::step(uint32_t tMs) {
    if (_collisionFlags.empty()) _collisionFlags.assign(_pamis.size(), 0);
    moveBodies();
    if (_opt.jetson) jetson(tMs);

    if (tMs % LIDAR_PERIOD_MS == 0) {
        for (size_t i = 0; i < _pamis.size(); ++i) {
            if (_pamis[i].app->lidarEnabled) scanLidar(_pamis[i], i);
        }
    }
    if (tMs % COLLISION_PERIOD_MS == 0) {
        checkCollisions(tMs);
        checkStables(tMs);
    }
    if (tMs >= _opt.recordFromMs && tMs % _opt.recordPeriodMs == 0) record(tMs);
}

double SimWorld::castRay(size_t self, double ox, double oy, double dx, double dy) const {
    double best = rayEdges(ox, oy, dx, dy);
    for (const Box& w : wallBoxes()) {
        best = std::min(best, rayBox(ox, oy, dx, dy, w));
    }
    for (const table::Rect& r : table::CARRIERES) {
        if (!table::CARRIERES_SUR_LA_TABLE) break;
        best = std::min(best, rayBox(ox, oy, dx, dy, aabb(r)));
    }
    for (size_t j = 0; j < _pamis.size(); ++j) {
        if (j != self) best = std::min(best, rayBox(ox, oy, dx, dy, pamiBox(_pamis[j])));
    }
    return best;
}

void SimWorld::scanLidar(SimPami& p, size_t index) {
    const LidarGS2::Config& cfg = p.app->lidar.simConfig();
    const uint16_t N = LidarGS2::POINT_COUNT;
    Pose q = p.truePose;  // le lidar voit depuis là où le PAMI est vraiment
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

    const std::vector<Box>& walls = wallBoxes();
    for (size_t i = 0; i < _pamis.size(); ++i) {
        const SimPami& p = _pamis[i];
        std::string who = pamiName(p.logId);
        Box b = pamiBox(p);

        if (boxOutsideTable(b)) {
            _collisionFlags[i] = 1;
            hit(who + "|bord", who, "collision", "touche le bord de la table");
        }
        for (size_t k = 0; k < walls.size(); ++k) {
            if (!overlapBoxBox(b, walls[k])) continue;
            _collisionFlags[i] = 1;
            bool blue = k >= walls.size() / 2;
            hit(who + fmt("|muraille%.0f", (double)k), who, "collision",
                std::string("touche une muraille du chateau ") + (blue ? "bleu" : "jaune") +
                    fmt(" (%.0f, %.0f) : elle tombe !", walls[k].cx, walls[k].cy));
        }
        for (const table::Rect& r : table::CARRIERES) {
            if (!table::CARRIERES_SUR_LA_TABLE) break;
            if (overlapBoxBox(b, aabb(r))) {
                _collisionFlags[i] = 1;
                hit(who + fmt("|carriere%.0f,%.0f", r.x0, r.y0), who, "collision",
                    fmt("touche la carriere x %.0f..%.0f, y %.0f..", r.x0, r.x1, r.y0) +
                        fmt("%.0f", r.y1));
            }
        }
        Box touch = b;  // agrandi de 2 mm : les PAMI se touchent sans se recouvrir
        touch.hl += 2.0;
        touch.hw += 2.0;
        if (p.blocked && boxOutsideTable(touch)) {
            _collisionFlags[i] = 1;
            hit(who + "|bord", who, "collision", "bloque contre le bord de la table");
        }
        for (size_t j = i + 1; j < _pamis.size(); ++j) {
            if (!overlapBoxBox(touch, pamiBox(_pamis[j]))) continue;
            _collisionFlags[i] = _collisionFlags[j] = 1;
            std::string other = pamiName(_pamis[j].logId);
            if (_pamis[j].ours == p.ours) {
                hit(who + "|" + other, who, "collision", "touche " + other + " (coequipier)");
            } else {
                // contact entre équipes : rapporté du point de vue de notre PAMI
                const std::string& mine = p.ours ? who : other;
                const std::string& theirs = p.ours ? other : who;
                hit(who + "|" + other, mine, "contact", "touche " + theirs);
            }
        }
    }
    _activeContacts = now;
}

// Règle : sortir de son écurie avant 85 s = retiré du match.
void SimWorld::checkStables(uint32_t tMs) {
    if (tMs >= ATTACK_PHASE_START_MS) return;
    for (const SimPami& p : _pamis) {
        if (_leftStableEarly.count(p.logId)) continue;
        Box b = pamiBox(p);
        bool inside = false;
        for (const Pose& s : table::STABLES) {
            Pose c = table::forTeam(s, p.team);
            if (fabs(b.cx - c.x) <= 100.0 && fabs(b.cy - c.y) <= 150.0) inside = true;
        }
        if (!inside) {
            _leftStableEarly.insert(p.logId);
            simLog(pamiName(p.logId), "alerte",
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
    for (size_t i = 0; i < _pamis.size(); ++i) {
        App& a = *_pamis[i].app;
        Pose q = _pamis[i].truePose;
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
    _frames.push_back(f);
}

bool SimWorld::writeOutput(const char* path, const char* name) const {
    FILE* f = fopen(path, "w");
    if (f == nullptr) return false;

    fprintf(f, "window.SIM_RUNS = window.SIM_RUNS || {};\n");
    fprintf(f, "window.SIM_RUNS[\"%s\"] = {\n", name);
    fprintf(f, "\"team\": \"%s\",\n", _opt.team == Team::Blue ? "bleu" : "jaune");
    fprintf(f, "\"table\": {\"length\": %.0f, \"width\": %.0f},\n", table::LENGTH_MM,
            table::WIDTH_MM);
    fprintf(f,
            "\"pami\": {\"length\": %.0f, \"width\": %.0f, \"lidarOffset\": %.0f, "
            "\"lidarRange\": %u, \"lidarFovDeg\": %.0f, \"corridorDepth\": %u, "
            "\"corridorHalf\": %u, \"axle\": %.0f},\n",
            _opt.pamiLengthMm, _opt.pamiWidthMm, LIDAR_OFFSET_X_MM, 300u, LIDAR_FOV_DEG,
            (unsigned)LIDAR_OBSTACLE_MM, (unsigned)LIDAR_CORRIDOR_HALF_WIDTH_MM,
            (double)PAMI_AXLE_FROM_CENTER_MM);
    fprintf(f, "\"attackPhaseMs\": %u, \"matchMs\": %u, \"jetson\": %s,\n",
            (unsigned)ATTACK_PHASE_START_MS, (unsigned)MATCH_DURATION_MS,
            _opt.jetson ? "true" : "false");

    fprintf(f, "\"carrieres\": [");
    bool first = true;
    for (const table::Rect& r : table::CARRIERES) {
        if (!table::CARRIERES_SUR_LA_TABLE) break;
        fprintf(f, "%s[%.0f,%.0f,%.0f,%.0f]", first ? "" : ",", r.x0, r.x1, r.y0, r.y1);
        first = false;
    }
    // murailles : centre x, centre y, angle, longueur, largeur
    fprintf(f, "],\n\"murailles\": [");
    first = true;
    for (const Box& w : wallBoxes()) {
        fprintf(f, "%s[%.1f,%.1f,%.4f,%.0f,%.0f]", first ? "" : ",", w.cx, w.cy, w.th, 2 * w.hl,
                2 * w.hw);
        first = false;
    }
    fprintf(f, "],\n");

    fprintf(f, "\"bodies\": [\n");
    const char* sep = "";
    for (const SimPami& p : _pamis) {
        const PamiProfile& prof = *p.app->profile;
        Pose s = table::forTeam(prof.startBlue, p.team);
        fprintf(f, "%s{\"name\": ", sep);
        jsonString(f, pamiName(p.logId));
        fprintf(f,
                ", \"side\": \"%s\", \"role\": \"%s\", \"start\": [%.0f,%.0f], "
                "\"target\": [%.0f,%.0f], \"route\": [",
                p.ours ? "ours" : "enemy", roleName(p.app->role), s.x, s.y, p.target.x,
                p.target.y);
        // sortie d'écurie tout droit, puis points de passage
        Pose exitPose = {s.x + prof.exitMm * cosf(s.theta), s.y + prof.exitMm * sinf(s.theta), 0};
        fprintf(f, "[%.0f,%.0f]", exitPose.x, exitPose.y);
        for (uint8_t k = 0; k < prof.routeLength; ++k) {
            Pose q = table::forTeam({prof.route[k].x, prof.route[k].y, 0.0f}, p.team);
            fprintf(f, ",[%.0f,%.0f]", q.x, q.y);
        }
        fprintf(f, "]}");
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
    for (int side = 0; side < 2; ++side) {
        bool ours = side == 0;
        bool any = false;
        for (const SimPami& p : _pamis) {
            if (p.ours != ours) continue;
            if (!any) {
                printf("\n=== %s (equipe %s) ===\n", ours ? "Nos PAMI" : "Adversaires",
                       p.team == Team::Blue ? "bleue" : "jaune");
                any = true;
            }
            Pose q = p.truePose;
            Box body = pamiBox(p);  // distance mesurée depuis le centre du PAMI
            double d = hypot(body.cx - p.target.x, body.cy - p.target.y);
            printf("%-8s (%-7s) : fin en (%4.0f, %4.0f), a %4.0f mm de sa cible",
                   pamiName(p.logId).c_str(), roleName(p.app->role), q.x, q.y, d);
            if (p.maxDriftMm >= 5.0f) {
                printf("  (bloque : odometrie fausse de %.0f mm)", p.maxDriftMm);
            }
            printf("\n");
        }
    }
    int collisions = 0, contacts = 0, alerts = 0;
    for (const SimEvent& e : g_events) {
        if (e.kind == "collision") collisions++;
        if (e.kind == "contact") contacts++;
        if (e.kind == "alerte") alerts++;
    }
    printf("\nJetson simulee (Wi-Fi) : %s\n",
           _opt.jetson ? "oui (positions adverses 10 Hz, recalage 1 Hz)" : "non");
    printf("Collisions : %d, contacts entre equipes : %d, alertes : %d\n", collisions,
           contacts, alerts);
    for (const SimEvent& e : g_events) {
        if (e.kind == "log") continue;
        printf("  %6.2f s  %-8s %s\n", e.t / 1000.0, e.who.c_str(), e.text.c_str());
    }
}
