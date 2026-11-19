// Stratégies de match par rôle.
// Toutes commencent par attendre la phase d'attaque (85 s).
//
// Quand le module de communication sera choisi, une cible reçue en cours de
// route pourra être donnée à GoTo via Options::provider (comme les capteurs pour
// l'attaque).
#include "Actions.h"
#include "app.h"

namespace {

// Point vu par les capteurs -> position sur la table.
Pose sensorToTable(const Pose& robot, float lx, float ly) {
    float x = lx + SENSOR_OFFSET_X_MM;  // repère robot (axe des roues)
    float c = cosf(robot.theta), s = sinf(robot.theta);
    return {robot.x + c * x - s * ly, robot.y + s * x + c * ly, 0.0f};
}

bool inTeammateAttackZone(const App& a, float x, float y);

float distToSegment(float px, float py, float ax, float ay, float bx, float by) {
    float dx = bx - ax, dy = by - ay;
    float len2 = dx * dx + dy * dy;
    float t = len2 > 0.0f ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0f;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return hypotf(px - (ax + t * dx), py - (ay + t * dy));
}

// Sans Jetson : point sur le trajet d'un de nos PAMI douve (sortie d'écurie,
// points de passage, puis sa douve) ? Bloqué en route, il serait pris pour
// cible par nos PAMI d'attaque.
bool onTeammateDouveRoute(const App& a, float x, float y) {
    for (const PamiProfile& p : PAMI_PROFILES) {
        if (p.role != Role::Douve) continue;
        Pose s = table::forTeam(p.startBlue, a.team);
        Pose from = {s.x + p.exitMm * cosf(s.theta), s.y + p.exitMm * sinf(s.theta), 0.0f};
        for (uint8_t i = 0; i <= p.routeLength; ++i) {
            Pose to = i < p.routeLength
                          ? table::forTeam({p.route[i].x, p.route[i].y, 0.0f}, a.team)
                          : table::forTeam(table::ENEMY_DOUVES[p.targetIndex % table::ENEMY_DOUVE_COUNT],
                                           a.team);
            if (distToSegment(x, y, from.x, from.y, to.x, to.y) <= OWN_ROUTE_CLEARANCE_MM) {
                return true;
            }
            from = to;
        }
    }
    return false;
}

// Avec les positions de nos robots (Jetson) : cet adversaire est-il déjà au
// contact d'un coéquipier, ou un coéquipier est-il sur le chemin ? Le viser
// ferait percuter le coéquipier.
bool allyInTheWay(const App& a, const Pose& robot, float ex, float ey) {
    if (!a.allies.fresh(REMOTE_ENEMIES_MAX_AGE_MS)) return false;
    for (uint8_t i = 0; i < a.allies.count; ++i) {
        float ax = a.allies.x[i], ay = a.allies.y[i];
        if (hypotf(ax - ex, ay - ey) <= ENGAGED_RADIUS_MM) return true;
        if (distToSegment(ax, ay, robot.x, robot.y, ex, ey) <= ALLY_PATH_CLEARANCE_MM) return true;
    }
    return false;
}

// Cible de l'attaque :
//  1) si la Jetson envoie les positions adverses (Wi-Fi) : le PAMI adverse le
//     plus proche, à portée, qui n'est ni au contact d'un coéquipier ni
//     derrière lui ; on sait alors qui est adversaire, on peut chasser tout
//     de suite ;
//  2) sinon, à partir de ATTACK_HUNT_FROM_MS : le point le plus proche vu par
//     les capteurs qui n'est ni un bord ni un obstacle connu (avant, ce serait
//     souvent un de nos PAMI douve : les capteurs ne distingue pas les équipes).
GoTo::TargetProvider nearestEnemyCandidate(App& a) {
    return [&a](const Pose& robot, Pose& out) {
        if (a.enemies.fresh(REMOTE_ENEMIES_MAX_AGE_MS)) {
            float best = ATTACK_REMOTE_RANGE_MM;
            bool found = false;
            // Cible collante : l'adversaire déjà visé (le plus proche de sa
            // dernière position) garde l'avantage de ATTACK_TARGET_STICKY_MM,
            // sinon le PAMI hésite entre deux adversaires à égale distance et
            // pivote d'un côté puis de l'autre sans avancer.
            int current = -1;
            if (a.hunt.valid && millis() - a.hunt.timeMs < 1000) {
                float bestMatch = 150.0f;
                for (uint8_t i = 0; i < a.enemies.count; ++i) {
                    float m = hypotf(a.enemies.x[i] - a.hunt.x, a.enemies.y[i] - a.hunt.y);
                    if (m < bestMatch) {
                        bestMatch = m;
                        current = i;
                    }
                }
            }
            int chosen = -1;
            for (uint8_t i = 0; i < a.enemies.count; ++i) {
                float dx = a.enemies.x[i] - robot.x, dy = a.enemies.y[i] - robot.y;
                float d = hypotf(dx, dy) - (i == current ? ATTACK_TARGET_STICKY_MM : 0.0f);
                if (d < best && hypotf(dx, dy) > 1.0f &&
                    !allyInTheWay(a, robot, a.enemies.x[i], a.enemies.y[i])) {
                    best = d;
                    chosen = i;
                    d = hypotf(dx, dy);
                    // viser sa surface, pas son centre (stopWithin compte depuis
                    // la surface). On ne connaît pas son orientation : on prend
                    // le pire cas, son coin (le plus loin de son centre, donc le
                    // plus près de nous), pour ne jamais le toucher avec la
                    // carrosserie ; les capteurs donnent ensuite la vraie
                    // distance (KeepDistance).
                    float back = ENEMY_CORNER_FROM_CENTER_MM / d;
                    out = {a.enemies.x[i] - dx * back, a.enemies.y[i] - dy * back, 0.0f};
                    found = true;
                }
            }
            a.hunt.valid = found;
            if (found) {
                a.hunt.x = a.enemies.x[chosen];
                a.hunt.y = a.enemies.y[chosen];
                a.hunt.timeMs = millis();
            }
            return found;
        }
        if (a.clock.elapsedMs() < ATTACK_HUNT_FROM_MS) return false;
        if (!a.sensorsEnabled || !a.tof.healthy()) return false;
        constexpr float EDGE_MARGIN_MM = 40.0f;

        ObstaclePoint pts[ObstacleSensor::MAX_POINTS] = {};  // au-delà des points : non valides
        a.tof.copyPoints(pts);
        bool found = false;
        float bestD2 = INFINITY;
        for (const ObstaclePoint& lp : pts) {
            if (!lp.valid || !lp.confirmed) continue;
            Pose p = sensorToTable(robot, lp.x, lp.y);
            // x dans le repère bleu : nos douves (que visent les adversaires)
            // sont du côté des x grands
            const float xBlue = a.team == Team::Yellow ? table::LENGTH_MM - p.x : p.x;
            if (table::nearEdge(p.x, p.y, EDGE_MARGIN_MM) ||
                table::isFixedObstacle(p.x, p.y, ATTACK_FIXED_OBSTACLE_MARGIN_MM) ||
                inTeammateAttackZone(a, p.x, p.y) ||  // un de nos PAMI d'attaque
                xBlue < ATTACK_SENSOR_HUNT_MIN_X_MM ||  // côté adverse : nos PAMI douve
                onTeammateDouveRoute(a, p.x, p.y)) {  // un de nos PAMI douve, bloqué en route
                continue;
            }
            float d2 = lp.x * lp.x + lp.y * lp.y;
            if (d2 < bestD2) {
                bestD2 = d2;
                out = p;
                found = true;
            }
        }
        return found;
    };
}

// Point dans la zone d'attaque d'un coéquipier ? Un PAMI d'attaque y
// stationne et y pivote : les autres n'y font ni esquive, ni contournement,
// ni courbe, et ne le prennent pas pour cible.
bool inTeammateAttackZone(const App& a, float x, float y) {
    const int own = a.role == Role::Attack ? a.profile->targetIndex % table::ATTACK_ZONE_COUNT : -1;
    for (uint8_t i = 0; i < table::ATTACK_ZONE_COUNT; ++i) {
        if (i == own) continue;
        Pose z = table::forTeam(table::ATTACK_ZONES[i], a.team);
        if (hypotf(x - z.x, y - z.y) <= TEAMMATE_ZONE_RADIUS_MM) return true;
    }
    return false;
}

// Le PAMI tient-il à cette pose sans toucher muraille, carrière ou bord ?
// On teste son contour (coins, milieux des côtés) et son centre.
// Avec `a` : ni dans la zone d'attaque d'un coéquipier.
bool isPoseFree(const Pose& axle, const App* a = nullptr) {
    constexpr float MARGIN_MM = 10.0f;
    const float c = cosf(axle.theta), s = sinf(axle.theta);
    const float front = PAMI_LENGTH_MM / 2 - PAMI_AXLE_FROM_CENTER_MM;
    const float back = -PAMI_LENGTH_MM / 2 - PAMI_AXLE_FROM_CENTER_MM;
    const float half = PAMI_WIDTH_MM / 2;
    const float along[3] = {back, (front + back) / 2, front};
    const float across[3] = {-half, 0.0f, half};
    for (float u : along) {
        for (float v : across) {
            float x = axle.x + c * u - s * v, y = axle.y + s * u + c * v;
            if (table::nearEdge(x, y, MARGIN_MM) || table::isFixedObstacle(x, y, MARGIN_MM) ||
                (a != nullptr && inTeammateAttackZone(*a, x, y))) {
                return false;
            }
        }
    }
    return true;
}

// Point des capteurs sur un obstacle connu (muraille, carrière, bord) ?
bool isKnownObstacle(const Pose& robot, float lx, float ly) {
    constexpr float MARGIN_MM = 30.0f;  // erreurs de position et des capteurs
    Pose p = sensorToTable(robot, lx, ly);
    return table::nearEdge(p.x, p.y, MARGIN_MM) || table::isFixedObstacle(p.x, p.y, MARGIN_MM);
}

// Attaque : point des capteurs qu'on a le droit de toucher ? Avec les positions
// adverses de la Jetson, seulement s'il est près d'un PAMI adverse (ou sur un
// obstacle connu, qui ne compte pas) : un autre point est sans doute un de nos
// PAMI douve, devant lequel on s'arrête au lieu de le pousser. Sans positions
// adverses, on ne sait pas qui est qui : tout point compte comme cible.
KnownObstacleFilter isEnemyOrKnown(App& a) {
    return [&a](const Pose& robot, float lx, float ly) {
        if (!a.enemies.fresh(REMOTE_ENEMIES_MAX_AGE_MS)) return true;
        if (isKnownObstacle(robot, lx, ly)) return true;
        Pose p = sensorToTable(robot, lx, ly);
        float enemyD = INFINITY, allyD = INFINITY;
        for (uint8_t i = 0; i < a.enemies.count; ++i) {
            enemyD = fminf(enemyD, hypotf(p.x - a.enemies.x[i], p.y - a.enemies.y[i]));
        }
        if (a.allies.fresh(REMOTE_ENEMIES_MAX_AGE_MS)) {
            for (uint8_t i = 0; i < a.allies.count; ++i) {
                allyD = fminf(allyD, hypotf(p.x - a.allies.x[i], p.y - a.allies.y[i]));
            }
        }
        // Plus près d'un coéquipier que d'un adversaire : c'est le coéquipier
        return enemyD <= ENEMY_MATCH_RADIUS_MM && enemyD < allyD;
    };
}

// Options de déplacement : évitement + contournement si bloqué.
GoTo::Options travelOptions(const App& a) {
    GoTo::Options o;
    o.timeoutMs = GOTO_TIMEOUT_MS;
    o.detourAfterMs = DETOUR_AFTER_MS;
    o.dodgeOncoming = true;
    o.smooth = true;
    o.smoothMaxRad = TRAVEL_SMOOTH_MAX_RAD;
    o.replanSmoothMaxRad = TRAVEL_REPLAN_SMOOTH_MAX_RAD;
    o.turnRadiusMm = a.role == Role::Attack ? ATTACK_TURN_RADIUS_MM : TURN_RADIUS_MM;
    o.isPoseFree = [&a](const Pose& p) { return isPoseFree(p, &a); };
    o.isKnownObstacle = isKnownObstacle;
    o.follow = true;
    o.bypass = true;
    o.detectStall = true;
    o.trackMotion = true;
    o.halfWidthMm = PAMI_WIDTH_MM / 2;
    o.sensorOffsetMm = SENSOR_OFFSET_X_MM;
    return o;
}

// Sortie d'écurie tout droit (3 s au plus, arrêt si obstacle), puis points
// de passage (avec évitement).
void addExitAndRoute(App& a, Strategy& s) {
    const PamiProfile& p = *a.profile;
    if (p.exitMm > 0.0f) {
        Forward* exit = new Forward(a.drive, a.sensorsOrNull(), p.exitMm, p.exitMs);
        exit->finishOnKnownObstacle(isKnownObstacle);  // muraille : ne pas l'attendre
        s.add(exit);
    }
    for (uint8_t i = 0; i < p.routeLength; ++i) {
        const Waypoint& w = p.route[i];
        GoTo::Options o = travelOptions(a);
        if (w.zone == Zone::HOME) {  // faire la queue derrière les coéquipiers
            o.detourAfterMs = 0;
            o.dodgeOncoming = false;
            o.smooth = false;  // couloir serré : pivoter sur place (ne déborde pas)
        }
        o.keepRightMm = w.zone == Zone::OPEN ? KEEP_RIGHT_MM : 0.0f;
        // Esquive en S : seulement en terrain ouvert (dans un passage étroit,
        // pas la place ; et un coéquipier qui y entre de biais la déclenche à tort)
        if (w.zone != Zone::OPEN) o.dodgeOncoming = false;
        // Passage étroit : pas la place de se décaler, contourner = reculer
        // pour laisser passer ; seulement si le blocage dure.
        if (w.zone == Zone::PASSAGE) o.detourAfterMs = PASSAGE_DETOUR_AFTER_MS;
        o.passThroughMm = PASS_THROUGH_MM;  // ne pas s'arrêter au point de passage
        Pose point = {w.x, w.y, 0.0f};
        s.add(new GoTo(a.drive, a.sensorsOrNull(), table::forTeam(point, a.team), o));
    }
}

// ------------------------------------------------------------------
// Recherche de chemin (PAMI avec Wi-Fi 6E)
// ------------------------------------------------------------------
GoTo::Options huntOptions(App& a);
void addAttackHold(App& a, Strategy& s);

// Le PAMI a le module Wi-Fi 6E : il peut connaître la position des autres
// robots (Jetson) et calculer un vrai chemin.
bool usesPathPlanning(const App& a) {
    return USE_PATH_PLANNING && a.pathPlanning && comTransportFor(a.role) != ComTransport::None;
}

Pose axleOf(const Pose& center) {
    return {center.x - CENTER_AHEAD_OF_AXLE_MM * cosf(center.theta),
            center.y - CENTER_AHEAD_OF_AXLE_MM * sinf(center.theta), center.theta};
}

// Carte des obstacles fixes pour le centre du PAMI (une seule fois).
void initPlanner(App& a) {
    if (a.planner.ready()) return;
    a.planner.init([](float x, float y) {
        return !table::nearEdge(x, y, PLANNER_STATIC_CLEARANCE_MM) &&
               !table::isFixedObstacle(x, y, PLANNER_STATIC_CLEARANCE_MM);
    });
    a.planner.setHorizon(PLANNER_HORIZON_S);
    a.planner.setSoftMargin(PLANNER_SOFT_MARGIN_MM);
}

// Robots à éviter : tous ceux que voit la Jetson (adversaires et coéquipiers),
// avec leur vitesse, sauf celui qui est le but (attaque).
void addRobotObstacles(const App& a, GridPlanner& planner, const Pose& goal) {
    for (const RemoteEnemies* r : {&a.enemies, &a.allies}) {
        if (!r->fresh(REMOTE_ENEMIES_MAX_AGE_MS)) continue;
        for (uint8_t i = 0; i < r->count; ++i) {
            if (hypotf(r->x[i] - goal.x, r->y[i] - goal.y) <= ENEMY_CORNER_FROM_CENTER_MM + 60.0f) {
                continue;  // la cible elle-même
            }
            planner.addObstacle({r->x[i], r->y[i], r->vx[i], r->vy[i], PLANNER_ROBOT_RADIUS_MM});
        }
    }
}

FollowPath::Options pathOptions(App& a) {
    FollowPath::Options o;
    o.centerAheadMm = CENTER_AHEAD_OF_AXLE_MM;
    o.sensorAheadMm = SENSOR_OFFSET_X_MM;
    o.stopWithinMm = 30.0f;
    o.turnRadiusMm = TURN_RADIUS_MM;
    o.isPoseFree = [&a](const Pose& p) { return isPoseFree(p, &a); };
    o.obstacles = [&a](GridPlanner& planner, const Pose& goal) {
        addRobotObstacles(a, planner, goal);
    };
    o.onPath = [&a](const GridPlanner::Point* pts, int count) {
        a.lastPath.count = (uint8_t)count;
        for (int i = 0; i < count; ++i) {
            a.lastPath.x[i] = pts[i].x;
            a.lastPath.y[i] = pts[i].y;
        }
        a.lastPath.version++;
    };
    return o;
}

void addDouve(App& a, Strategy& s) {
    uint8_t idx = a.profile->targetIndex % table::ENEMY_DOUVE_COUNT;
    addExitAndRoute(a, s);
    GoTo::Options o = travelOptions(a);
    o.ignoreObstacleWithinMm = DOUVE_IGNORE_OBSTACLE_WITHIN_MM;
    o.stopWithinMm = CENTER_AHEAD_OF_AXLE_MM;  // le centre du PAMI sur la douve
    s.add(new GoTo(a.drive, a.sensorsOrNull(),
                   table::forTeam(table::ENEMY_DOUVES[idx], a.team), o));
}

void addAttack(App& a, Strategy& s) {
    uint8_t idx = a.profile->targetIndex % table::ATTACK_ZONE_COUNT;
    Pose zone = table::forTeam(table::ATTACK_ZONES[idx], a.team);

    // 1) Aller à sa zone, tourné vers l'adversaire. On ne chasse pas en route :
    //    on prendrait pour cible le coéquipier sorti juste avant de la même
    //    écurie.
    // 2) Attendre qu'un PAMI adverse passe à portée, puis le rejoindre jusqu'à
    //    ATTACK_APPROACH_GAP_MM de sa surface. L'obstacle, c'est la cible : on
    //    ne s'arrête que devant ce qui n'est pas un adversaire (un coéquipier,
    //    si la Jetson donne les positions).
    addExitAndRoute(a, s);
    if (usesPathPlanning(a)) {
        // Wi-Fi 6E : vrai chemin, recalculé avec la position prévue de tous
        // les robots vus par la Jetson
        initPlanner(a);
        FollowPath::Options toZone = pathOptions(a);
        toZone.finalHeading = true;
        toZone.isIgnored = isKnownObstacle;
        toZone.timeoutMs = GOTO_TIMEOUT_MS;
        toZone.turnRadiusMm = ATTACK_TURN_RADIUS_MM;
        s.add(new FollowPath(a.drive, a.sensorsOrNull(), a.planner, zone, toZone));

        if (!PATH_PLANNING_HUNT) {  // cible proche et qui bouge : poursuite directe
            s.add(new GoTo(a.drive, a.sensorsOrNull(), zone, huntOptions(a)));
            addAttackHold(a, s);
            return;
        }
        FollowPath::Options hunt = pathOptions(a);
        hunt.turnRadiusMm = 0.0f;  // cible proche : pivot précis (meilleur en simulation)
        hunt.goalProvider = [&a](const Pose& center, Pose& goal) {
            return nearestEnemyCandidate(a)(axleOf(center), goal);
        };
        hunt.waitForGoal = true;
        hunt.faceGoal = true;
        hunt.stopWithinMm = FRONT_AHEAD_OF_AXLE_MM - CENTER_AHEAD_OF_AXLE_MM + ATTACK_APPROACH_GAP_MM;
        hunt.isIgnored = isEnemyOrKnown(a);
        s.add(new FollowPath(a.drive, a.sensorsOrNull(), a.planner, zone, hunt));
    } else {
        GoTo::Options toZone = travelOptions(a);
        toZone.finalHeading = true;
        toZone.stopWithinMm = CENTER_AHEAD_OF_AXLE_MM;
        s.add(new GoTo(a.drive, a.sensorsOrNull(), zone, toZone));
        s.add(new GoTo(a.drive, a.sensorsOrNull(), zone, huntOptions(a)));
    }
    addAttackHold(a, s);
}

GoTo::Options huntOptions(App& a) {
    GoTo::Options hunt;
    hunt.timeoutMs = 0;  // jusqu'à la fin du match
    hunt.isKnownObstacle = isEnemyOrKnown(a);
    hunt.stopWithinMm = ATTACK_STOP_WITHIN_MM;
    hunt.faceTarget = true;
    hunt.smooth = true;
    hunt.smoothMaxRad = HUNT_SMOOTH_MAX_RAD;
    hunt.replanSmoothMaxRad = HUNT_SMOOTH_MAX_RAD;
    hunt.turnRadiusMm = 0.0f;  // cible proche : pivot précis (meilleur en simulation)
    hunt.isPoseFree = [&a](const Pose& p) { return isPoseFree(p, &a); };
    hunt.provider = nearestEnemyCandidate(a);
    hunt.waitForTarget = true;
    hunt.retargetPeriodMs = 200;
    hunt.retargetMinMm = 20.0f;
    return hunt;
}

void addAttackHold(App& a, Strategy& s) {
    // 3) Déployer l'actionneur, puis se tenir à ATTACK_HOLD_GAP_MM de la face
    //    adverse jusqu'à la fin du match : l'actionneur touche, la carrosserie
    //    non (règlement). Si l'adversaire bouge, on le suit.
    s.add(new ServoMove(a.servo, ATTACK_SERVO_DEPLOY_DEG));  // tenu jusqu'à la fin
    KeepDistance::Options hold;
    hold.provider = nearestEnemyCandidate(a);
    hold.frontAheadMm = FRONT_AHEAD_OF_AXLE_MM;
    hold.sensorAheadMm = SENSOR_OFFSET_X_MM;
    hold.gapMm = ATTACK_HOLD_GAP_MM;
    hold.toleranceMm = ATTACK_HOLD_TOLERANCE_MM;
    hold.maxHeadingRad = ATTACK_HOLD_MAX_HEADING_RAD;
    hold.isIgnored = isEnemyOrKnown(a);
    hold.isPoseFree = [&a](const Pose& p) { return isPoseFree(p, &a); };
    s.add(new KeepDistance(a.drive, a.sensorsOrNull(), hold));
}

}  // namespace

void buildMatchStrategy(App& a) {
    Strategy& s = a.strategy;
    s.clear();
    s.add(new WaitMatchTime(a.clock, ATTACK_PHASE_START_MS + ATTACK_START_MARGIN_MS +
                                         a.profile->startDelayMs));

    switch (a.role) {
        case Role::Douve:
            addDouve(a, s);
            break;
        case Role::Attack:
            addAttack(a, s);
            break;
        case Role::Test:
            for (int i = 0; i < 4; ++i) {
                s.add(new Forward(a.drive, a.sensorsOrNull(), 300.0f));
                s.add(new Turn(a.drive, (float)M_PI_2));
            }
            break;
    }
    s.start();
}
