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

// Cible de l'attaque :
//  1) si la Jetson envoie les positions adverses (Wi-Fi) : le PAMI adverse le
//     plus proche, à portée ; on sait alors qui est adversaire, on peut
//     chasser tout de suite ;
//  2) sinon, à partir de ATTACK_HUNT_FROM_MS : le point le plus proche vu par
//     les capteurs qui n'est ni un bord ni un obstacle connu (avant, ce serait
//     souvent un de nos PAMI douve : les capteurs ne distingue pas les équipes).
GoTo::TargetProvider nearestEnemyCandidate(App& a) {
    return [&a](const Pose& robot, Pose& out) {
        if (a.enemies.fresh(REMOTE_ENEMIES_MAX_AGE_MS)) {
            float best = ATTACK_REMOTE_RANGE_MM;
            bool found = false;
            for (uint8_t i = 0; i < a.enemies.count; ++i) {
                float dx = a.enemies.x[i] - robot.x, dy = a.enemies.y[i] - robot.y;
                float d = hypotf(dx, dy);
                if (d < best && d > 1.0f) {
                    best = d;
                    // viser sa face, pas son centre (stopWithin compte depuis la surface)
                    float back = (PAMI_LENGTH_MM / 2) / d;
                    out = {a.enemies.x[i] - dx * back, a.enemies.y[i] - dy * back, 0.0f};
                    found = true;
                }
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
            if (table::nearEdge(p.x, p.y, EDGE_MARGIN_MM) ||
                table::isFixedObstacle(p.x, p.y, ATTACK_FIXED_OBSTACLE_MARGIN_MM)) {
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

// Le PAMI tient-il à cette pose sans toucher muraille, carrière ou bord ?
// On teste son contour (coins, milieux des côtés) et son centre.
bool isPoseFree(const Pose& axle) {
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
            if (table::nearEdge(x, y, MARGIN_MM) || table::isFixedObstacle(x, y, MARGIN_MM)) {
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

// Options de déplacement : évitement + contournement si bloqué.
GoTo::Options travelOptions() {
    GoTo::Options o;
    o.timeoutMs = GOTO_TIMEOUT_MS;
    o.detourAfterMs = DETOUR_AFTER_MS;
    o.dodgeOncoming = true;
    o.smooth = true;
    o.isPoseFree = isPoseFree;
    o.isKnownObstacle = isKnownObstacle;
    o.follow = true;
    o.bypass = true;
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
        GoTo::Options o = travelOptions();
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

void addDouve(App& a, Strategy& s) {
    uint8_t idx = a.profile->targetIndex % table::ENEMY_DOUVE_COUNT;
    addExitAndRoute(a, s);
    GoTo::Options o = travelOptions();
    o.ignoreObstacleWithinMm = DOUVE_IGNORE_OBSTACLE_WITHIN_MM;
    o.stopWithinMm = CENTER_AHEAD_OF_AXLE_MM;  // le centre du PAMI sur la douve
    s.add(new GoTo(a.drive, a.sensorsOrNull(),
                   table::forTeam(table::ENEMY_DOUVES[idx], a.team), o));
}

void addAttack(App& a, Strategy& s) {
    uint8_t idx = a.profile->targetIndex % table::ATTACK_ZONE_COUNT;
    Pose zone = table::forTeam(table::ATTACK_ZONES[idx], a.team);

    // 1) Aller à sa zone, tourné vers l'adversaire, avec l'évitement normal.
    //    On ne chasse pas en route : on prendrait pour cible le coéquipier
    //    sorti juste avant de la même écurie.
    addExitAndRoute(a, s);
    GoTo::Options toZone = travelOptions();
    toZone.finalHeading = true;
    toZone.stopWithinMm = CENTER_AHEAD_OF_AXLE_MM;
    s.add(new GoTo(a.drive, a.sensorsOrNull(), zone, toZone));

    // 3) Attendre qu'un PAMI adverse passe à portée des capteurs (~30 cm), puis
    //    le suivre en corrigeant la direction en roulant, jusqu'à
    //    ATTACK_STOP_WITHIN_MM de lui.
    GoTo::Options hunt;
    hunt.timeoutMs = 0;  // jusqu'à la fin du match
    hunt.pauseOnObstacle = false;  // l'obstacle, c'est la cible
    hunt.stopWithinMm = ATTACK_STOP_WITHIN_MM;
    hunt.provider = nearestEnemyCandidate(a);
    hunt.waitForTarget = true;
    hunt.retargetPeriodMs = 200;
    hunt.retargetMinMm = 20.0f;
    s.add(new GoTo(a.drive, a.sensorsOrNull(), zone, hunt));

    // 4) Avancer pour toucher, puis déployer.
    s.add(new Forward(a.drive, nullptr, ATTACK_CREEP_MM));  // sans capteur : on veut toucher
    s.add(new ServoMove(a.servo, ATTACK_SERVO_DEPLOY_DEG));  // tenu jusqu'à la fin
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
