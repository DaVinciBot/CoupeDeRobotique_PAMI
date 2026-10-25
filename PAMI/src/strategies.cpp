// Stratégies de match par rôle.
// Toutes commencent par attendre la phase d'attaque (85 s).
//
// Quand le module de communication sera choisi, une cible reçue en cours de
// route pourra être donnée à GoTo via Options::provider (comme le lidar pour
// l'attaque).
#include "Actions.h"
#include "app.h"

namespace {

// Point vu par le lidar -> position sur la table.
Pose lidarToTable(const Pose& robot, float lx, float ly) {
    float x = lx + LIDAR_OFFSET_X_MM;  // repère robot (centre des roues)
    float c = cosf(robot.theta), s = sinf(robot.theta);
    return {robot.x + c * x - s * ly, robot.y + s * x + c * ly, 0.0f};
}

// Cible = point le plus proche vu par le lidar qui n'est ni un bord de table,
// ni une tour, ni une carrière : en principe un PAMI adverse.
GoTo::TargetProvider nearestEnemyCandidate(App& a) {
    return [&a](const Pose& robot, Pose& out) {
        if (!a.lidarEnabled || !a.lidar.healthy()) return false;
        constexpr float EDGE_MARGIN_MM = 40.0f;

        LidarPoint pts[LidarGS2::POINT_COUNT];
        a.lidar.copyPoints(pts);
        bool found = false;
        float bestD2 = INFINITY;
        for (const LidarPoint& lp : pts) {
            if (!lp.valid || !lp.confirmed) continue;
            Pose p = lidarToTable(robot, lp.x, lp.y);
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

void addDouve(App& a, Strategy& s) {
    uint8_t idx = a.profile->targetIndex % table::ENEMY_DOUVE_COUNT;
    GoTo::Options o;
    o.timeoutMs = GOTO_TIMEOUT_MS;
    o.ignoreObstacleWithinMm = DOUVE_IGNORE_OBSTACLE_WITHIN_MM;
    s.add(new GoTo(a.drive, a.lidarOrNull(),
                   table::forTeam(table::ENEMY_DOUVES[idx], a.team), o));
}

void addAttack(App& a, Strategy& s) {
    uint8_t idx = a.profile->targetIndex % table::ATTACK_ZONE_COUNT;
    Pose zone = table::forTeam(table::ATTACK_ZONES[idx], a.team);

    // 1) Aller à sa zone, tourné vers l'adversaire, avec l'évitement normal.
    //    On ne chasse pas en route : on prendrait pour cible le coéquipier
    //    sorti juste avant de la même écurie.
    GoTo::Options toZone;
    toZone.timeoutMs = GOTO_TIMEOUT_MS;
    toZone.finalHeading = true;
    s.add(new GoTo(a.drive, a.lidarOrNull(), zone, toZone));

    // 2) Attendre qu'un PAMI adverse passe à portée du lidar (~30 cm), puis
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
    s.add(new GoTo(a.drive, a.lidarOrNull(), zone, hunt));

    // 3) Avancer pour toucher, puis déployer.
    s.add(new Forward(a.drive, nullptr, ATTACK_CREEP_MM));  // sans lidar : on veut toucher
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
                s.add(new Forward(a.drive, a.lidarOrNull(), 300.0f));
                s.add(new Turn(a.drive, (float)M_PI_2));
            }
            break;
    }
    s.start();
}
