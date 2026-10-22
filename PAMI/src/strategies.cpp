// Stratégies de match par rôle.
// Toutes commencent par attendre la phase d'attaque (85 s).
//
// Quand le module de communication sera choisi, une cible reçue en cours de
// route pourra être donnée à GoTo via Options::provider.
#include "Actions.h"
#include "app.h"

namespace {

// Point vu par le lidar -> position sur la table.
Pose lidarToTable(const Pose& robot, float lx, float ly) {
    float x = lx + LIDAR_OFFSET_X_MM;  // repère robot (centre des roues)
    float c = cosf(robot.theta), s = sinf(robot.theta);
    return {robot.x + c * x - s * ly, robot.y + s * x + c * ly, 0.0f};
}

// Cible = objet le plus proche vu par le lidar, s'il n'est pas une bordure.
GoTo::TargetProvider nearestSeenObject(App& a) {
    return [&a](const Pose& robot, Pose& out) {
        float lx, ly;
        if (!a.lidarEnabled || !a.lidar.nearest(lx, ly)) return false;
        Pose p = lidarToTable(robot, lx, ly);
        constexpr float EDGE_MARGIN_MM = 40.0f;
        if (p.x < EDGE_MARGIN_MM || p.x > table::LENGTH_MM - EDGE_MARGIN_MM ||
            p.y < EDGE_MARGIN_MM || p.y > table::WIDTH_MM - EDGE_MARGIN_MM) {
            return false;
        }
        out = p;
        return true;
    };
}

void addDouve(App& a, Strategy& s) {
    uint8_t idx = a.profile->douveIndex % table::ENEMY_DOUVE_COUNT;
    GoTo::Options o;
    o.timeoutMs = GOTO_TIMEOUT_MS;
    o.ignoreObstacleWithinMm = DOUVE_IGNORE_OBSTACLE_WITHIN_MM;
    s.add(new GoTo(a.drive, a.lidarOrNull(),
                   table::forTeam(table::ENEMY_DOUVES[idx], a.team), o));
}

void addAttack(App& a, Strategy& s) {
    // On part vers la zone d'attaque ; dès que le lidar voit un objet (portée
    // ~30 cm, tout le champ), on se dirige vers lui. Au contact (objet dans le
    // couloir), on avance encore pour toucher et on déploie.
    GoTo::Options o;
    o.timeoutMs = GOTO_TIMEOUT_MS;
    o.pauseOnObstacle = false;
    o.finishOnObstacle = true;
    o.provider = nearestSeenObject(a);
    o.retargetPeriodMs = 300;
    s.add(new GoTo(a.drive, a.lidarOrNull(),
                   table::forTeam(table::ATTACK_DEFAULT_TARGET, a.team), o));
    s.add(new Forward(a.drive, nullptr, ATTACK_CREEP_MM));  // sans lidar : on veut toucher
    s.add(new ServoMove(a.servo, ATTACK_SERVO_DEPLOY_DEG));  // tenu jusqu'à la fin
}

void addBoulet(App& a, Strategy& s) {
    GoTo::Options o;
    o.timeoutMs = GOTO_TIMEOUT_MS;
    o.finalHeading = true;
    s.add(new GoTo(a.drive, a.lidarOrNull(),
                   table::forTeam(table::BOULET_LAUNCH_POINT, a.team), o));
    s.add(new ServoMove(a.servo, BOULET_SERVO_RELEASE_DEG, 800));
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
        case Role::Boulet:
            addBoulet(a, s);
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
