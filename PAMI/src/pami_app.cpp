// Démarrage et déroulé du match, sans dépendre de setup()/loop() :
// utilisé par main.cpp sur le robot et par le simulateur (sim/).
#include "app.h"

Team readTeam() {
    if (COLOR_OVERRIDE == 1) return Team::Yellow;
    if (COLOR_OVERRIDE == -1) return Team::Blue;
    return digitalRead(COLOR_PIN_READ) == HIGH ? Team::Yellow : Team::Blue;
}

void applyRoleIdlePosition(App& a) {
    if (a.role == Role::Attack) a.servo.write(ATTACK_SERVO_REST_DEG);
}

void appBegin(App& a, uint8_t pamiId, HardwareSerial& lidarSerial) {
    a.profile = &pamiProfile(pamiId);
    a.role = a.profile->role;

    DiffDrive::Config dc;
    dc.leftStepPin = LEFT_STEP_PIN;
    dc.leftDirPin = LEFT_DIR_PIN;
    dc.leftEnPin = LEFT_EN_PIN;
    dc.leftDirHighIsForward = LEFT_DIR_HIGH_IS_FORWARD;
    dc.rightStepPin = RIGHT_STEP_PIN;
    dc.rightDirPin = RIGHT_DIR_PIN;
    dc.rightEnPin = RIGHT_EN_PIN;
    dc.rightDirHighIsForward = RIGHT_DIR_HIGH_IS_FORWARD;
    dc.stepsPerRev = MOTOR_FULL_STEPS_PER_REV * a.profile->microstepping;
    dc.wheelDiameterMm = WHEEL_DIAMETER_MM;
    dc.wheelBaseMm = WHEEL_BASE_MM;
    dc.speedMmS = DRIVE_SPEED_MM_S;
    dc.accelMmS2 = DRIVE_ACCEL_MM_S2;
    a.drive.begin(dc);

    a.servo.begin(SERVO_PIN, 0);
    applyRoleIdlePosition(a);

    pinMode(COLOR_PIN_VCC, OUTPUT);
    digitalWrite(COLOR_PIN_VCC, HIGH);
    pinMode(COLOR_PIN_READ, INPUT_PULLDOWN);
    a.tirette.begin(TIRETTE_PIN);

    if (ENABLE_LIDAR) {
        LidarGS2::Config lc;
        lc.rxPin = LIDAR_RX_PIN;
        lc.txPin = LIDAR_TX_PIN;
        lc.obstacleMm = LIDAR_OBSTACLE_MM;
        lc.corridorHalfWidthMm = LIDAR_CORRIDOR_HALF_WIDTH_MM;
        lc.yLeftPositive = LIDAR_Y_LEFT_POSITIVE;
        lc.fallbackFront[0] = {LIDAR_FALLBACK_A_FIRST, LIDAR_FALLBACK_A_LAST};
        lc.fallbackFront[1] = {LIDAR_FALLBACK_B_FIRST, LIDAR_FALLBACK_B_LAST};
        a.lidar.begin(lidarSerial, lc);
        a.lidar.startTask(0);
        a.lidarEnabled = true;
    }

    Serial.printf("=== PAMI %u pret : role %s, micropas 1/%u, couleur lue %s ===\n",
                  a.profile->id, roleName(a.role), a.profile->microstepping,
                  teamName(readTeam()));
}

void startMatch(App& a, uint32_t elapsedMs, const char* source) {
    if (a.state == MatchState::Running || a.state == MatchState::Ended) return;
    if (a.state == MatchState::Manual) {
        a.strategy.stop();
        a.drive.stop();
    }

    a.clock.startAt(millis(), elapsedMs);
    a.team = a.remoteTeamSet ? a.remoteTeam : readTeam();
    a.drive.setPose(table::forTeam(a.profile->startBlue, a.team));

    a.state = MatchState::Running;
    Serial.printf("=== MATCH LANCE (%s, t=%lu ms) : PAMI %u, role %s, %s ===\n",
                  source, (unsigned long)elapsedMs, a.profile->id, roleName(a.role),
                  teamName(a.team));
    buildMatchStrategy(a);
}

void endMatch(App& a, const char* reason) {
    if (a.state != MatchState::Running && a.state != MatchState::Manual) return;
    a.strategy.stop();
    a.drive.stop();      // moteurs coupés dès l'arrêt (voir matchUpdate)
    a.servo.release();   // actionneur coupé (règlement H.3) : le bras reste où il est
    a.state = a.state == MatchState::Running ? MatchState::Ended
                                             : MatchState::WaitStart;
    Serial.printf("=== ARRET : %s ===\n", reason);
}

void matchUpdate(App& a) {
    switch (a.state) {
        case MatchState::Running:
            a.strategy.update();
            if (a.clock.elapsedMs() >= MATCH_DURATION_MS - MATCH_END_MARGIN_MS) {
                endMatch(a, "fin du match");
            }
            break;
        case MatchState::Ended:
            // Règlement : actionneurs coupés à la fin -> moteurs non alimentés
            // dès que le PAMI est arrêté
            if (a.drive.powered() && !a.drive.isMoving()) a.drive.enableOutputs(false);
            break;
        case MatchState::Manual:
            a.strategy.update();
            if (a.strategy.isFinished()) {
                Pose p = a.drive.pose();
                Serial.printf("ok  pose (%.1f, %.1f, %.1f deg)\n", p.x, p.y,
                              p.theta * RAD_TO_DEG);
                a.state = MatchState::WaitStart;
            }
            break;
        default:
            break;
    }
}
