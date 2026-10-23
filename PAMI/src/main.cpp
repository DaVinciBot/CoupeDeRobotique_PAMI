// Firmware PAMI 2027 (ESP32-S3).
//
// Démarrage : tirette retirée. Le PAMI attend la phase d'attaque (85 s),
// exécute la stratégie de son rôle, puis s'arrête à la fin du match (100 s).
#include "app.h"

App app;
static HardwareSerial LidarSerial(1);

Team readTeam() {
    if (COLOR_OVERRIDE == 1) return Team::Yellow;
    if (COLOR_OVERRIDE == -1) return Team::Blue;
    return digitalRead(COLOR_PIN_READ) == HIGH ? Team::Yellow : Team::Blue;
}

void applyRoleIdlePosition(App& a) {
    if (a.role == Role::Attack) a.servo.write(ATTACK_SERVO_REST_DEG);
}

void startMatch(App& a, uint32_t elapsedMs, const char* source) {
    if (a.state == MatchState::Running || a.state == MatchState::Ended) return;
    if (a.state == MatchState::Manual) {
        a.strategy.stop();
        a.drive.stop();
    }

    a.clock.startAt(millis(), elapsedMs);
    a.team = readTeam();
    a.drive.setPose(table::forTeam(a.profile->startBlue, a.team));

    a.state = MatchState::Running;
    Serial.printf("=== MATCH LANCE (%s, t=%lu ms) : PAMI %u, role %s, %s ===\n",
                  source, (unsigned long)elapsedMs, PAMI_ID, roleName(a.role),
                  teamName(a.team));
    buildMatchStrategy(a);
}

void endMatch(App& a, const char* reason) {
    if (a.state != MatchState::Running && a.state != MatchState::Manual) return;
    a.strategy.stop();  // l'actionneur garde sa position
    a.drive.stop();
    a.state = a.state == MatchState::Running ? MatchState::Ended
                                             : MatchState::WaitStart;
    Serial.printf("=== ARRET : %s ===\n", reason);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    app.profile = &pamiProfile(PAMI_ID);
    app.role = app.profile->role;

    DiffDrive::Config dc;
    dc.leftStepPin = LEFT_STEP_PIN;
    dc.leftDirPin = LEFT_DIR_PIN;
    dc.leftEnPin = LEFT_EN_PIN;
    dc.leftDirHighIsForward = LEFT_DIR_HIGH_IS_FORWARD;
    dc.rightStepPin = RIGHT_STEP_PIN;
    dc.rightDirPin = RIGHT_DIR_PIN;
    dc.rightEnPin = RIGHT_EN_PIN;
    dc.rightDirHighIsForward = RIGHT_DIR_HIGH_IS_FORWARD;
    dc.stepsPerRev = MOTOR_FULL_STEPS_PER_REV * app.profile->microstepping;
    dc.wheelDiameterMm = WHEEL_DIAMETER_MM;
    dc.wheelBaseMm = WHEEL_BASE_MM;
    dc.speedMmS = DRIVE_SPEED_MM_S;
    dc.accelMmS2 = DRIVE_ACCEL_MM_S2;
    app.drive.begin(dc);

    app.servo.begin(SERVO_PIN, 0);
    applyRoleIdlePosition(app);

    pinMode(COLOR_PIN_VCC, OUTPUT);
    digitalWrite(COLOR_PIN_VCC, HIGH);
    pinMode(COLOR_PIN_READ, INPUT_PULLDOWN);
    app.tirette.begin(TIRETTE_PIN);

    if (ENABLE_LIDAR) {
        LidarGS2::Config lc;
        lc.rxPin = LIDAR_RX_PIN;
        lc.txPin = LIDAR_TX_PIN;
        lc.obstacleMm = LIDAR_OBSTACLE_MM;
        lc.corridorHalfWidthMm = LIDAR_CORRIDOR_HALF_WIDTH_MM;
        lc.yLeftPositive = LIDAR_Y_LEFT_POSITIVE;
        lc.fallbackFront[0] = {LIDAR_FALLBACK_A_FIRST, LIDAR_FALLBACK_A_LAST};
        lc.fallbackFront[1] = {LIDAR_FALLBACK_B_FIRST, LIDAR_FALLBACK_B_LAST};
        app.lidar.begin(LidarSerial, lc);
        app.lidar.startTask(0);
        app.lidarEnabled = true;
    }

    Serial.printf("\n=== PAMI %u pret : role %s, micropas 1/%u, couleur lue %s ===\n",
                  PAMI_ID, roleName(app.role), app.profile->microstepping,
                  teamName(readTeam()));
    Serial.println("Attente tirette. Tapez \"help\" pour la console.");
}

void loop() {
    app.drive.update();

    if (app.tirette.update() && app.state == MatchState::WaitStart) {
        startMatch(app, 0, "tirette");
    }

    consoleUpdate(app);

    switch (app.state) {
        case MatchState::Running:
            app.strategy.update();
            if (app.clock.elapsedMs() >= MATCH_DURATION_MS - MATCH_END_MARGIN_MS) {
                endMatch(app, "fin du match");
            }
            break;
        case MatchState::Manual:
            app.strategy.update();
            if (app.strategy.isFinished()) {
                Pose p = app.drive.pose();
                Serial.printf("ok  pose (%.1f, %.1f, %.1f deg)\n", p.x, p.y,
                              p.theta * RAD_TO_DEG);
                app.state = MatchState::WaitStart;
            }
            break;
        default:
            break;
    }
}
