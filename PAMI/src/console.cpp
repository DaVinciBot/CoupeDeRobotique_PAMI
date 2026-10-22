// Console série (USB) pour tester et calibrer sans reflasher.
// Ouvrir avec : pio device monitor   puis taper "help".
#include "Actions.h"
#include "app.h"

namespace {

char s_line[96];
size_t s_len = 0;

void printHelp() {
    Serial.println(
        "Commandes :\n"
        "  f <mm>          avancer (negatif = reculer)\n"
        "  t <deg>         tourner (positif = gauche)\n"
        "  g <x> <y>       aller au point (repere table)\n"
        "  sq <mm>         carre de test (4 x avancer + 90 deg)\n"
        "  stop            arreter le mouvement\n"
        "  pose            afficher la pose\n"
        "  setpose <x> <y> <deg>\n"
        "  speed <mm/s>    accel <mm/s2>    base <mm> (entraxe)\n"
        "  servo <deg>     color\n"
        "  lidar [pts]     etat du lidar (pts : tous les points x,y)\n"
        "  zone <prof> <larg>  taille du couloir surveille (mm)\n"
        "  start [ms]      simuler le depart (ex: start 84000)\n"
        "  end             simuler la fin du match\n"
        "  reset           revenir en attente de depart\n");
}

bool canMoveManually(App& a) {
    if (a.state == MatchState::Running) {
        Serial.println("refuse : match en cours (\"end\" pour arreter)");
        return false;
    }
    return true;
}

void runManual(App& a, Action* action) {
    a.strategy.clear();
    a.strategy.add(action);
    a.strategy.start();
    a.state = MatchState::Manual;
}

// "lidar"     : état, zone surveillée, objet le plus proche
// "lidar pts" : tous les points (indice, distance brute, x, y)
void printLidar(App& a, bool allPoints) {
    if (!a.lidarEnabled) {
        Serial.println("lidar desactive (ENABLE_LIDAR)");
        return;
    }
    LidarGS2::Stats st = a.lidar.stats();
    LidarGS2::Calibration cal = a.lidar.calibration();
    Serial.printf("trames=%lu checksum_faux=%lu age=%lu ms calibration=%s\n",
                  (unsigned long)st.frames, (unsigned long)st.badChecksum,
                  (unsigned long)st.lastFrameAgeMs, cal.valid ? "ok" : "ABSENTE");
    Serial.printf("couloir %u x %u mm : min=%u mm, obstacle=%d\n",
                  a.lidar.obstacleThreshold(), 2 * a.lidar.corridorHalfWidth(),
                  a.lidar.frontMinMm(), a.lidar.obstacle());
    float nx, ny;
    if (a.lidar.nearest(nx, ny)) {
        Serial.printf("plus proche : x=%.0f y=%.0f mm (%s)\n", nx, ny,
                      ny > 0 ? "gauche" : "droite");
    } else {
        Serial.println("plus proche : rien");
    }
    if (!allPoints) return;

    uint16_t d[LidarGS2::POINT_COUNT];
    LidarPoint p[LidarGS2::POINT_COUNT];
    a.lidar.copyDistances(d);
    a.lidar.copyPoints(p);
    for (uint16_t i = 0; i < LidarGS2::POINT_COUNT; ++i) {
        if (p[i].valid) {
            Serial.printf("%3u: %3u mm -> (%4.0f, %4.0f)\n", i, d[i], p[i].x, p[i].y);
        } else if (d[i] > 0) {
            Serial.printf("%3u: %3u mm\n", i, d[i]);
        }
    }
}

void execute(App& a, char* line) {
    char* cmd = strtok(line, " \t");
    if (cmd == nullptr) return;
    char* a1 = strtok(nullptr, " \t");
    char* a2 = strtok(nullptr, " \t");
    char* a3 = strtok(nullptr, " \t");

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        printHelp();
    } else if (!strcmp(cmd, "f") && a1) {
        if (canMoveManually(a)) runManual(a, new Forward(a.drive, a.lidarOrNull(), atof(a1)));
    } else if (!strcmp(cmd, "t") && a1) {
        if (canMoveManually(a)) runManual(a, new Turn(a.drive, atof(a1) * DEG_TO_RAD));
    } else if (!strcmp(cmd, "g") && a1 && a2) {
        if (canMoveManually(a)) {
            Pose target = {(float)atof(a1), (float)atof(a2), 0.0f};
            runManual(a, new GoTo(a.drive, a.lidarOrNull(), target));
        }
    } else if (!strcmp(cmd, "sq")) {
        if (canMoveManually(a)) {
            float side = a1 ? atof(a1) : 300.0f;
            a.strategy.clear();
            for (int i = 0; i < 4; ++i) {
                a.strategy.add(new Forward(a.drive, a.lidarOrNull(), side));
                a.strategy.add(new Turn(a.drive, (float)M_PI_2));
            }
            a.strategy.start();
            a.state = MatchState::Manual;
        }
    } else if (!strcmp(cmd, "stop")) {
        a.strategy.stop();
        a.drive.stop();
        if (a.state == MatchState::Manual) a.state = MatchState::WaitStart;
        Serial.println("ok");
    } else if (!strcmp(cmd, "pose")) {
        Pose p = a.drive.pose();
        Serial.printf("pose (%.1f, %.1f, %.1f deg)\n", p.x, p.y, p.theta * RAD_TO_DEG);
    } else if (!strcmp(cmd, "setpose") && a1 && a2 && a3) {
        a.drive.setPose({(float)atof(a1), (float)atof(a2), (float)(atof(a3) * DEG_TO_RAD)});
        Serial.println("ok");
    } else if (!strcmp(cmd, "speed") && a1) {
        a.drive.setSpeed(atof(a1));
        Serial.printf("vitesse %.0f mm/s\n", a.drive.speed());
    } else if (!strcmp(cmd, "accel") && a1) {
        a.drive.setAccel(atof(a1));
        Serial.printf("acceleration %.0f mm/s2\n", a.drive.accel());
    } else if (!strcmp(cmd, "base") && a1) {
        a.drive.setWheelBase(atof(a1));
        Serial.printf("entraxe %.1f mm (a reporter dans config.h)\n", a.drive.wheelBase());
    } else if (!strcmp(cmd, "servo") && a1) {
        a.servo.write((uint8_t)constrain(atoi(a1), 0, 180));
        Serial.println("ok");
    } else if (!strcmp(cmd, "lidar")) {
        printLidar(a, a1 && !strcmp(a1, "pts"));
    } else if (!strcmp(cmd, "zone") && a1 && a2) {
        a.lidar.setObstacleThreshold((uint16_t)atoi(a1));
        a.lidar.setCorridorHalfWidth((uint16_t)(atoi(a2) / 2));
        Serial.printf("couloir %u x %u mm (a reporter dans config.h)\n",
                      a.lidar.obstacleThreshold(), 2 * a.lidar.corridorHalfWidth());
    } else if (!strcmp(cmd, "color")) {
        Serial.printf("couleur %s\n", teamName(readTeam()));
    } else if (!strcmp(cmd, "start")) {
        startMatch(a, a1 ? strtoul(a1, nullptr, 10) : 0, "console");
    } else if (!strcmp(cmd, "end")) {
        endMatch(a, "console");
    } else if (!strcmp(cmd, "reset")) {
        a.strategy.stop();
        a.drive.stop();
        a.clock.reset();
        a.state = MatchState::WaitStart;
        applyRoleIdlePosition(a);
        Serial.println("ok, en attente de depart");
    } else {
        Serial.printf("commande inconnue : %s (help)\n", cmd);
    }
}

}  // namespace

void consoleUpdate(App& a) {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            s_line[s_len] = '\0';
            if (s_len > 0) execute(a, s_line);
            s_len = 0;
        } else if (s_len < sizeof(s_line) - 1) {
            s_line[s_len++] = c;
        }
    }
}
