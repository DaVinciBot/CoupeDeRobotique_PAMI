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
        "  tof             capteurs ToF : distances, points, couloir\n"
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

// "tof" : chaque capteur (distance, point), couloir surveillé, objet le plus proche
void printSensors(App& a) {
    if (!a.sensorsEnabled) {
        Serial.println("capteurs ToF desactives ou absents (ENABLE_TOF, cablage)");
        return;
    }
    const TofArray::Config& cfg = a.tof.config();
    ObstaclePoint p[ObstacleSensor::MAX_POINTS] = {};
    a.tof.copyPoints(p);
    for (uint8_t i = 0; i < cfg.count; ++i) {
        const TofArray::Mount& m = cfg.sensors[i];
        if (!a.tof.sensorAlive(i)) {
            Serial.printf("ToF %u (%+4.0f deg) : ne repond pas\n", i, m.angleDeg);
        } else if (!p[i].valid) {
            Serial.printf("ToF %u (%+4.0f deg) : rien (%u mm)\n", i, m.angleDeg,
                          a.tof.distanceMm(i));
        } else {
            Serial.printf("ToF %u (%+4.0f deg) : %4u mm -> (%4.0f, %4.0f)\n", i, m.angleDeg,
                          a.tof.distanceMm(i), p[i].x, p[i].y);
        }
    }
    Serial.printf("trames=%lu  couloir %u x %u mm : min=%u mm, obstacle=%d\n",
                  (unsigned long)a.tof.frames(), a.tof.obstacleThreshold(),
                  2 * a.tof.corridorHalfWidth(), a.tof.frontMinMm(), a.tof.obstacle());
    float nx, ny;
    if (a.tof.nearest(nx, ny)) {
        Serial.printf("plus proche : x=%.0f y=%.0f mm (%s)\n", nx, ny,
                      ny > 0 ? "gauche" : "droite");
    } else {
        Serial.println("plus proche : rien");
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
        if (canMoveManually(a)) runManual(a, new Forward(a.drive, a.sensorsOrNull(), atof(a1)));
    } else if (!strcmp(cmd, "t") && a1) {
        if (canMoveManually(a)) runManual(a, new Turn(a.drive, atof(a1) * DEG_TO_RAD));
    } else if (!strcmp(cmd, "g") && a1 && a2) {
        if (canMoveManually(a)) {
            Pose target = {(float)atof(a1), (float)atof(a2), 0.0f};
            runManual(a, new GoTo(a.drive, a.sensorsOrNull(), target));
        }
    } else if (!strcmp(cmd, "sq")) {
        if (canMoveManually(a)) {
            float side = a1 ? atof(a1) : 300.0f;
            a.strategy.clear();
            for (int i = 0; i < 4; ++i) {
                a.strategy.add(new Forward(a.drive, a.sensorsOrNull(), side));
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
    } else if (!strcmp(cmd, "tof")) {
        printSensors(a);
    } else if (!strcmp(cmd, "zone") && a1 && a2) {
        a.tof.setObstacleThreshold((uint16_t)atoi(a1));
        a.tof.setCorridorHalfWidth((uint16_t)(atoi(a2) / 2));
        Serial.printf("couloir %u x %u mm (a reporter dans config.h)\n",
                      a.tof.obstacleThreshold(), 2 * a.tof.corridorHalfWidth());
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
