// Ce que fait le PAMI des messages reçus en Wi-Fi.
// Sans dépendance au Wi-Fi : utilisé par wifi.cpp sur le robot, et par le
// simulateur qui joue le rôle de la Jetson.
#include "app.h"

// Départ du match (si le PAMI attend encore), ou resynchronisation de son
// horloge sur celle de l'hôte (si le match est déjà lancé par la tirette).
void remoteMatchStart(App& a, uint32_t elapsedMs) {
    if (a.state == MatchState::Running) {
        a.clock.startAt(millis(), elapsedMs);
        return;
    }
    startMatch(a, elapsedMs, "wifi");
}

// Recalage : la caméra voit le PAMI ici. L'odométrie repart de cette pose.
void remoteSetOdometry(App& a, const Pose& pose) {
    a.drive.setPose(pose);
}

void remoteEnemyPositions(App& a, const float* xs, const float* ys, uint8_t count) {
    RemoteEnemies& e = a.enemies;
    e.count = count < RemoteEnemies::MAX ? count : RemoteEnemies::MAX;
    for (uint8_t i = 0; i < e.count; ++i) {
        e.x[i] = xs[i];
        e.y[i] = ys[i];
    }
    e.timeMs = millis();
    if (e.timeMs == 0) e.timeMs = 1;  // 0 = « jamais reçu »
}

void remoteStop(App& a) {
    endMatch(a, "arret demande en Wi-Fi");
}

void remoteSetTeam(App& a, Team team) {
    a.remoteTeamSet = true;
    a.remoteTeam = team;
    Serial.printf("[WiFi] couleur imposee : %s\n", teamName(team));
}
