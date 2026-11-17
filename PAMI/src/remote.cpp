// Ce que fait le PAMI des messages reçus en Wi-Fi.
// Sans dépendance au Wi-Fi : utilisé par wifi.cpp sur le robot, et par le
// simulateur qui joue le rôle de la Jetson.
#include "app.h"
#include "JetsonText.h"

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

namespace {

void storePositions(RemoteEnemies& e, const float* xs, const float* ys, uint8_t count) {
    e.count = count < RemoteEnemies::MAX ? count : RemoteEnemies::MAX;
    for (uint8_t i = 0; i < e.count; ++i) {
        e.x[i] = xs[i];
        e.y[i] = ys[i];
    }
    e.timeMs = millis();
    if (e.timeMs == 0) e.timeMs = 1;  // 0 = « jamais reçu »
}

}  // namespace

void remoteEnemyPositions(App& a, const float* xs, const float* ys, uint8_t count) {
    storePositions(a.enemies, xs, ys, count);
}

// Nos autres robots : l'attaque ne vise pas un adversaire déjà au contact
// d'un coéquipier, et ne fonce pas dans un coéquipier (strategies.cpp).
void remoteAllyPositions(App& a, const float* xs, const float* ys, uint8_t count) {
    storePositions(a.allies, xs, ys, count);
}

void remoteStop(App& a) {
    endMatch(a, "arret demande en Wi-Fi");
}

void remoteSetTeam(App& a, Team team) {
    a.remoteTeamSet = true;
    a.remoteTeam = team;
    Serial.printf("[WiFi] couleur imposee : %s\n", teamName(team));
}

// --- Texte de la Jetson (calcul déporté) : seules les positions (cmd 5) servent.
namespace {

bool isTeamAruco(int id, Team team) {
    if (team == Team::Yellow) return id >= ARUCO_YELLOW_MIN && id <= ARUCO_YELLOW_MAX;
    return id >= ARUCO_BLUE_MIN && id <= ARUCO_BLUE_MAX;
}

bool isEnemyAruco(int id, Team us) {
    return isTeamAruco(id, us == Team::Blue ? Team::Yellow : Team::Blue);
}

void jetsonVision(App& a, const jetson_text::Marker* m, int count) {
    static bool logged = false;
    const jetson_text::FrameConfig frame = {JETSON_MIRROR_X, JETSON_MIRROR_Y, table::LENGTH_MM,
                                            table::WIDTH_MM};
    uint8_t pamiId = a.profile ? a.profile->id : 0;
    int ownId = pamiId < sizeof(PAMI_ARUCO_ID) ? PAMI_ARUCO_ID[pamiId] : 0;

    float xs[RemoteEnemies::MAX], ys[RemoteEnemies::MAX];
    float allyX[RemoteEnemies::MAX], allyY[RemoteEnemies::MAX];
    uint8_t n = 0, nAllies = 0;
    for (int i = 0; i < count; ++i) {
        float x, y, theta;
        jetson_text::toTable(m[i], frame, x, y, theta);
        if (!logged) {
            Serial.printf("[Jetson] marqueur %d : (%d cm, %d cm, %d deg) -> (%.0f, %.0f) mm, %.2f rad\n",
                          m[i].id, m[i].xCm, m[i].yCm, m[i].yawDeg, x, y, theta);
        }
        if (ownId > 0 && m[i].id == ownId) {
            if (JETSON_USE_RECALAGE) remoteSetOdometry(a, {x, y, theta});
        } else if (isEnemyAruco(m[i].id, a.team) && n < RemoteEnemies::MAX) {
            xs[n] = x;
            ys[n] = y;
            ++n;
        } else if (ownId > 0 && isTeamAruco(m[i].id, a.team) && nAllies < RemoteEnemies::MAX) {
            // (sans notre numéro ArUco, on se compterait soi-même comme coéquipier)
            allyX[nAllies] = x;
            allyY[nAllies] = y;
            ++nAllies;
        }
    }
    if (!logged) {
        logged = true;
        if (!JETSON_USE_ENEMIES && !JETSON_USE_RECALAGE) {
            Serial.println("[Jetson] positions ignorees : repere a confirmer (wifi_config.h)");
        }
    }
    if (JETSON_USE_ENEMIES) {
        remoteEnemyPositions(a, xs, ys, n);
        remoteAllyPositions(a, allyX, allyY, nAllies);
    }
}

}  // namespace

void remoteJetsonText(App& a, const char* text, size_t len) {
    jetson_text::forEachLine(text, len, [&a](const char* line, size_t n) {
        jetson_text::Marker m[jetson_text::MAX_MARKERS];
        int count = jetson_text::parseVision(line, n, m, jetson_text::MAX_MARKERS);
        if (count > 0) jetsonVision(a, m, count);
    });
}
