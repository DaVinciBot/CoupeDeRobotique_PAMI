// Wi-Fi du PAMI (robot seulement) : relie les messages reçus aux actions
// de remote.cpp et envoie l'état du PAMI à l'hôte.
#include <string.h>

#include "PamiMessages.h"
#include "WifiLink.h"
#include "app.h"
#include "wifi_config.h"

using namespace pami_msg;

namespace {

WifiLink wifiLink;
bool enabled = false;
uint32_t lastHelloMs = 0;
uint32_t lastStatusMs = 0;

// Copie le message reçu dans sa structure si la taille correspond
template <typename T>
bool readMsg(const uint8_t* msg, size_t size, T& out) {
    if (size != sizeof(T)) return false;
    memcpy(&out, msg, sizeof(T));
    return true;
}

bool forMe(const App& a, uint8_t pamiId) {
    return pamiId == ALL_PAMI || pamiId == a.profile->id;
}

}  // namespace

void wifiBegin(App& a) {
    if (!ENABLE_WIFI || strlen(WIFI_SSID) == 0) {
        Serial.println("[WiFi] desactive (pas de include/wifi_secrets.h)");
        return;
    }
    enabled = true;
    wifiLink.begin(WIFI_SSID, WIFI_PASSWORD, PAMI_PORT, HOST_PORT);

    wifiLink.onMessage(MATCH_START, [&a](const uint8_t* m, size_t n) {
        MsgMatchStart msg;
        if (readMsg(m, n, msg) && forMe(a, msg.pami_id)) remoteMatchStart(a, msg.elapsed_ms);
    });
    wifiLink.onMessage(SET_ODOMETRIE, [&a](const uint8_t* m, size_t n) {
        MsgSetOdometrie msg;
        if (readMsg(m, n, msg) && forMe(a, msg.pami_id)) {
            remoteSetOdometry(a, {msg.x, msg.y, msg.theta});
        }
    });
    wifiLink.onMessage(ENEMY_POSITIONS, [&a](const uint8_t* m, size_t n) {
        MsgEnemyPositions msg;
        if (!readMsg(m, n, msg) || !forMe(a, msg.pami_id)) return;
        float xs[MAX_ENEMIES], ys[MAX_ENEMIES];
        uint8_t count = msg.count < MAX_ENEMIES ? msg.count : MAX_ENEMIES;
        for (uint8_t i = 0; i < count; ++i) {
            xs[i] = msg.xy[i][0];
            ys[i] = msg.xy[i][1];
        }
        remoteEnemyPositions(a, xs, ys, count);
    });
    wifiLink.onMessage(STOP_PAMI, [&a](const uint8_t* m, size_t n) {
        MsgStopPami msg;
        if (readMsg(m, n, msg) && forMe(a, msg.pami_id)) remoteStop(a);
    });
    wifiLink.onMessage(SET_TEAM, [&a](const uint8_t* m, size_t n) {
        MsgSetTeam msg;
        if (readMsg(m, n, msg) && forMe(a, msg.pami_id)) {
            remoteSetTeam(a, msg.team == 1 ? Team::Yellow : Team::Blue);
        }
    });
    wifiLink.onMessage(RESET_PAMI, [&a](const uint8_t* m, size_t n) {
        MsgResetPami msg;
        if (readMsg(m, n, msg) && forMe(a, msg.pami_id)) {
            Serial.println("[WiFi] redemarrage demande");
            delay(50);
            ESP.restart();
        }
    });
}

void wifiUpdate(App& a) {
    if (!enabled) return;
    wifiLink.update();
    if (!wifiLink.connected()) return;

    uint32_t now = millis();
    // Se signaler tant que l'hôte ne s'est pas manifesté
    if (!wifiLink.hostKnown() && now - lastHelloMs >= WIFI_HELLO_PERIOD_MS) {
        lastHelloMs = now;
        MsgHello hello;
        hello.pami_id = a.profile->id;
        hello.role = (uint8_t)a.role;
        hello.team = a.team == Team::Yellow ? 1 : 0;
        wifiLink.send(&hello, sizeof(hello));
    }
    // État périodique
    if (wifiLink.hostKnown() && now - lastStatusMs >= WIFI_STATUS_PERIOD_MS) {
        lastStatusMs = now;
        Pose p = a.drive.pose();
        MsgUpdatePami up;
        up.pami_id = a.profile->id;
        up.team = a.team == Team::Yellow ? 1 : 0;
        up.state = (uint8_t)a.state;
        up.match_ms = a.clock.elapsedMs();
        up.x = p.x;
        up.y = p.y;
        up.theta = p.theta;
        wifiLink.send(&up, sizeof(up));
    }
}
