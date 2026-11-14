// Firmware PAMI 2027 (ESP32-S3).
//
// Démarrage : tirette retirée. Le PAMI attend la phase d'attaque (85 s),
// exécute la stratégie de son rôle, puis s'arrête à la fin du match (100 s).
// Le déroulé du match est dans pami_app.cpp (partagé avec le simulateur).
#include "app.h"

App app;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    appBegin(app, PAMI_ID);
    wifiBegin(app);  // facultatif : sans Wi-Fi, le PAMI marche seul
    if (app.tirette.armed()) {
        Serial.println("[Tirette] branchee : PAMI arme, la retirer pour partir.");
    } else {
        Serial.println("[Tirette] non detectee : la brancher pour armer le PAMI.");
    }
    Serial.println("Tapez \"help\" pour la console.");
}

void loop() {
    app.drive.update();
    if (app.sensorsEnabled) app.tof.update();  // capteurs ToF (non bloquant)

    // Tirette : on signale l'armement. Si ce message n'apparaît pas quand on
    // la branche, c'est le câblage (fil coupé, faux contact).
    bool wasArmed = app.tirette.armed();
    bool pulled = app.tirette.update();
    if (!wasArmed && app.tirette.armed()) {
        Serial.println("[Tirette] branchee : PAMI arme, la retirer pour partir.");
    }
    if (pulled && app.state == MatchState::WaitStart) {
        startMatch(app, 0, "tirette");
    }

    consoleUpdate(app);
    wifiUpdate(app);
    matchUpdate(app);
}
