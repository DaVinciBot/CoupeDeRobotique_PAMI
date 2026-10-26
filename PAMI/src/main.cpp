// Firmware PAMI 2027 (ESP32-S3).
//
// Démarrage : tirette retirée. Le PAMI attend la phase d'attaque (85 s),
// exécute la stratégie de son rôle, puis s'arrête à la fin du match (100 s).
// Le déroulé du match est dans pami_app.cpp (partagé avec le simulateur).
#include "app.h"

App app;
static HardwareSerial LidarSerial(1);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    appBegin(app, PAMI_ID, LidarSerial);
    wifiBegin(app);  // facultatif : sans Wi-Fi, le PAMI marche seul
    Serial.println("Attente tirette. Tapez \"help\" pour la console.");
}

void loop() {
    app.drive.update();

    if (app.tirette.update() && app.state == MatchState::WaitStart) {
        startMatch(app, 0, "tirette");
    }

    consoleUpdate(app);
    wifiUpdate(app);
    matchUpdate(app);
}
