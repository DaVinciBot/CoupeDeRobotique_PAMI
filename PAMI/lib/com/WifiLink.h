// Lien Wi-Fi (UDP) entre un PAMI et l'hôte (Jetson ou Pi).
//
// - Wi-Fi intégré de l'ESP32-S3, en station, sur la bande 2,4 GHz du routeur
//   (l'ESP32 ne capte ni le 5 GHz ni le 6 GHz).
// - Rien ne bloque : connexion et reconnexion se font en tâche de fond,
//   update() lit les datagrammes reçus et appelle les fonctions enregistrées.
// - L'hôte est appris automatiquement : c'est l'expéditeur du dernier
//   message valide. Tant qu'il n'est pas connu, send() diffuse sur le réseau.
//
// Le PAMI doit marcher sans Wi-Fi (tirette, stratégie embarquée) : le lien
// n'apporte que des informations en plus.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <functional>

class WifiLink {
   public:
    using Handler = std::function<void(const uint8_t* msg, size_t size)>;

    void begin(const char* ssid, const char* password, uint16_t localPort, uint16_t hostPort);
    void update();  // à chaque tour de loop()

    void onMessage(uint8_t id, Handler handler) { _handlers[id] = handler; }
    bool send(const void* message, size_t size);

    bool connected() const { return WiFi.status() == WL_CONNECTED; }
    bool hostKnown() const { return _hostKnown; }
    uint32_t lastRxMs() const { return _lastRxMs; }

   private:
    void dispatch(const uint8_t* frame, size_t len);

    WiFiUDP _udp;
    const char* _ssid = nullptr;
    const char* _password = nullptr;
    uint16_t _localPort = 0;
    uint16_t _hostPort = 0;
    bool _udpStarted = false;
    bool _wasConnected = false;
    uint32_t _lastAttemptMs = 0;
    bool _hostKnown = false;
    IPAddress _host;
    uint32_t _lastRxMs = 0;
    Handler _handlers[256];
    uint8_t _rx[300];
    uint8_t _tx[300];
};
