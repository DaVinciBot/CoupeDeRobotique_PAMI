#include "WifiLink.h"

#include "PamiFrame.h"
#include "PamiMessages.h"

static constexpr uint32_t RECONNECT_PERIOD_MS = 2000;

void WifiLink::begin(const char* ssid, const char* password, uint16_t localPort,
                     uint16_t hostPort) {
    _ssid = ssid;
    _password = password;
    _localPort = localPort;
    _hostPort = hostPort;
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // pas de mise en veille : latence minimale
    WiFi.setAutoReconnect(true);
    WiFi.begin(_ssid, _password);  // non bloquant
    _lastAttemptMs = millis();
    Serial.printf("[WiFi] connexion a \"%s\"...\n", _ssid);
}

void WifiLink::update() {
    bool isConnected = connected();
    if (isConnected && !_wasConnected) {
        Serial.printf("[WiFi] connecte, IP %s\n", WiFi.localIP().toString().c_str());
        _udpStarted = _udp.begin(_localPort);
        if (!_udpStarted) Serial.println("[WiFi] ERREUR : port UDP indisponible");
    }
    if (!isConnected && _wasConnected) {
        Serial.println("[WiFi] connexion perdue");
        _udp.stop();
        _udpStarted = false;
    }
    _wasConnected = isConnected;

    if (!isConnected) {
        if (millis() - _lastAttemptMs > RECONNECT_PERIOD_MS) {
            _lastAttemptMs = millis();
            WiFi.reconnect();
        }
        return;
    }
    if (!_udpStarted) return;

    // Lire tous les datagrammes en attente
    for (int len = _udp.parsePacket(); len > 0; len = _udp.parsePacket()) {
        IPAddress from = _udp.remoteIP();
        int n = _udp.read(_rx, sizeof(_rx));
        if (n <= 0 || len > (int)sizeof(_rx)) continue;  // trop long : ignoré
        size_t size = pami_frame::decode(_rx, (size_t)n);
        bool text = size == 0 && _rx[0] >= '0' && _rx[0] <= '9';  // texte de la Jetson
        if (size == 0 && !text) continue;                            // trame invalide
        if (size > 0 && _rx[0] >= pami_msg::FIRST_PAMI_MSG_ID) continue;  // autre PAMI
        _host = from;
        _hostKnown = true;
        _lastRxMs = millis();
        if (text) {
            dispatchText((const char*)_rx, (size_t)n);
        } else {
            dispatch(_rx, size);
        }
    }
}

bool WifiLink::send(const void* message, size_t size) {
    if (!_udpStarted) return false;
    size_t len = pami_frame::encode(message, size, _tx);
    if (len == 0) return false;
    IPAddress to = _hostKnown ? _host : WiFi.broadcastIP();
    if (!_udp.beginPacket(to, _hostPort)) return false;
    _udp.write(_tx, len);
    return _udp.endPacket() == 1;
}
