#include "UartLink.h"

#include <string.h>

#include "PamiMessages.h"

// La passerelle envoie BRIDGE_STATUS toutes les 500 ms : sans nouvelles
// depuis plus longtemps, on la considère absente.
static constexpr uint32_t BRIDGE_TIMEOUT_MS = 1500;

void UartLink::begin(HardwareSerial& serial, uint32_t baud, int8_t rxPin, int8_t txPin) {
    _serial = &serial;
    _serial->setRxBufferSize(1024);
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
    Serial.printf("[Com] passerelle Wi-Fi 6E sur UART (%lu bauds)\n", (unsigned long)baud);
}

bool UartLink::statusFresh() const {
    return _lastStatusMs != 0 && millis() - _lastStatusMs <= BRIDGE_TIMEOUT_MS;
}

void UartLink::update() {
    if (_serial == nullptr) return;
    while (_serial->available()) {
        size_t size = _parser.feed((uint8_t)_serial->read());
        if (size == 0) continue;
        const uint8_t* msg = _parser.message();

        if (msg[0] == pami_msg::BRIDGE_STATUS && size == sizeof(pami_msg::MsgBridgeStatus)) {
            pami_msg::MsgBridgeStatus st;
            memcpy(&st, msg, sizeof(st));
            bool wasConnected = connected();
            _lastStatusMs = millis();
            if (_lastStatusMs == 0) _lastStatusMs = 1;
            _wifiConnected = st.wifi_connected != 0;
            _hostKnown = st.host_known != 0;
            _rssi = st.rssi;
            if (!_announcedAlive) {
                Serial.println("[Com] passerelle detectee");
                _announcedAlive = true;
            }
            if (connected() && !wasConnected) {
                Serial.printf("[Com] Wi-Fi joint par la passerelle (%d dBm)\n", _rssi);
            }
            continue;
        }
        dispatch(msg, size);  // message de l'hôte, relayé tel quel
    }
    if (_announcedAlive && !statusFresh()) {
        Serial.println("[Com] passerelle muette");
        _announcedAlive = false;
    }
}

bool UartLink::send(const void* message, size_t size) {
    if (_serial == nullptr) return false;
    size_t len = pami_frame::encode(message, size, _tx);
    if (len == 0) return false;
    return _serial->write(_tx, len) == len;
}
