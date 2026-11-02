// Lien vers le second contrôleur Wi-Fi 6E (passerelle), relié en UART.
//
// L'ESP32 échange avec la passerelle exactement les trames qui circulent en
// Wi-Fi (PamiMessages.h) ; la passerelle les relaie en UDP. Elle envoie en
// plus BRIDGE_STATUS (UART seulement) pour dire si le Wi-Fi est joint et si
// l'hôte est connu. Ce que doit faire son programme : lib/com/BRIDGE.md.
#pragma once

#include <Arduino.h>

#include "FrameParser.h"
#include "PamiLink.h"

class UartLink : public PamiLink {
   public:
    void begin(HardwareSerial& serial, uint32_t baud, int8_t rxPin, int8_t txPin);

    void update() override;
    bool send(const void* message, size_t size) override;
    bool connected() const override { return statusFresh() && _wifiConnected; }
    bool hostKnown() const override { return statusFresh() && _hostKnown; }

    bool bridgeAlive() const { return statusFresh(); }
    int8_t rssi() const { return _rssi; }

   private:
    bool statusFresh() const;

    HardwareSerial* _serial = nullptr;
    FrameParser _parser;
    uint32_t _lastStatusMs = 0;
    bool _wifiConnected = false;
    bool _hostKnown = false;
    int8_t _rssi = 0;
    bool _announcedAlive = false;
    uint8_t _tx[pami_frame::MAX_MESSAGE + pami_frame::OVERHEAD];
};
