// Lien de communication d'un PAMI avec l'hôte (Jetson, Pi), quel que soit
// le transport :
//   - UartLink : second contrôleur Wi-Fi 6E relié en UART (choix retenu,
//     ex. AW-CU640 sur le PCB du PAMI) ;
//   - WifiLink : Wi-Fi intégré de l'ESP32-S3 (2,4 GHz seulement).
// Les messages et les trames sont les mêmes dans les deux cas.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <functional>

class PamiLink {
   public:
    using Handler = std::function<void(const uint8_t* msg, size_t size)>;
    virtual ~PamiLink() = default;

    virtual void update() = 0;  // à chaque tour de loop() : réception
    virtual bool send(const void* message, size_t size) = 0;
    virtual bool connected() const = 0;  // réseau Wi-Fi joint
    virtual bool hostKnown() const = 0;  // l'hôte s'est déjà manifesté

    void onMessage(uint8_t id, Handler handler) { _handlers[id] = handler; }

   protected:
    void dispatch(const uint8_t* msg, size_t size) {
        if (size > 0 && _handlers[msg[0]]) _handlers[msg[0]](msg, size);
    }

   private:
    Handler _handlers[256];
};
