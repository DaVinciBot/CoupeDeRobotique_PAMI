// Lien de communication d'un PAMI avec l'hôte (Jetson, Pi), quel que soit
// le transport :
//   - UartLink : second contrôleur Wi-Fi 6E relié en UART (choix retenu,
//     Ezurio Veda IF913, sur les PAMI d'attaque) ;
//   - WifiLink : Wi-Fi intégré de l'ESP32-S3 (2,4 GHz seulement).
// Les messages et les trames sont les mêmes dans les deux cas. Le texte de
// la Jetson (calcul déporté) est aussi accepté : voir JetsonText.h.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <functional>

#include "JetsonText.h"
#include "PamiMessages.h"

class PamiLink {
   public:
    using Handler = std::function<void(const uint8_t* msg, size_t size)>;
    using TextHandler = std::function<void(const char* text, size_t len)>;
    virtual ~PamiLink() = default;

    virtual void update() = 0;  // à chaque tour de loop() : réception
    virtual bool send(const void* message, size_t size) = 0;
    virtual bool connected() const = 0;  // réseau Wi-Fi joint
    virtual bool hostKnown() const = 0;  // l'hôte s'est déjà manifesté

    void onMessage(uint8_t id, Handler handler) { _handlers[id] = handler; }
    // Texte de la Jetson (JetsonText.h), une fois les morceaux réunis
    void onText(TextHandler handler) { _textHandler = handler; }

   protected:
    // Message binaire reçu (trame déjà vérifiée)
    void dispatch(const uint8_t* msg, size_t size) {
        if (size == 0) return;
        if (msg[0] >= pami_msg::FIRST_PAMI_MSG_ID) return;  // un autre PAMI (diffusion)
        if (msg[0] == pami_msg::JETSON_TEXT) {
            size_t len = _text.feed(msg, size);
            if (len > 0) dispatchText(_text.text(), len);
            return;
        }
        if (_handlers[msg[0]]) _handlers[msg[0]](msg, size);
    }
    // Texte de la Jetson reçu en entier
    void dispatchText(const char* text, size_t len) {
        if (_textHandler) _textHandler(text, len);
    }

   private:
    Handler _handlers[256];
    TextHandler _textHandler;
    jetson_text::Reassembler _text;
};
