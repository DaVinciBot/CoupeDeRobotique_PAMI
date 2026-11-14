// Trames texte de la Jetson (calcul déporté, dépôt CoupeDeRobotique_CD,
// src/net/Protocol.hpp) : "cmd|arg|arg", une par ligne, en UDP.
//
//   cmd 5 : "5|id|x_cm|y_cm|yaw_deg|id|x_cm|..." positions des marqueurs
//           ArUco, dans le repère de la Jetson : origine en haut à gauche,
//           x vers la droite, y vers le bas, en cm ; yaw en degrés entiers.
//   cmd 2 et 3 (numéro attribué, zones de dépôt) : inutiles aux PAMI.
//
// La passerelle Wi-Fi 6E recopie ce texte sur l'UART en morceaux
// (message JETSON_TEXT, PamiMessages.h) : Reassembler les remet bout à bout.
// Sans dépendance à Arduino : utilisé aussi par le simulateur et les tests.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace jetson_text {

constexpr size_t MAX_TEXT = 1500;   // un datagramme UDP
constexpr size_t MAX_MARKERS = 40;  // kMaxVisionItems côté Jetson

struct Marker {
    int id;
    int xCm, yCm;
    int yawDeg;
};

// Lit une ligne "5|...". Renvoie le nombre de marqueurs lus (au plus `max`),
// -1 si ce n'est pas une trame cmd 5 valide.
int parseVision(const char* line, size_t len, Marker* out, size_t max);

// Appelle fn(ligne, longueur) pour chaque ligne non vide du texte.
template <typename F>
void forEachLine(const char* text, size_t len, F fn) {
    size_t start = 0;
    for (size_t i = 0; i <= len; ++i) {
        if (i == len || text[i] == '\n') {
            size_t end = i;
            while (end > start && (text[end - 1] == '\r' || text[end - 1] == ' ')) --end;
            if (end > start) fn(text + start, end - start);
            start = i + 1;
        }
    }
}

// Morceaux JETSON_TEXT : | 20 | numéro (0, 1...) | 1 si dernier | texte |
class Reassembler {
   public:
    // `msg` : message JETSON_TEXT complet. Renvoie la longueur du texte quand
    // le dernier morceau arrive (texte dans text()), 0 sinon. Un morceau
    // perdu fait oublier le texte entier.
    size_t feed(const uint8_t* msg, size_t size);
    const char* text() const { return _buf; }

   private:
    char _buf[MAX_TEXT + 1] = {};
    size_t _len = 0;
    int _expected = -1;  // numéro attendu, -1 : attendre un morceau 0
};

// Repère de la Jetson -> repère de la table des PAMI (mm, radians, sens
// trigo). mirrorX / mirrorY : axes de la Jetson inversés par rapport aux
// nôtres (y vers le bas chez elle : mirrorY = true).
struct FrameConfig {
    bool mirrorX;
    bool mirrorY;
    float lengthMm;  // taille de la table en x
    float widthMm;   // en y
};

void toTable(const Marker& m, const FrameConfig& c, float& xMm, float& yMm, float& theta);

}  // namespace jetson_text
