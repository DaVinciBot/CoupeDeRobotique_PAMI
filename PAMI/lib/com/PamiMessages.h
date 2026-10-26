// Messages échangés en Wi-Fi (UDP) entre la Jetson (ou le Pi) et les PAMI.
//
// Même convention que usb_com / com_pami (dépôt CoupeDeRobotique) :
//   - ID 0..127   : hôte (Jetson, Pi) -> PAMI
//   - ID 128..255 : PAMI -> hôte
//   - structures compactes (pack 1), little-endian, flottants 32 bits
//   - trame = message | taille (1 octet) | CRC-8 | signature BA DD 1C C5
//     (voir PamiFrame.h)
//
// Chaque message vers un PAMI porte le numéro du PAMI visé (pami_id) :
// 0 = tous les PAMI. Les positions sont en mm, angles en radians, dans le
// repère de la table (x sur la longueur, y sur la largeur).
//
// Le pendant Python est tools/pami_wifi.py : à garder aligné.
#pragma once

#include <stdint.h>

namespace pami_msg {

constexpr uint8_t SIGNATURE[4] = {0xBA, 0xDD, 0x1C, 0xC5};
constexpr uint8_t ALL_PAMI = 0;
constexpr uint8_t MAX_ENEMIES = 6;

// Ports UDP
constexpr uint16_t PAMI_PORT = 5005;  // les PAMI écoutent ici
constexpr uint16_t HOST_PORT = 5006;  // l'hôte écoute ici (HELLO en diffusion, états)

enum : uint8_t {
    // hôte -> PAMI
    SET_ODOMETRIE = 2,     // recalage : position vue par la caméra
    MATCH_START = 10,      // départ / resynchronisation de l'horloge du match
    ENEMY_POSITIONS = 11,  // positions des PAMI adverses
    STOP_PAMI = 12,        // arrêt immédiat (fin de match à distance)
    SET_TEAM = 13,         // couleur de l'équipe (remplace la tirette couleur)
    RESET_PAMI = 126,      // redémarrer l'ESP32
    NACK = 127,

    // PAMI -> hôte
    UPDATE_PAMI = 128,  // état périodique (10 Hz)
    HELLO = 130,        // « je suis là » (diffusé tant que l'hôte n'a rien envoyé)
    PRINT = 254,
    UNKNOWN_MSG_TYPE = 255,
};

#pragma pack(push, 1)

struct MsgSetOdometrie {
    uint8_t command = SET_ODOMETRIE;
    uint8_t pami_id;
    float x, y, theta;
};

struct MsgMatchStart {
    uint8_t command = MATCH_START;
    uint8_t pami_id;
    uint32_t elapsed_ms;  // temps de match écoulé au moment de l'envoi
};

struct MsgEnemyPositions {
    uint8_t command = ENEMY_POSITIONS;
    uint8_t pami_id;
    uint8_t count;               // nombre de positions valides (<= MAX_ENEMIES)
    float xy[MAX_ENEMIES][2];    // centres des PAMI adverses
};

struct MsgStopPami {
    uint8_t command = STOP_PAMI;
    uint8_t pami_id;
};

struct MsgSetTeam {
    uint8_t command = SET_TEAM;
    uint8_t pami_id;
    uint8_t team;  // 0 = bleu, 1 = jaune
};

struct MsgResetPami {
    uint8_t command = RESET_PAMI;
    uint8_t pami_id;
};

struct MsgUpdatePami {
    uint8_t command = UPDATE_PAMI;
    uint8_t pami_id;
    uint8_t team;   // 0 = bleu, 1 = jaune
    uint8_t state;  // 0 attente, 1 match, 2 fini, 3 manuel
    uint32_t match_ms;
    float x, y, theta;  // odométrie (axe des roues)
};

struct MsgHello {
    uint8_t command = HELLO;
    uint8_t pami_id;
    uint8_t role;  // 0 test, 1 douve, 2 attaque
    uint8_t team;
};

struct MsgUnknownMsgType {
    uint8_t command = UNKNOWN_MSG_TYPE;
    uint8_t pami_id;
    uint8_t type_id;
};

#pragma pack(pop)

}  // namespace pami_msg
