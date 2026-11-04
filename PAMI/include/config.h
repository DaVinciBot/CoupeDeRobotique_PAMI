// Configuration commune à tous les PAMI.
// Ce qui change d'un PAMI à l'autre est dans pami_profiles.h,
// les positions sur la table dans table_2027.h.
#pragma once

#include <Arduino.h>

#include "TofArray.h"

#ifndef PAMI_ID
#define PAMI_ID 1  // fourni par l'environnement PlatformIO (pio run -e pamiN)
#endif

// ============================================================
// Match (règlement 2027 + réglementation générale)
// ============================================================
constexpr uint32_t MATCH_DURATION_MS = 100000;
constexpr uint32_t ATTACK_PHASE_START_MS = 85000;  // sortie d'écurie interdite avant
constexpr uint32_t ATTACK_START_MARGIN_MS = 300;   // sécurité si l'horloge est en avance
// Règlement général H.3 : à 100 s, les robots doivent être ARRÊTÉS et leurs
// actionneurs coupés (robot qui bouge encore : -50 points). Freiner depuis
// 300 mm/s à 800 mm/s² prend 0,375 s : on commence 0,5 s avant la fin.
constexpr uint32_t MATCH_END_MARGIN_MS = 500;

// ============================================================
// Brochage carte mère PAMI (DocVB : cdr/paris/pami/elek/motherboard-pami)
// ============================================================
constexpr uint8_t LEFT_STEP_PIN = 46;
constexpr uint8_t LEFT_DIR_PIN = 3;
constexpr uint8_t LEFT_EN_PIN = 9;

constexpr uint8_t RIGHT_STEP_PIN = 47;
constexpr uint8_t RIGHT_DIR_PIN = 21;
constexpr uint8_t RIGHT_EN_PIN = 48;

constexpr uint8_t SERVO_PIN = 42;  // connecteur Servo 1

// En 2026, la tirette était câblée sur le connecteur Servo 2 (GPIO41)
// et la tirette couleur sur M0/M1 du LoRa (GPIO14/15). À confirmer.
constexpr uint8_t TIRETTE_PIN = 41;      // HIGH = tirette en place
constexpr uint8_t COLOR_PIN_VCC = 14;    // sortie à HIGH
constexpr uint8_t COLOR_PIN_READ = 15;   // HIGH = jaune, LOW = bleu

// Capteurs ToF (I2C) : voir la section « Capteurs d'obstacles » plus bas.

// ============================================================
// Mécanique
// ============================================================
constexpr float MOTOR_FULL_STEPS_PER_REV = 400.0f;  // NEMA 17 0,9°
constexpr float WHEEL_DIAMETER_MM = 60.01f;
// TODO mesurer : 132 mm à l'origine, 200 mm dans le code 2026
// (valeur gonflée pour compenser les rotations trop courtes).
constexpr float WHEEL_BASE_MM = 132.0f;

// Gabarit (règlement : périmètre <= 600 mm au départ, plus grand qu'un
// cylindre Ø80 x 150). 3 PAMI par écurie de 300 mm -> 95 mm de large max.
#ifndef PAMI_LENGTH_MM
#define PAMI_LENGTH_MM 190.0f  // dans le sens de marche
#endif
#ifndef PAMI_WIDTH_MM
#define PAMI_WIDTH_MM 95.0f
#endif
// Position de l'axe des roues par rapport au centre du PAMI (+ = vers
// l'avant, - = vers l'arrière). Le PAMI tourne autour de cet axe.
//
// /!\ MÉCA : AXE DES ROUES À L'ARRIÈRE (roue folle devant).
// Avec 3 PAMI par écurie, celui contre le bord n'a que 2,5 mm de marge.
//   - roues au centre : en tournant, son arrière déborde de 59 mm dans la
//     bordure -> il reste bloqué (même la courbe la plus douce déborde de
//     2,4 mm) : les PAMI douve 1 et 2 n'arrivent jamais.
//   - roues à 85 mm derrière le centre (10 mm du bord arrière) : l'arrière
//     déborde de 1 mm -> les 6 PAMI sortent (vérifié en simulation).
// Mettre ici la valeur RÉELLE du PAMI construit (odométrie, capteurs, arrêt
// sur les cibles en dépendent).
#ifndef PAMI_AXLE_FROM_CENTER_MM
#define PAMI_AXLE_FROM_CENTER_MM -85.0f
#endif

// Sens moteur : niveau de DIR qui fait AVANCER la roue (repris du code 2026).
constexpr bool LEFT_DIR_HIGH_IS_FORWARD = false;
constexpr bool RIGHT_DIR_HIGH_IS_FORWARD = true;

// ============================================================
// Mouvement (modifiable à chaud depuis la console : speed / accel)
// ============================================================
constexpr float DRIVE_SPEED_MM_S = 200.0f;
constexpr float DRIVE_ACCEL_MM_S2 = 800.0f;
constexpr uint32_t GOTO_TIMEOUT_MS = 15000;

// ============================================================
// Capteurs d'obstacles : ToF VL53L1X à l'avant (évitement)
// ============================================================
constexpr bool ENABLE_TOF = true;

// /!\ À confirmer avec l'élec et la méca (TODO) : nombre de capteurs,
// position, orientation, broches. Valeurs provisoires, à mettre à jour
// selon le PCB et le support des capteurs.
constexpr int8_t TOF_SDA_PIN = 1;
constexpr int8_t TOF_SCL_PIN = 2;
// Point de référence des capteurs : au milieu de l'avant du PAMI, à cette
// distance devant son centre. Positions ci-dessous en mm par rapport à ce
// point (x vers l'avant, y vers la gauche), angle + = vers la gauche.
constexpr float SENSOR_FROM_CENTER_MM = 85.0f;
constexpr TofArray::Mount TOF_SENSORS[] = {
    // XSHUT   x       y      angle
    {4, -5.0f, -35.0f, -30.0f},  // avant droit
    {5, 0.0f, 0.0f, 0.0f},       // avant centre
    {6, -5.0f, 35.0f, 30.0f},    // avant gauche
};
constexpr uint8_t TOF_COUNT = sizeof(TOF_SENSORS) / sizeof(TOF_SENSORS[0]);
// Cône de chaque capteur : resserré à ~15° (zone de 4 x 4 SPAD au lieu de
// 16 x 16, ~27°). À 27°, en longeant le bord de la table à 50 mm (PAMI
// contre le bord de l'écurie), le capteur central voit la bordure à 214 mm
// et le PAMI ne sort jamais (vu en simulation).
constexpr uint8_t TOF_ROI_SPADS = 4;
constexpr float TOF_FOV_DEG = 15.0f;  // cône correspondant (simulation, affichage)
constexpr uint16_t TOF_MAX_MM = 600;  // au-delà : on considère qu'il n'y a rien

// Zone surveillée : couloir devant le point de référence, profondeur x largeur.
constexpr uint16_t OBSTACLE_STOP_MM = 220;  // face à face à 300 mm/s : 2 x 56 mm de freinage + retard
// Demi-largeur du couloir : à peine plus que le PAMI. Dans l'écurie, le
// voisin est à 52 mm de l'axe : plus large, le PAMI le verrait et ne
// sortirait jamais (idem le long du bord de la table).
constexpr uint16_t OBSTACLE_CORRIDOR_HALF_MM = (uint16_t)(PAMI_WIDTH_MM / 2 + 2);
// Point de référence des capteurs devant l'axe des roues (sert à placer
// les points sur la table)
constexpr float SENSOR_OFFSET_X_MM = SENSOR_FROM_CENTER_MM - PAMI_AXLE_FROM_CENTER_MM;
// Distance entre l'axe des roues et le centre du PAMI quand l'axe est à
// l'arrière : pour que le PAMI (et pas son axe) arrive sur une cible.
constexpr float CENTER_AHEAD_OF_AXLE_MM =
    PAMI_AXLE_FROM_CENTER_MM < 0.0f ? -PAMI_AXLE_FROM_CENTER_MM : 0.0f;

// ============================================================
// Couleur : 0 = tirette couleur, 1 = jaune forcé, -1 = bleu forcé
// ============================================================
constexpr int COLOR_OVERRIDE = 0;

// Communication : choix du transport dans include/wifi_config.h
// (COM_TRANSPORT : passerelle Wi-Fi en UART par défaut, ou Wi-Fi intégré de
// l'ESP32 en 2,4 GHz), code dans lib/com. Facultatif : sans communication,
// départ à la tirette.

// ============================================================
// Debug
// ============================================================
#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) ((void)0)
#define DEBUG_PRINTLN(...) ((void)0)
#define DEBUG_PRINTF(...) ((void)0)
#endif
