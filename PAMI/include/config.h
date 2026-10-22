// Configuration commune à tous les PAMI.
// Ce qui change d'un PAMI à l'autre est dans pami_profiles.h,
// les positions sur la table dans table_2027.h.
#pragma once

#include <Arduino.h>

#ifndef PAMI_ID
#define PAMI_ID 1  // fourni par l'environnement PlatformIO (pio run -e pamiN)
#endif

// ============================================================
// Match (règlement 2027 + réglementation générale)
// ============================================================
constexpr uint32_t MATCH_DURATION_MS = 100000;
constexpr uint32_t ATTACK_PHASE_START_MS = 85000;  // sortie d'écurie interdite avant
constexpr uint32_t ATTACK_START_MARGIN_MS = 300;   // sécurité si l'horloge est en avance
constexpr uint32_t MATCH_END_MARGIN_MS = 200;      // s'arrêter juste avant la fin

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

constexpr int8_t LIDAR_RX_PIN = 44;  // UART0 (RX0/TX0), Serial = USB natif
constexpr int8_t LIDAR_TX_PIN = 43;

// ============================================================
// Mécanique
// ============================================================
constexpr float MOTOR_FULL_STEPS_PER_REV = 400.0f;  // NEMA 17 0,9°
constexpr float WHEEL_DIAMETER_MM = 60.01f;
// TODO mesurer : 132 mm à l'origine, 200 mm dans le code 2026
// (valeur gonflée pour compenser les rotations trop courtes).
constexpr float WHEEL_BASE_MM = 132.0f;

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
// Lidar YDLIDAR GS2 (évitement)
// ============================================================
constexpr bool ENABLE_LIDAR = true;
// Zone surveillée : couloir devant le lidar, profondeur x largeur.
constexpr uint16_t LIDAR_OBSTACLE_MM = 120;
constexpr uint16_t LIDAR_CORRIDOR_HALF_WIDTH_MM = 90;  // demi-largeur PAMI + marge
// Position du lidar devant l'axe des roues (pour placer les points sur la
// table). TODO mesurer.
constexpr float LIDAR_OFFSET_X_MM = 60.0f;
// Gauche/droite : mettre la main à gauche du PAMI, "lidar" doit donner y > 0.
constexpr bool LIDAR_Y_LEFT_POSITIVE = true;
// Indices "devant" utilisés seulement si le lidar ne donne pas sa
// calibration (repris du code 2026).
constexpr uint8_t LIDAR_FALLBACK_A_FIRST = 0;
constexpr uint8_t LIDAR_FALLBACK_A_LAST = 20;
constexpr uint8_t LIDAR_FALLBACK_B_FIRST = 140;
constexpr uint8_t LIDAR_FALLBACK_B_LAST = 159;

// ============================================================
// Couleur : 0 = tirette couleur, 1 = jaune forcé, -1 = bleu forcé
// ============================================================
constexpr int COLOR_OVERRIDE = 0;

// Communication : assurée par un module Wi-Fi externe (pas encore choisi),
// à intégrer quand il sera connu. En attendant, départ à la tirette.

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
