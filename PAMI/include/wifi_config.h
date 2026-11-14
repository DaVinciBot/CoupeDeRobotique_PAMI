// Réglages de la communication des PAMI.
//
// Seuls les PAMI d'attaque ont un module Wi-Fi 6E : un Ezurio Veda IF913
// (puce Infineon CYW55913) relié en UART, qui relaie les trames en UDP. Son
// programme est dans bridge_veda_if913/ (voir aussi lib/com/BRIDGE.md).
// Les PAMI de douve n'ont pas de module : ils jouent sans communication.
//
// Sans lien (module absent ou pas encore connecté), un PAMI marche seul :
// départ à la tirette, stratégie embarquée.
#pragma once

#include <stdint.h>

enum class ComTransport : uint8_t {
    None,        // pas de communication (PAMI autonome)
    UartBridge,  // module Wi-Fi 6E (Veda IF913) en UART
    Esp32Wifi,   // Wi-Fi intégré de l'ESP32-S3 (2,4 GHz), pour les essais
};

// Transport selon le rôle du PAMI (pami_profiles.h)
constexpr ComTransport COM_TRANSPORT_ATTACK = ComTransport::UartBridge;
constexpr ComTransport COM_TRANSPORT_OTHERS = ComTransport::None;

// UART vers le module Wi-Fi 6E (connecteur LoRa J13 de la carte).
// /!\ Broches provisoires : à fixer selon le schéma KiCad de la carte.
constexpr int8_t BRIDGE_UART_RX_PIN = 17;  // ESP32 RX <- TX du module
constexpr int8_t BRIDGE_UART_TX_PIN = 18;  // ESP32 TX -> RX du module
constexpr uint32_t BRIDGE_UART_BAUD = 921600;  // même valeur que BRIDGE_UART_BAUD du module

// Wi-Fi intégré (Esp32Wifi, essais seulement) : le nom et le mot de passe du
// réseau ne sont PAS dans le dépôt : copier include/wifi_secrets.example.h en
// include/wifi_secrets.h (ignoré par git). Avec le module Wi-Fi 6E, c'est lui
// qui connaît le réseau (bridge_veda_if913/bridge_secrets.h).

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

constexpr bool ENABLE_WIFI = true;  // false : aucune communication, pour tous

constexpr uint32_t WIFI_STATUS_PERIOD_MS = 100;  // état envoyé à l'hôte (10 Hz)
constexpr uint32_t WIFI_HELLO_PERIOD_MS = 1000;  // « je suis là », tant que l'hôte est inconnu

// --- Texte de la Jetson (calcul déporté, dépôt CoupeDeRobotique_CD) ---
// La Jetson envoie "5|id|x_cm|y_cm|yaw_deg|..." : les marqueurs ArUco qu'elle
// voit, dans son repère (origine en haut à gauche, y vers le bas). Voir
// lib/com/JetsonText.h. Les autres trames (2, 3) sont ignorées.
//
// /!\ À confirmer avec le calcul déporté avant d'activer :
//   - le coin de la table qui sert d'origine (JETSON_MIRROR_X / _Y) ;
//   - les numéros ArUco des PAMI et des robots adverses.
// Un repère faux enverrait les PAMI d'attaque chasser au mauvais endroit.
// Pour vérifier : poser un PAMI à un endroit connu et lire « [Jetson] » dans
// sa console (première trame reçue, convertie dans notre repère).
constexpr bool JETSON_MIRROR_X = false;  // x de la Jetson dans l'autre sens que le nôtre
constexpr bool JETSON_MIRROR_Y = true;   // y vers le bas chez elle, vers le haut chez nous
constexpr bool JETSON_USE_ENEMIES = false;   // positions adverses -> cible de l'attaque
constexpr bool JETSON_USE_RECALAGE = false;  // marqueur du PAMI -> recalage de l'odométrie

// Numéro ArUco porté par chaque PAMI (indice = numéro du PAMI, 0 = aucun).
// /!\ À fixer avec l'équipe.
constexpr uint8_t PAMI_ARUCO_ID[7] = {0, 0, 0, 0, 0, 0, 0};

// Numéros ArUco des robots de chaque équipe. /!\ Absents du règlement 2027 :
// valeurs des années précédentes (bleu 1 à 5, jaune 6 à 10), à confirmer.
constexpr uint8_t ARUCO_BLUE_MIN = 1, ARUCO_BLUE_MAX = 5;
constexpr uint8_t ARUCO_YELLOW_MIN = 6, ARUCO_YELLOW_MAX = 10;
