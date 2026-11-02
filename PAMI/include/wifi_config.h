// Réglages de la communication des PAMI.
//
// Transport retenu : un second contrôleur Wi-Fi 6E (ex. AW-CU640, sur le PCB
// du PAMI) relié en UART, qui relaie les trames en UDP (lib/com/BRIDGE.md).
// Variante : Wi-Fi intégré de l'ESP32-S3 (2,4 GHz seulement).
//
// Sans lien (pas de passerelle, ou Wi-Fi intégré sans identifiants), le
// PAMI marche seul : départ à la tirette, stratégie embarquée.
#pragma once

#include <stdint.h>

enum class ComTransport : uint8_t {
    UartBridge,    // passerelle Wi-Fi 6E en UART (choix retenu)
    Esp32Wifi,     // Wi-Fi intégré de l'ESP32-S3 (2,4 GHz)
};
constexpr ComTransport COM_TRANSPORT = ComTransport::UartBridge;

// UART vers la passerelle. /!\ Broches à fixer selon le PCB (TODO) : ces
// valeurs sont provisoires et doivent être libres sur la carte.
constexpr int8_t BRIDGE_UART_RX_PIN = 17;  // ESP32 RX <- TX passerelle
constexpr int8_t BRIDGE_UART_TX_PIN = 18;  // ESP32 TX -> RX passerelle
constexpr uint32_t BRIDGE_UART_BAUD = 921600;

// Wi-Fi intégré (variante Esp32Wifi) : le nom et le mot de passe du réseau ne
// sont PAS dans le dépôt : copier include/wifi_secrets.example.h en
// include/wifi_secrets.h (ignoré par git). Avec la passerelle, c'est elle qui
// connaît le réseau.

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

constexpr bool ENABLE_WIFI = true;  // false : aucune communication

constexpr uint32_t WIFI_STATUS_PERIOD_MS = 100;  // état envoyé à l'hôte (10 Hz)
constexpr uint32_t WIFI_HELLO_PERIOD_MS = 1000;  // « je suis là », tant que l'hôte est inconnu
