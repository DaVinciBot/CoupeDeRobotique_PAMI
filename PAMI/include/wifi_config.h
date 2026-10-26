// Réglages du Wi-Fi des PAMI.
//
// Le nom et le mot de passe du réseau ne sont PAS dans le dépôt : copier
// include/wifi_secrets.example.h en include/wifi_secrets.h (ignoré par git)
// et le remplir. Sans ce fichier, le Wi-Fi est désactivé et le PAMI marche
// seul (tirette, stratégie embarquée).
#pragma once

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

constexpr bool ENABLE_WIFI = true;

constexpr uint32_t WIFI_STATUS_PERIOD_MS = 100;  // état envoyé à l'hôte (10 Hz)
constexpr uint32_t WIFI_HELLO_PERIOD_MS = 1000;  // « je suis là », tant que l'hôte est inconnu
