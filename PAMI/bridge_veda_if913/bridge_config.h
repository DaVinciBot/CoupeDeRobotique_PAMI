/*
 * Réglages de la passerelle Wi-Fi 6E (Ezurio Veda IF913, puce CYW55913).
 * Les valeurs doivent correspondre à celles de l'ESP32 :
 * PAMI/include/wifi_config.h et PAMI/lib/com/PamiMessages.h.
 */
#ifndef BRIDGE_CONFIG_H_
#define BRIDGE_CONFIG_H_

#include "cybsp.h"
#include "cy_wcm.h"
#include "frame.h"

/* ---------------------------------------------------------------------------
 * Réseau Wi-Fi 6E (routeur du calcul déporté)
 * Le nom et le mot de passe ne sont PAS dans le dépôt : copier
 * bridge_secrets.example.h en bridge_secrets.h (ignoré par git).
 * ------------------------------------------------------------------------- */
#if __has_include("bridge_secrets.h")
#include "bridge_secrets.h"
#endif

#ifndef BRIDGE_WIFI_SSID
#define BRIDGE_WIFI_SSID     ""
#endif
#ifndef BRIDGE_WIFI_PASSWORD
#define BRIDGE_WIFI_PASSWORD ""
#endif

/* Le 6 GHz impose le WPA3 (SAE). */
#define BRIDGE_WIFI_SECURITY        (CY_WCM_SECURITY_WPA3_SAE)
#define BRIDGE_WIFI_RETRY_MS        (1000u)   /* attente entre deux essais de connexion */

/* ---------------------------------------------------------------------------
 * UART vers l'ESP32-S3 (connecteur LoRa J13 de la carte du PAMI)
 * Croisé : TX du module -> RX de l'ESP32 (BRIDGE_UART_RX_PIN côté ESP32),
 *          RX du module <- TX de l'ESP32 (BRIDGE_UART_TX_PIN côté ESP32).
 * Sur le kit de test, broches Arduino D1 (TX) et D0 (RX).
 * /!\ Sur la carte du PAMI : broches à fixer selon le schéma de la carte
 *     adaptatrice.
 * ------------------------------------------------------------------------- */
#define BRIDGE_UART_TX_PIN          (CYBSP_D1)
#define BRIDGE_UART_RX_PIN          (CYBSP_D0)
#define BRIDGE_UART_BAUD            (921600u) /* = BRIDGE_UART_BAUD de l'ESP32 */

/* ---------------------------------------------------------------------------
 * UDP (PamiMessages.h)
 * ------------------------------------------------------------------------- */
#define BRIDGE_PAMI_PORT            (5005u)   /* la passerelle écoute ici */
#define BRIDGE_HOST_PORT            (5005u)   /* la Jetson écoute ici (WIFI_LISTEN_PORT) */
#define BRIDGE_UDP_MAX              (1472u)   /* plus grand datagramme reçu */

/* Trames texte de la Jetson ("cmd|arg|arg
", calcul déporté) : recopiées sur
 * l'UART par morceaux, dans le message JETSON_TEXT :
 *   | 20 | numéro du morceau (0, 1...) | 1 si dernier | texte (247 octets max) | */
#define BRIDGE_JETSON_TEXT_ID       (20u)
#define BRIDGE_TEXT_CHUNK           (FRAME_MAX_MESSAGE - 3u)

/* Messages 128..255 : envoyés par un PAMI (diffusion d'un autre PAMI) -> ignorés */
#define BRIDGE_FIRST_PAMI_MSG_ID    (128u)

/* État envoyé à l'ESP32 (BRIDGE_STATUS, ID 200), sur l'UART seulement */
#define BRIDGE_STATUS_ID            (200u)
#define BRIDGE_STATUS_PERIOD_MS     (500u)

/* Messages de suivi sur l'UART de debug du kit (USB, 115200 bauds) */
#define BRIDGE_DEBUG                (1)

#endif /* BRIDGE_CONFIG_H_ */
