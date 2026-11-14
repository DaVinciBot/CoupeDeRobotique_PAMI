/*
 * Passerelle Wi-Fi 6E <-> UART des PAMI d'attaque (spécification :
 * PAMI/lib/com/BRIDGE.md).
 *
 * Deux tâches :
 *  - réseau : connexion au Wi-Fi (et reconnexion), socket UDP sur le port
 *    5005 ; chaque trame valide reçue est recopiée sur l'UART, et le texte
 *    de la Jetson ("5|id|x|y|yaw...") y est recopié par morceaux JETSON_TEXT ;
 *  - UART : découpe les octets venant de l'ESP32 en trames et les envoie en
 *    UDP vers l'hôte (port 5005, ou en diffusion tant qu'il est inconnu) ;
 *    envoie aussi BRIDGE_STATUS à l'ESP32 toutes les 500 ms.
 */
#ifndef BRIDGE_H_
#define BRIDGE_H_

#include "cyabs_rtos.h"

/* Initialise l'UART et lance les deux tâches. Renvoie CY_RSLT_SUCCESS. */
cy_rslt_t bridge_start(void);

#endif /* BRIDGE_H_ */
