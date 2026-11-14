# Passerelle Wi-Fi 6E des PAMI d'attaque (Ezurio Veda IF913)

Programme du module Wi-Fi 6E placé sur les **3 PAMI d'attaque** (PAMI 4, 5 et 6).
Le module (Ezurio **Veda IF913**, puce Infineon **CYW55913**) est relié à l'ESP32-S3
en **UART** et relaie les trames entre l'UART et le Wi-Fi (UDP), sans les modifier.
Spécification complète : [../lib/com/BRIDGE.md](../lib/com/BRIDGE.md).

Les PAMI de douve (1, 2, 3) n'ont pas de module : ils jouent sans communication
(`COM_TRANSPORT_OTHERS` dans [../include/wifi_config.h](../include/wifi_config.h)).

```
Jetson  <-- Wi-Fi 6E (UDP 5005) -->  Veda IF913  <-- UART 921 600 bauds -->  ESP32-S3
```

La passerelle accepte nos trames binaires **et** le texte de la Jetson du calcul déporté
(`5|id|x_cm|y_cm|yaw_deg|...`, dépôt CoupeDeRobotique_CD), qu'elle recopie à l'ESP32 par
morceaux (message `JETSON_TEXT`). Elle ignore les messages des autres PAMI.

## Fichiers

| Fichier | Rôle |
|---|---|
| `main.c` | Démarrage de la carte, puis lancement de la passerelle |
| `bridge.c` / `bridge.h` | Connexion Wi-Fi (et reconnexion), relais UDP ↔ UART, état `BRIDGE_STATUS` toutes les 500 ms |
| `frame.c` / `frame.h` | Format des trames, identique à `lib/com/PamiFrame` et `FrameParser` de l'ESP32 |
| `bridge_config.h` | Broches UART, vitesse, ports, sécurité Wi-Fi |
| `bridge_secrets.example.h` | Modèle pour le nom et le mot de passe du réseau (à copier en `bridge_secrets.h`, ignoré par git) |
| `Makefile`, `deps/` | Projet ModusToolbox (réglages repris de l'exemple officiel Infineon) |

## Compiler et flasher

Il faut **ModusToolbox** (Infineon, gratuit) : https://www.infineon.com/modustoolbox

Le plus sûr est de partir de l'exemple officiel, qui règle la carte et les bibliothèques :

1. Dans ModusToolbox, **New Application** → choisir le kit (**CYW955913EVK-01**, ou le
   BSP du kit Ezurio Veda IF913 s'il est proposé) → choisir l'exemple
   **Wi-Fi UDP Server** (`mtb-example-threadx-wifi-udp-server`).
2. Dans le projet créé, **supprimer** `udp_server.c` et `udp_server.h`, puis **copier**
   ici `main.c`, `bridge.c`, `bridge.h`, `frame.c`, `frame.h`, `bridge_config.h`.
3. Copier `bridge_secrets.example.h` en `bridge_secrets.h` et mettre le nom et le mot
   de passe du réseau **6 GHz** du routeur (WPA3).
4. Dans le `Makefile` du projet, ajouter la ligne du pays (canaux 6 GHz européens) :
   `DEFINES+= CY_WIFI_COUNTRY=WHD_COUNTRY_FRANCE`
5. Compiler et flasher : **Build** puis **Program** dans l'IDE, ou `make program` dans
   le terminal `modus-shell`.

## Brancher sur l'ESP32

| Kit / module | ESP32-S3 (`wifi_config.h`) |
|---|---|
| TX (Arduino **D1**) | RX (`BRIDGE_UART_RX_PIN`, 17 pour l'instant) |
| RX (Arduino **D0**) | TX (`BRIDGE_UART_TX_PIN`, 18 pour l'instant) |
| GND | GND |

Les deux côtés sont en 3,3 V. La vitesse doit être la même des deux côtés (921 600 bauds :
`BRIDGE_UART_BAUD` ici et dans `wifi_config.h`).

## Tester

1. Ouvrir le port série **debug** du kit (USB, 115 200 bauds). On doit voir :
   `[WiFi] connecte a '...'`, l'adresse IP, puis `[UDP] ecoute sur le port 5005`.
2. Flasher un PAMI d'attaque (`pio run -e pami5 -t upload`) et ouvrir sa console : il doit
   afficher `[Com] passerelle Wi-Fi 6E sur UART`, puis que la passerelle répond.
3. Sur un PC connecté au même réseau, depuis le dossier `PAMI` :
   `python tools/pami_wifi.py listen`. Le PAMI doit apparaître (message `HELLO`).
   Ensuite, `python tools/pami_wifi.py team jaune` (l'hôte devient connu) : le PAMI
   envoie alors son état 10 fois par seconde.
4. Texte de la Jetson : `python tools/pami_wifi.py texte "5|7|150|80|90"`. La console du
   PAMI affiche `[Jetson] marqueur 7 : ...` avec la position convertie dans notre repère.

## À vérifier sur le vrai matériel

Ce code suit les fonctions de l'exemple officiel Infineon, mais il n'a **pas encore été
compilé avec ModusToolbox** ni testé sur le module :

- **Broches UART** : `CYBSP_D0` / `CYBSP_D1` sur le kit Infineon. Sur le kit Ezurio et
  sur la carte adaptatrice des PAMI, elles peuvent changer (`bridge_config.h`).
- **Vitesse UART** : le message `[UART] ... bauds obtenus` au démarrage dit si les
  921 600 bauds sont atteints. Sinon, baisser des deux côtés (460 800 par exemple).
- **Diffusion** : tant que l'hôte est inconnu, la passerelle envoie en diffusion
  (255.255.255.255). Si `tools/pami_wifi.py` ne voit rien, il faut faire envoyer un
  message par l'hôte d'abord : la passerelle mémorise alors son adresse.
- **Bande 6 GHz** : le routeur doit avoir un réseau 6 GHz en **WPA3**.
