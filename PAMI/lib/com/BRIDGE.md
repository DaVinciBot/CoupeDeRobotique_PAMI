# Passerelle Wi-Fi 6E des PAMI — ce que doit faire son programme

Sur chaque PAMI, un second contrôleur (ex. **AW-CU640**, soudé sur le PCB) s'occupe
uniquement du Wi-Fi 6E. Il est relié à l'ESP32-S3 par un **UART** et fait **passerelle** :
il relaie les trames entre l'UART et le réseau, **sans les modifier ni les comprendre**.

```
Jetson / Pi  <-- Wi-Fi 6E, UDP -->  passerelle  <-- UART -->  ESP32-S3 (PAMI)
```

Le format des trames, les ports et les messages sont définis dans
[PamiMessages.h](PamiMessages.h) et [PamiFrame.h](PamiFrame.h) ; côté hôte,
[tools/pami_wifi.py](../../tools/pami_wifi.py).

## Liaison UART avec l'ESP32

| Réglage | Valeur |
|---|---|
| Vitesse | 921 600 bauds, 8N1, sans contrôle de flux (`BRIDGE_UART_BAUD`) |
| Broches côté ESP32 | `BRIDGE_UART_RX_PIN` / `BRIDGE_UART_TX_PIN` dans `include/wifi_config.h` — **à fixer selon le PCB** |
| Niveaux | 3,3 V |

Sur l'UART circulent des trames, l'une après l'autre :

```
| message (taille octets) | taille (1 octet) | CRC-8 (poly 0x07) | BA DD 1C C5 |
```

## Ce que fait la passerelle

1. **Se connecter au réseau Wi-Fi 6E** (nom et mot de passe dans son propre programme),
   se reconnecter seule en cas de perte. Écouter en UDP sur le **port 5005**.

2. **Wi-Fi → UART** : chaque datagramme UDP reçu sur le port 5005 est **une trame**.
   - Si sa signature, sa taille ou son CRC est faux : l'ignorer.
   - Sinon : mémoriser l'adresse IP de l'expéditeur (c'est **l'hôte**) et recopier la
     trame **telle quelle** sur l'UART.

3. **UART → Wi-Fi** : découper le flux UART en trames (accumuler les octets jusqu'à la
   signature `BA DD 1C C5`, vérifier taille et CRC — voir `FrameParser.cpp`).
   Chaque trame valide est envoyée **telle quelle** en un datagramme UDP vers :
   - l'hôte mémorisé, port **5006** ;
   - ou, tant qu'aucun hôte n'est connu, en **diffusion** (broadcast) sur le port 5006.

4. **État, toutes les 500 ms**, sur l'UART uniquement (jamais en Wi-Fi) : une trame
   `BRIDGE_STATUS` (ID 200) :

   | Octet | Champ | Valeur |
   |---|---|---|
   | 0 | ID | 200 |
   | 1 | `wifi_connected` | 1 si le réseau est joint, sinon 0 |
   | 2 | `host_known` | 1 si l'hôte a déjà envoyé un message, sinon 0 |
   | 3 | `rssi` | puissance du signal en dBm (entier signé), 0 si inconnue |

   Exemple (joint, hôte connu, −55 dBm) : `c8 01 01 c9 04 <CRC> ba dd 1c c5`.
   Sans cette trame pendant 1,5 s, l'ESP32 considère la passerelle absente.

C'est tout : pas d'accusé de réception, pas de file d'attente. Une trame perdue n'est
pas grave, l'ESP32 renvoie son état 10 fois par seconde et l'hôte fait de même.

## Règlement (règlement général Eurobot, v1.2)

- **F.6** : la passerelle et le point d'accès Wi-Fi doivent être **sur la table**
  (dans le PAMI, le robot ou le dispositif de calcul déporté) : aucune communication
  avec un système hors de la table pendant le match.
- **F.7** : la passerelle ne doit pas permettre de **piloter** le PAMI à distance ;
  elle ne fait que relayer des informations.
- **F.5** : en compétition, le 2,4 GHz est saturé ; le 5/6 GHz est recommandé.

## Tester sans passerelle

Un adaptateur USB-UART 3,3 V branché sur les broches de l'ESP32 et un petit script
Python qui encode/décode les trames (`tools/pami_wifi.py`, fonctions `encode` /
`decode`) suffisent à simuler la passerelle sur PC.
