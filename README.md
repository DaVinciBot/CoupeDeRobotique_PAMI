# CDR 2027 — PAMI

Firmware des PAMI (« chevaliers ») pour Eurobot 2027, réécrit à partir du code 2026.
Matériel : carte mère ESP32-S3, drivers A4988 + NEMA 17 (17HM15-0904S), servo. La vision se fait avec des capteurs **ToF VL53L1X** (3 en éventail à l'avant, en I2C) à la place du lidar GS2.

## Contenu

```
CDR2027/
└── PAMI/                    firmware PAMI (PlatformIO)
    ├── include/
    │   ├── config.h         brochage, mécanique, vitesses, capteurs ToF
    │   ├── pami_profiles.h  rôle + micropas + départ de chaque PAMI (1 à 6)
    │   └── table_2027.h     positions sur la table (figure 8 du règlement)
    ├── lib/
    │   ├── drive/           base roulante (FastAccelStepper + odométrie)
    │   ├── sensors/         capteurs ToF (TofArray) + détection commune (ObstacleSensor)
    │   ├── VL53L1X/         bibliothèque Pololu des capteurs ToF
    │   ├── lidar/           ancien lecteur du GS2 (plus compilé, gardé pour mémoire)
    │   ├── actuators/       servo (PWM matériel)
    │   ├── match/           horloge du match, tirette
    │   ├── actions/         Wait, WaitMatchTime, Forward, Turn, GoTo, ServoMove, ServoSweep
    │   └── strategy/        enchaînement d'actions
    └── src/
        ├── main.cpp         démarrage et déroulé du match
        ├── strategies.cpp   stratégie de chaque rôle
        └── console.cpp      console série de test
```

## Ce qui change par rapport à 2026

| 2026 | 2027 |
|---|---|
| Pas générés par `loop()` avec `delayMicroseconds(500)` : lent, et les `println` ralentissaient les moteurs | Pas générés en matériel par **FastAccelStepper** : rien ne bloque |
| Actions bloquantes (`BlockingForward`, `BlockingTurn`) | Toutes les actions sont **non bloquantes** |
| Rotations au chronomètre + entraxe gonflé à 200 mm + angles « magiques » | Rotations comptées en pas, angles réels |
| Trajet codé en dur (avance / tourne / avance…) | Le PAMI va à un **point (x, y)** de la table |
| Lidar lu sans checksum, trames mélangées, quelques indices « devant » devinés | Capteurs **ToF** (VL53L1X) : chaque mesure devient un point (x, y), **couloir** surveillé devant le PAMI avec hystérésis, objet le plus proche, mesures trop vieilles ignorées |
| `PAMI_ID` modifié à la main dans `config.h` | Un environnement PlatformIO par PAMI : `pio run -e pami3` |
| Moteurs alimentés en permanence | Moteurs alimentés seulement pendant les mouvements (pauses comprises) |

## Déroulé d'un match

1. Au démarrage, le PAMI attend la **tirette** (branchée puis retirée).
2. Il attend la **phase d'attaque** (85 s + 0,3 s de marge + un décalage par PAMI pour ne pas se gêner à la sortie).
3. Il exécute la stratégie de son rôle :
   - **douve** : aller à une douve adverse en s'arrêtant devant les obstacles (capteurs ignorés dans les 15 derniers cm) ;
   - **attaque** : aller à sa zone (une par PAMI d'attaque, `table::ATTACK_ZONES`) avec l'évitement normal, puis attendre qu'un objet passe à portée des capteurs. Les bordures, carrières et remparts sont ignorés. Le PAMI suit la cible en corrigeant sa direction en roulant, s'arrête à 18 cm, avance encore de 8 cm et déploie l'actionneur ;
   - **test** : carré de 300 mm.
4. À 99,8 s, il s'arrête. L'actionneur garde sa position.

La couleur vient de la tirette couleur (ou de `COLOR_OVERRIDE`).

## Communication (Wi-Fi 6E)

Sur chaque PAMI, un **second contrôleur Wi-Fi 6E** (ex. AW-CU640, sur le PCB) est relié à l'ESP32-S3
en **UART** et fait passerelle : il relaie les trames en **UDP** sans les modifier.
Ce que doit faire son programme : [PAMI/lib/com/BRIDGE.md](PAMI/lib/com/BRIDGE.md).
Variante possible : le Wi-Fi intégré de l'ESP32-S3 (2,4 GHz seulement), `COM_TRANSPORT` dans
[wifi_config.h](PAMI/include/wifi_config.h).

Les messages ont le même format que `usb_com` / `com_pami` :
`message | taille | CRC-8 | BA DD 1C C5`, ID 0..127 hôte → PAMI, 128..255 PAMI → hôte.

**Le Wi-Fi est un plus, pas une condition** : sans lui, le PAMI part à la tirette et suit sa stratégie.

| Message | Sens | Effet sur le PAMI |
|---|---|---|
| `MATCH_START` | hôte → PAMI | départ (ou resynchronisation de l'horloge si déjà parti) |
| `SET_ODOMETRIE` | hôte → PAMI | recalage sur la position vue par la caméra |
| `ENEMY_POSITIONS` | hôte → PAMI | positions adverses : l'attaque sait qui viser (plus de tir sur un coéquipier) |
| `STOP_PAMI` | hôte → PAMI | arrêt immédiat |
| `SET_TEAM` | hôte → PAMI | couleur (remplace la tirette couleur) |
| `RESET_PAMI` | hôte → PAMI | redémarrage |
| `HELLO` | PAMI → hôte | « je suis là », diffusé chaque seconde tant que l'hôte ne s'est pas manifesté |
| `UPDATE_PAMI` | PAMI → hôte | état, temps de match et position, 10 fois par seconde |

Chaque message vers un PAMI porte son numéro (0 = tous). Détail des champs :
[PAMI/lib/com/PamiMessages.h](PAMI/lib/com/PamiMessages.h). Ports : les PAMI écoutent sur **5005**, l'hôte sur **5006**.

**Mise en route :**

1. Fixer les broches de l'UART vers la passerelle (`BRIDGE_UART_RX_PIN` / `TX_PIN`) selon le PCB.
2. Flasher. La console affiche `[Com] passerelle detectee`, puis `[Com] Wi-Fi joint par la passerelle`.
   (Variante Wi-Fi intégré : copier `wifi_secrets.example.h` en `wifi_secrets.h`, ignoré par git.)
3. Côté Jetson / Pi / PC, l'outil Python (même format, à garder aligné) :

```bash
cd PAMI
python tools/pami_wifi.py listen              # HELLO puis états des PAMI
python tools/pami_wifi.py start               # départ de tous les PAMI
python tools/pami_wifi.py odom 3 2700 250 3.14
python tools/pami_wifi.py enemies 1500 800 1200 1300
```

Depuis le code de la Jetson : `from pami_wifi import PamiLink, match_start, enemy_positions, ...`.

La simulation joue le rôle de la Jetson (positions adverses à 10 Hz, recalage à 1 Hz) ;
`python sim/run.py --no-jetson` montre le match sans Wi-Fi.

## Compiler et flasher

```bash
cd PAMI
pio run -e pami1 -t upload      # PAMI n°1 (pami1 ... pami6)
pio device monitor              # console, tapez "help"
```

## Simulation

Le vrai code des 6 PAMI peut tourner sur PC, avec un affichage du match vu de haut :

```bash
cd PAMI
python sim/run.py
```

Détails dans [PAMI/sim/README.md](PAMI/sim/README.md).

## Première mise en route (à faire dans cet ordre)

Tout se fait depuis la console série, sans reflasher.

1. **Sens des moteurs** : `f 100`. Si une roue tourne à l'envers, inverser `LEFT_DIR_HIGH_IS_FORWARD` ou `RIGHT_DIR_HIGH_IS_FORWARD` dans `config.h`.
2. **Distance** : `f 500`, mesurer. Si l'écart dépasse quelques mm, vérifier le **micropas** dans `pami_profiles.h` et `WHEEL_DIAMETER_MM`.
3. **Entraxe** : `t 360`. S'il tourne trop, diminuer l'entraxe (`base 128` par exemple), recommencer, puis reporter la valeur dans `WHEEL_BASE_MM`.
4. **Vitesse** : augmenter avec `speed 300`, `accel 1000`… jusqu'à ce que le PAMI perde des pas, puis redescendre de 20 %. Il faut environ 2,1 m en 15 s.
5. **Capteurs ToF** : au démarrage, la console doit afficher `[ToF] 3/3 capteurs actifs`. Sinon, vérifier le câblage I2C et les broches XSHUT (`TOF_SENSORS` dans `config.h`). Ensuite :
   - `tof` : distance de chaque capteur, couloir et point le plus proche. Mettre la main **devant**, puis **à gauche** : le capteur de gauche (angle +30°) doit réagir, et y doit être positif ;
   - `zone 120 180` : régler le couloir surveillé (profondeur, largeur), puis le reporter dans `config.h`.
6. **Test complet** : `start 84000` simule un départ à 84 s, et le PAMI part 1,3 s plus tard.

## Reste à faire

- [ ] **Coordonnées de la table** dans `table_2027.h` : relevées sur la figure 8 (±20 mm), à vérifier sur une vraie table.
- [ ] **Méca : axe des roues à l'arrière du PAMI** (`PAMI_AXLE_FROM_CENTER_MM`). Avec 3 PAMI par écurie, celui contre le bord n'a que 2,5 mm de marge : roues au centre, il touche la bordure dès qu'il tourne (vérifié en simulation). Roues à l'arrière, les 6 PAMI sortent sans contact.
- [ ] **Entraxe réel** (`WHEEL_BASE_MM`) et micropas de chaque PAMI.
- [ ] Vérifier le **sens des moteurs**, les **capteurs ToF** (nombre, angles, broches I2C `TOF_SDA_PIN` / `TOF_SCL_PIN` et XSHUT dans `TOF_SENSORS`, provisoires), le **sens gauche/droite** et leur position (`SENSOR_FROM_CENTER_MM`).
- [ ] Pins de la **tirette** et de la **couleur** : repris du câblage 2026, en conflit avec Servo 2 et M0/M1 du LoRa sur le schéma.
- [ ] Angles du servo d'attaque (`pami_profiles.h`).
- [ ] Position des **zones d'attaque**, des **remparts** et des **carrières** (`table_2027.h`) : relevées sur le plan beta, à vérifier sur la version définitive.
- [ ] Gyroscope (IMU ICM-45686 en SPI) pour corriger les rotations, si elle est soudée.
- [ ] Intégrer le **module de communication** quand il sera choisi.

## Organisation des branches

- `main` : code testé, celui qui part en compétition.
- `dev` : branche d'intégration et branche par défaut.
- `feat/*`, `fix/*`, `perf/*`, `test/*`, `refactor/*`, `docs/*` : travail en cours, créées depuis `dev`.
- `hotfix/*` : correction urgente, créée depuis `main`.

Les règles complètes sont dans [CONTRIBUTING.md](CONTRIBUTING.md).
