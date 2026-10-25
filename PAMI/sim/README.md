# Simulation des PAMI

Fait tourner **le vrai code des 6 PAMI** (stratégies, `GoTo`, `DiffDrive`,
déroulé du match) sur PC, avec des moteurs, un lidar et des PAMI adverses
simulés, puis affiche le match vu de haut sur l'image de la table.

## Lancer

Depuis le dossier `PAMI` :

```bash
python sim/run.py                 # équipe bleue, avec adversaires
python sim/run.py --team jaune
python sim/run.py --no-enemies
python sim/run.py --size 190 95   # taille d'un PAMI : longueur largeur (mm)
```

Le script compile, simule les 100 s du match (en ~1 s), affiche un résumé
(collisions, distance à la cible) et ouvre `sim/viewer.html` dans le
navigateur. Il faut `g++` : MinGW ou MSYS2 sous Windows.

## Le visualiseur

- **Lecture / pause** (barre d'espace), vitesse, barre de temps, flèches ← →
  pour avancer ou reculer de 0,5 s. La lecture commence à 84 s.
- Chaque PAMI est dessiné à sa **vraie taille** (rectangle), le triangle
  noir indique l'avant. Contour **rouge** = collision.
- Zone blanche devant : champ du lidar. Rectangle devant : couloir surveillé,
  **rouge** quand un obstacle y est détecté.
- Lignes pointillées : trajet direct vers la cible (douve ou zone d'attaque).
- Journal : collisions, contacts adverses, alertes (sortie d'écurie avant
  85 s…). Cliquer sur une ligne y amène. « messages série » affiche aussi
  ce que les PAMI écrivent sur leur console.

## Ce qui est simulé (et ce qui ne l'est pas)

| Simulé | Pas simulé |
|---|---|
| Rampes des moteurs pas à pas (vitesse, accélération) | Patinage, pertes de pas : la position calculée = la vraie |
| Lidar : rayons sur les bords, tours, carrières, autres PAMI | Bruit du lidar, murs du château (pas encore relevés) |
| Collisions (détectées et signalées) | Physique des chocs : les PAMI se traversent |
| Adversaires : 3 PAMI qui vont tout droit vers nos douves | Vraie stratégie adverse, robots principaux |

Les remplaçants du matériel sont dans `sim/include` (Arduino, FastAccelStepper,
LidarGS2) et `sim/src/sim_hw.cpp`. La détection du lidar simulé reprend celle
de `lib/lidar/LidarGS2.cpp` : à garder alignée si elle change.
