"""Compile et lance la simulation de match PAMI, puis ouvre le visualiseur.

Usage (depuis le dossier PAMI) :
    python sim/run.py                   # equipe bleue, avec adversaires
    python sim/run.py --team jaune
    python sim/run.py --no-enemies      # seulement nos PAMI seuls (par defaut :
                                        # 6 contre 6 ET seuls, au choix dans la page)
    python sim/run.py --speed 250       # vitesse max des PAMI (mm/s, defaut 300)
    python sim/run.py --size 190 95     # taille d'un PAMI (longueur largeur, mm)
    python sim/run.py --axle 0          # axe des roues par rapport au centre (mm,
                                        # + = avant, - = arriere ; defaut -85 : config.h)
    python sim/run.py --no-jetson       # sans Jetson (pas de Wi-Fi : lidar seul)
    python sim/run.py --no-open         # ne pas ouvrir le navigateur

--size et --axle remplacent les valeurs de config.h (recompilation).

Il faut un compilateur C++ (g++) : MinGW ou MSYS2 sous Windows, g++ sous
Linux/macOS. Le script le cherche dans le PATH et aux emplacements habituels.
"""

import os
import shutil
import subprocess
import sys
import webbrowser
from pathlib import Path

SIM = Path(__file__).resolve().parent
PAMI = SIM.parent
OUT = SIM / "out"

SOURCES = [
    PAMI / "src" / "pami_app.cpp",
    PAMI / "src" / "strategies.cpp",
    PAMI / "src" / "remote.cpp",
    PAMI / "lib" / "actions" / "Actions.cpp",
    PAMI / "lib" / "strategy" / "Strategy.cpp",
    PAMI / "lib" / "drive" / "DiffDrive.cpp",
    PAMI / "lib" / "actuators" / "ServoPwm.cpp",
    SIM / "src" / "sim_hw.cpp",
    SIM / "src" / "sim_world.cpp",
    SIM / "src" / "sim_main.cpp",
]

# sim/include en premier : il remplace Arduino.h, FastAccelStepper.h et LidarGS2.h
INCLUDES = [
    SIM / "include",
    SIM / "src",
    PAMI / "include",
    PAMI / "src",
    PAMI / "lib" / "actions",
    PAMI / "lib" / "actuators",
    PAMI / "lib" / "drive",
    PAMI / "lib" / "match",
    PAMI / "lib" / "strategy",
]

COMPILER_CANDIDATES = [
    "C:/msys64/ucrt64/bin/g++.exe",
    "C:/msys64/mingw64/bin/g++.exe",
    "C:/MinGW/bin/g++.exe",
    "C:/mingw64/bin/g++.exe",
]


def find_compiler():
    found = shutil.which("g++")
    if found:
        return found
    for candidate in COMPILER_CANDIDATES:
        if Path(candidate).exists():
            return candidate
    sys.exit(
        "Compilateur g++ introuvable. Installez MinGW-w64 ou MSYS2 "
        "(https://www.msys2.org) puis relancez."
    )


def main():
    args = sys.argv[1:]
    open_viewer = "--no-open" not in args
    defines, sim_args = [], []
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--size" and i + 2 < len(args):
            defines += [f"-DPAMI_LENGTH_MM={float(args[i + 1])}f",
                        f"-DPAMI_WIDTH_MM={float(args[i + 2])}f"]
            i += 3
        elif a == "--axle" and i + 1 < len(args):
            defines.append(f"-DPAMI_AXLE_FROM_CENTER_MM={float(args[i + 1])}f")
            i += 2
        elif a == "--no-open":
            i += 1
        else:
            sim_args.append(a)
            i += 1

    OUT.mkdir(exist_ok=True)
    gxx = find_compiler()
    env = dict(os.environ)
    env["PATH"] = str(Path(gxx).parent) + os.pathsep + env.get("PATH", "")

    exe = OUT / ("pami_sim.exe" if os.name == "nt" else "pami_sim")
    cmd = [gxx, "-std=gnu++1z", "-O2", "-Wall", "-o", str(exe)] + defines
    cmd += [f"-I{p}" for p in INCLUDES]
    cmd += [str(s) for s in SOURCES]
    print("Compilation...", flush=True)
    if subprocess.run(cmd, env=env).returncode != 0:
        sys.exit("La compilation a echoue.")

    # Vitesse par défaut de la simulation : 300 mm/s (config.h garde la
    # valeur du robot, à régler après calibration)
    if "--speed" not in sim_args:
        sim_args += ["--speed", "300"]

    # Deux déroulés, choisis dans le visualiseur (menu "Adversaires") :
    # 6 contre 6 et nos PAMI seuls. --no-enemies : seulement le second.
    only_solo = "--no-enemies" in sim_args
    base_args = [a for a in sim_args if a != "--no-enemies"]
    runs = [] if only_solo else [("6 contre 6", "sim_data.js", [])]
    runs.append(("nos PAMI seuls", "sim_data_sans.js", ["--no-enemies"]))
    for title, filename, extra in runs:
        print(f"\nSimulation du match ({title})...", flush=True)
        run = subprocess.run([str(exe), "--out", str(OUT / filename)] + base_args + extra,
                             env=env)
        if run.returncode != 0:
            sys.exit("La simulation a echoue.")
    if only_solo:
        (OUT / "sim_data.js").unlink(missing_ok=True)  # pas de 6 contre 6 périmé

    viewer = SIM / "viewer.html"
    print(f"\nVisualiseur : {viewer}")
    if open_viewer:
        webbrowser.open(viewer.as_uri())


if __name__ == "__main__":
    main()
