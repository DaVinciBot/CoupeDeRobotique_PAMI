"""Compile et lance la simulation de match PAMI, puis ouvre le visualiseur.

Usage (depuis le dossier PAMI) :
    python sim/run.py                   # equipe bleue, avec adversaires
    python sim/run.py --team jaune
    python sim/run.py --no-enemies
    python sim/run.py --size 190 95     # taille d'un PAMI (longueur largeur, mm)
    python sim/run.py --no-open         # ne pas ouvrir le navigateur

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
    sim_args = [a for a in args if a != "--no-open"]

    OUT.mkdir(exist_ok=True)
    gxx = find_compiler()
    env = dict(os.environ)
    env["PATH"] = str(Path(gxx).parent) + os.pathsep + env.get("PATH", "")

    exe = OUT / ("pami_sim.exe" if os.name == "nt" else "pami_sim")
    cmd = [gxx, "-std=gnu++1z", "-O2", "-Wall", "-o", str(exe)]
    cmd += [f"-I{p}" for p in INCLUDES]
    cmd += [str(s) for s in SOURCES]
    print("Compilation...")
    if subprocess.run(cmd, env=env).returncode != 0:
        sys.exit("La compilation a echoue.")

    data = OUT / "sim_data.js"
    print("Simulation du match...")
    run = subprocess.run([str(exe), "--out", str(data)] + sim_args, env=env)
    if run.returncode != 0:
        sys.exit("La simulation a echoue.")

    viewer = SIM / "viewer.html"
    print(f"\nVisualiseur : {viewer}")
    if open_viewer:
        webbrowser.open(viewer.as_uri())


if __name__ == "__main__":
    main()
