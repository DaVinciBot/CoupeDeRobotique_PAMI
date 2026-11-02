"""Parler aux PAMI en Wi-Fi (UDP), depuis la Jetson, le Pi ou un PC.

Même format que lib/com/PamiMessages.h et PamiFrame.h (à garder aligné) :
    trame = message | taille | CRC-8 (poly 0x07) | BA DD 1C C5
    message = ID (1 octet) + champs (struct.pack, little-endian, float32)

Les PAMI écoutent sur le port 5005 ; l'hôte écoute sur le port 5006
(HELLO diffusés par les PAMI, puis leur état à 10 Hz).

Utilisation en ligne de commande (sur le réseau des PAMI) :
    python tools/pami_wifi.py listen                 # affiche HELLO et états
    python tools/pami_wifi.py start                  # départ du match, tous les PAMI
    python tools/pami_wifi.py start --elapsed 85000  # ... à t = 85 s (test)
    python tools/pami_wifi.py stop --pami 3
    python tools/pami_wifi.py team jaune
    python tools/pami_wifi.py odom 3 2700 250 3.14   # recalage du PAMI 3
    python tools/pami_wifi.py enemies 1500 800 1200 1300   # positions adverses
    python tools/pami_wifi.py selftest               # vérifie l'encodage

Depuis un programme Python (Jetson) :
    link = PamiLink()
    link.send(match_start())
    for pami_id, msg, fields in link.poll(): ...
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from enum import IntEnum

SIGNATURE = bytes([0xBA, 0xDD, 0x1C, 0xC5])
PAMI_PORT = 5005
HOST_PORT = 5006
ALL_PAMI = 0
MAX_ENEMIES = 6


class Msg(IntEnum):
    # hôte -> PAMI
    SET_ODOMETRIE = 2
    MATCH_START = 10
    ENEMY_POSITIONS = 11
    STOP_PAMI = 12
    SET_TEAM = 13
    RESET_PAMI = 126
    NACK = 127
    # PAMI -> hôte
    UPDATE_PAMI = 128
    HELLO = 130
    PRINT = 254
    UNKNOWN_MSG_TYPE = 255


# Champs après l'ID (struct, little-endian)
FORMATS = {
    Msg.SET_ODOMETRIE: "<Bfff",          # pami_id, x, y, theta
    Msg.MATCH_START: "<BI",              # pami_id, elapsed_ms
    Msg.ENEMY_POSITIONS: "<BB12f",       # pami_id, count, 6 x (x, y)
    Msg.STOP_PAMI: "<B",                 # pami_id
    Msg.SET_TEAM: "<BB",                 # pami_id, team (0 bleu, 1 jaune)
    Msg.RESET_PAMI: "<B",                # pami_id
    Msg.UPDATE_PAMI: "<BBBIfff",         # pami_id, team, state, match_ms, x, y, theta
    Msg.HELLO: "<BBB",                   # pami_id, role, team
    Msg.UNKNOWN_MSG_TYPE: "<BB",         # pami_id, type_id
}

STATES = {0: "attente", 1: "match", 2: "fini", 3: "manuel"}
ROLES = {0: "test", 1: "douve", 2: "attaque"}
TEAMS = {0: "bleu", 1: "jaune"}


# ---------------------------------------------------------------- trames
def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode(message: bytes) -> bytes:
    if not 0 < len(message) <= 250:
        raise ValueError("message vide ou trop long")
    body = message + bytes([len(message)])
    return body + bytes([crc8(body)]) + SIGNATURE


def decode(frame: bytes) -> bytes | None:
    if len(frame) < 7 or frame[-4:] != SIGNATURE:
        return None
    size = frame[-6]
    if size == 0 or size + 6 != len(frame) or crc8(frame[: size + 1]) != frame[-5]:
        return None
    return frame[:size]


def pack(msg: Msg, *fields) -> bytes:
    return bytes([msg]) + struct.pack(FORMATS[msg], *fields)


def unpack(message: bytes):
    msg = Msg(message[0])
    fmt = FORMATS.get(msg)
    if fmt is None or struct.calcsize(fmt) != len(message) - 1:
        return msg, ()
    return msg, struct.unpack(fmt, message[1:])


# ---------------------------------------------------------------- messages
def match_start(elapsed_ms: int = 0, pami_id: int = ALL_PAMI) -> bytes:
    return pack(Msg.MATCH_START, pami_id, elapsed_ms)


def set_odometry(pami_id: int, x: float, y: float, theta: float) -> bytes:
    return pack(Msg.SET_ODOMETRIE, pami_id, x, y, theta)


def enemy_positions(positions, pami_id: int = ALL_PAMI) -> bytes:
    positions = list(positions)[:MAX_ENEMIES]
    flat = [v for xy in positions for v in xy]
    flat += [0.0] * (2 * MAX_ENEMIES - len(flat))
    return pack(Msg.ENEMY_POSITIONS, pami_id, len(positions), *flat)


def stop(pami_id: int = ALL_PAMI) -> bytes:
    return pack(Msg.STOP_PAMI, pami_id)


def set_team(team: int, pami_id: int = ALL_PAMI) -> bytes:
    return pack(Msg.SET_TEAM, pami_id, team)


def reset(pami_id: int) -> bytes:
    return pack(Msg.RESET_PAMI, pami_id)


# ---------------------------------------------------------------- réseau
class PamiLink:
    """Socket UDP de l'hôte : reçoit les PAMI, leur envoie des messages."""

    def __init__(self, bind: str = "0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind((bind, HOST_PORT))
        self.sock.setblocking(False)
        self.pamis: dict[int, str] = {}  # numéro -> adresse IP

    def send(self, message: bytes, pami_id: int | None = None, ip: str | None = None):
        """Envoie à un PAMI connu, à une IP donnée, ou à tous (diffusion)."""
        frame = encode(message)
        if ip is None and pami_id is not None:
            ip = self.pamis.get(pami_id)
        self.sock.sendto(frame, (ip or "255.255.255.255", PAMI_PORT))

    def poll(self):
        """Messages reçus depuis le dernier appel : (pami_id, Msg, champs)."""
        out = []
        while True:
            try:
                frame, (ip, _port) = self.sock.recvfrom(512)
            except (BlockingIOError, OSError):
                return out
            message = decode(frame)
            if message is None:
                continue
            msg, fields = unpack(message)
            if fields:
                self.pamis[fields[0]] = ip
                out.append((fields[0], msg, fields[1:]))


# ---------------------------------------------------------------- outil
def describe(pami_id, msg, fields) -> str:
    if msg == Msg.HELLO:
        role, team = fields
        return f"PAMI {pami_id} : HELLO ({ROLES.get(role, role)}, {TEAMS.get(team, team)})"
    if msg == Msg.UPDATE_PAMI:
        team, state, match_ms, x, y, theta = fields
        return (f"PAMI {pami_id} : {STATES.get(state, state):7s} t={match_ms / 1000:6.2f} s "
                f"({x:6.0f}, {y:6.0f}, {theta * 57.2958:5.0f} deg) {TEAMS.get(team, team)}")
    return f"PAMI {pami_id} : {msg.name} {fields}"


def selftest():
    msg = set_odometry(3, 2700.0, 250.0, 3.14)
    frame = encode(msg)
    assert decode(frame) == msg
    assert unpack(msg) == (Msg.SET_ODOMETRIE, struct.unpack("<Bfff", msg[1:]))
    bad = bytearray(frame)
    bad[2] ^= 0xFF
    assert decode(bytes(bad)) is None
    print("SET_ODOMETRIE(3, 2700, 250, 3.14) ->", frame.hex(" "))
    print("selftest ok")


def main():
    parser = argparse.ArgumentParser(description="Messages Wi-Fi vers les PAMI")
    parser.add_argument("--pami", type=int, default=ALL_PAMI, help="numéro du PAMI (0 = tous)")
    parser.add_argument("--ip", help="IP du PAMI (sinon : diffusion)")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("listen")
    p = sub.add_parser("start")
    p.add_argument("--elapsed", type=int, default=0, help="temps de match écoulé (ms)")
    sub.add_parser("stop")
    p = sub.add_parser("team")
    p.add_argument("couleur", choices=["bleu", "jaune"])
    p = sub.add_parser("odom")
    p.add_argument("id", type=int)
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("theta", type=float)
    p = sub.add_parser("enemies")
    p.add_argument("coords", type=float, nargs="*", help="x1 y1 x2 y2 ...")
    p = sub.add_parser("reset")
    p.add_argument("id", type=int)
    sub.add_parser("selftest")
    args = parser.parse_args()

    if args.cmd == "selftest":
        selftest()
        return

    link = PamiLink()
    if args.cmd == "listen":
        print(f"Ecoute sur le port {HOST_PORT} (Ctrl+C pour arreter)")
        try:
            while True:
                for pami_id, msg, fields in link.poll():
                    print(describe(pami_id, msg, fields))
                time.sleep(0.02)
        except KeyboardInterrupt:
            return

    if args.cmd == "start":
        message = match_start(args.elapsed, args.pami)
    elif args.cmd == "stop":
        message = stop(args.pami)
    elif args.cmd == "team":
        message = set_team(1 if args.couleur == "jaune" else 0, args.pami)
    elif args.cmd == "odom":
        message = set_odometry(args.id, args.x, args.y, args.theta)
    elif args.cmd == "enemies":
        if len(args.coords) % 2:
            sys.exit("il faut des paires x y")
        pairs = list(zip(args.coords[0::2], args.coords[1::2]))
        message = enemy_positions(pairs, args.pami)
    else:  # reset
        message = reset(args.id)
    link.send(message, ip=args.ip)
    print("envoye :", encode(message).hex(" "))


if __name__ == "__main__":
    main()
