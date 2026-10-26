// État d'un PAMI, partagé entre main.cpp, pami_app.cpp, strategies.cpp et
// console.cpp (et le simulateur, qui en fait tourner six).
#pragma once

#include <Arduino.h>

#include "DiffDrive.h"
#include "LidarGS2.h"
#include "MatchClock.h"
#include "ServoPwm.h"
#include "Strategy.h"
#include "Tirette.h"
#include "config.h"
#include "pami_profiles.h"
#include "table_2027.h"

enum class MatchState : uint8_t {
    WaitStart = 0,  // en attente de la tirette
    Running = 1,    // match en cours
    Ended = 2,      // match fini, robot arrêté
    Manual = 3,     // mouvement lancé depuis la console
};

// Positions des PAMI adverses reçues en Wi-Fi (caméra de la Jetson).
struct RemoteEnemies {
    static constexpr uint8_t MAX = 6;
    uint8_t count = 0;
    float x[MAX] = {}, y[MAX] = {};
    uint32_t timeMs = 0;  // millis() à la réception (0 = jamais reçu)
    bool fresh(uint32_t maxAgeMs) const { return timeMs != 0 && millis() - timeMs <= maxAgeMs; }
};

struct App {
    DiffDrive drive;
    LidarGS2 lidar;
    ServoPwm servo;
    MatchClock clock;
    Tirette tirette;
    Strategy strategy;

    const PamiProfile* profile = nullptr;
    Role role = Role::Test;
    Team team = Team::Blue;
    MatchState state = MatchState::WaitStart;
    bool lidarEnabled = false;

    // Reçu en Wi-Fi (remote.cpp)
    RemoteEnemies enemies;
    bool remoteTeamSet = false;  // couleur imposée par l'hôte
    Team remoteTeam = Team::Blue;

    const LidarGS2* lidarOrNull() const { return lidarEnabled ? &lidar : nullptr; }
};

extern App app;

// pami_app.cpp
void appBegin(App& a, uint8_t pamiId, HardwareSerial& lidarSerial);
void matchUpdate(App& a);  // à chaque tour de boucle, après drive.update()
void startMatch(App& a, uint32_t elapsedMs, const char* source);
void endMatch(App& a, const char* reason);
Team readTeam();
void applyRoleIdlePosition(App& a);

// remote.cpp : ce que fait le PAMI des messages reçus en Wi-Fi
// (partagé avec le simulateur, qui joue le rôle de la Jetson)
void remoteMatchStart(App& a, uint32_t elapsedMs);
void remoteSetOdometry(App& a, const Pose& pose);
void remoteEnemyPositions(App& a, const float* xs, const float* ys, uint8_t count);
void remoteStop(App& a);
void remoteSetTeam(App& a, Team team);

// wifi.cpp (robot seulement)
void wifiBegin(App& a);
void wifiUpdate(App& a);

void buildMatchStrategy(App& a);  // strategies.cpp
void consoleUpdate(App& a);       // console.cpp
