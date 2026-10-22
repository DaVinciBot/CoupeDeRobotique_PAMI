// État global du PAMI, partagé entre main.cpp, strategies.cpp et console.cpp.
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

    const LidarGS2* lidarOrNull() const { return lidarEnabled ? &lidar : nullptr; }
};

extern App app;

void startMatch(App& a, uint32_t elapsedMs, const char* source);
void endMatch(App& a, const char* reason);
Team readTeam();
void applyRoleIdlePosition(App& a);

void buildMatchStrategy(App& a);  // strategies.cpp
void consoleUpdate(App& a);       // console.cpp
