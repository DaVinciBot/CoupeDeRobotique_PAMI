// Journal de la simulation : messages série des PAMI, collisions, contacts.
#pragma once

#include <stdint.h>

#include <string>
#include <vector>

struct SimEvent {
    uint32_t t;        // ms depuis le départ du match
    std::string who;   // "PAMI 3", "Adv. 1", ...
    std::string kind;  // "log", "collision", "contact", "alerte"
    std::string text;
};

extern std::vector<SimEvent> g_events;
extern int g_currentPami;  // PAMI en cours d'exécution (pour Serial)

void simLog(const std::string& who, const std::string& kind, const std::string& text);
std::string pamiName(int id);
