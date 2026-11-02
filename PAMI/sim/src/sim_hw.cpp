// Simulation : implémentation des remplaçants du matériel
// (temps, broches, port série, moteurs pas à pas, capteurs ToF).
#include <stdarg.h>

#include <map>
#include <vector>

#include "Arduino.h"
#include "FastAccelStepper.h"
#include "TofArray.h"
#include "sim_log.h"

// ------------------------------------------------------------------
// Temps, broches, journal
// ------------------------------------------------------------------
uint64_t g_simMicros = 0;
int g_pinLevels[64] = {};
std::vector<SimEvent> g_events;
int g_currentPami = 0;

// Nos PAMI : "PAMI 3" ; ceux de l'adversaire (id + 100) : "Adv. 3"
std::string pamiName(int id) {
    char buf[16];
    if (id > 100) {
        snprintf(buf, sizeof(buf), "Adv. %d", id - 100);
    } else {
        snprintf(buf, sizeof(buf), "PAMI %d", id);
    }
    return buf;
}

void simLog(const std::string& who, const std::string& kind, const std::string& text) {
    g_events.push_back({millis(), who, kind, text});
}

// ------------------------------------------------------------------
// Port série : une ligne complète = un événement "log"
// ------------------------------------------------------------------
SimSerial Serial;
static std::map<int, std::string> s_lines;

static size_t serialWrite(const char* s) {
    std::string& line = s_lines[g_currentPami];
    for (const char* p = s; *p; ++p) {
        if (*p == '\n') {
            if (!line.empty()) simLog(pamiName(g_currentPami), "log", line);
            line.clear();
        } else if (*p != '\r') {
            line += *p;
        }
    }
    return strlen(s);
}

size_t SimSerial::print(const char* s) { return serialWrite(s); }
size_t SimSerial::print(char c) {
    char s[2] = {c, 0};
    return serialWrite(s);
}
size_t SimSerial::print(int v) { return printf("%d", v); }
size_t SimSerial::print(unsigned v) { return printf("%u", v); }
size_t SimSerial::print(long v) { return printf("%ld", v); }
size_t SimSerial::print(unsigned long v) { return printf("%lu", v); }
size_t SimSerial::print(double v) { return printf("%.2f", v); }

size_t SimSerial::printf(const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    return serialWrite(buf);
}

// ------------------------------------------------------------------
// Moteurs pas à pas
// ------------------------------------------------------------------
static std::vector<FastAccelStepper*> s_steppers;

FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(uint8_t) {
    FastAccelStepper* s = new FastAccelStepper();
    s->simOwner = g_currentPami;
    s_steppers.push_back(s);
    return s;
}

void simTickSteppers(double dtS) {
    for (FastAccelStepper* s : s_steppers) s->simStep(dtS);
}

bool FastAccelStepper::isRunning() const {
    return _v != 0.0 || getCurrentPosition() != _target;
}

void FastAccelStepper::stopMove() {
    // Décélère jusqu'à l'arrêt : nouvelle cible = position d'arrêt
    double a = std::max<int32_t>(1, _accel);
    double brake = _v * _v / (2.0 * a);
    _target = (int32_t)lround(_pos + (_v > 0 ? brake : -brake));
}

void FastAccelStepper::forceStop() {
    _v = 0.0;
    _pos = (double)getCurrentPosition();
    _target = (int32_t)_pos;
}

void FastAccelStepper::simStep(double dtS) {
    double a = std::max<int32_t>(1, _accel);
    double vMax = std::max<uint32_t>(1, _speedHz);
    double dist = _target - _pos;

    if (fabs(dist) < 0.5 && fabs(_v) <= a * dtS * 1.5) {
        _pos = _target;
        _v = 0.0;
        return;
    }

    // Vitesse voulue : vitesse max, limitée pour pouvoir freiner à temps
    double vWanted = (dist > 0 ? 1.0 : -1.0) * std::min(vMax, sqrt(2.0 * a * fabs(dist)));
    double dv = vWanted - _v;
    double maxDv = a * dtS;
    if (dv > maxDv) dv = maxDv;
    if (dv < -maxDv) dv = -maxDv;
    _v += dv;
    _pos += _v * dtS;

    if (!_enabled && _v != 0.0 && !_warnedDisabled) {
        _warnedDisabled = true;
        simLog(pamiName(simOwner), "alerte", "pas envoyes alors que les moteurs sont coupes");
    }
}

// ------------------------------------------------------------------
// Capteurs ToF : la géométrie et la détection sont celles du robot
// (lib/sensors/TofArray.cpp, ObstacleSensor.cpp). Seule la partie I2C est
// remplacée : les distances viennent du monde simulé (SimWorld::scanTof).
// ------------------------------------------------------------------
uint8_t TofArray::begin(const Config& cfg) {
    configure(cfg);
    return _cfg.count;
}

void TofArray::update() {}  // le simulateur appelle setReading() puis process()
