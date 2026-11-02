// Simulation : remplace le cœur Arduino de l'ESP32 sur PC.
// Seules les fonctions utilisées par le code PAMI sont fournies.
#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>

using std::max;
using std::min;

typedef uint8_t byte;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

template <typename T, typename L, typename H>
T constrain(T x, L lo, H hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Temps simulé (avancé par la boucle du simulateur)
extern uint64_t g_simMicros;
inline uint32_t millis() { return (uint32_t)(g_simMicros / 1000); }
inline uint32_t micros() { return (uint32_t)g_simMicros; }
inline void delay(uint32_t) {}  // le temps n'avance que dans la boucle du simulateur
inline void delayMicroseconds(uint32_t) {}

// Broches : on mémorise juste les niveaux (la couleur est lue ici)
extern int g_pinLevels[64];
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, uint8_t level) {
    if (pin < 64) g_pinLevels[pin] = level;
}
inline int digitalRead(uint8_t pin) { return pin < 64 ? g_pinLevels[pin] : LOW; }

// PWM (servo)
inline void ledcSetup(uint8_t, uint32_t, uint8_t) {}
inline void ledcAttachPin(uint8_t, uint8_t) {}
inline void ledcDetachPin(uint8_t) {}
inline void ledcWrite(uint8_t, uint32_t) {}

// Série : les lignes écrites deviennent des événements du journal,
// rattachés au PAMI en cours d'exécution.
class SimSerial {
   public:
    void begin(unsigned long) {}
    int available() { return 0; }
    int read() { return -1; }

    size_t print(const char* s);
    size_t print(char c);
    size_t print(int v);
    size_t print(unsigned v);
    size_t print(long v);
    size_t print(unsigned long v);
    size_t print(double v);
    size_t println() { return print('\n'); }
    template <typename T>
    size_t println(T v) {
        size_t n = print(v);
        return n + print('\n');
    }
    size_t printf(const char* fmt, ...);
};
extern SimSerial Serial;

class HardwareSerial {
   public:
    explicit HardwareSerial(int) {}
};
