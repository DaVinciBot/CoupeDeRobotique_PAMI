#pragma once

#include <Arduino.h>

// Tirette de départ : il faut d'abord la voir branchée (armement), puis
// retirée. Une tirette jamais branchée ne déclenche donc jamais de départ.
class Tirette {
   public:
    void begin(uint8_t pin, uint32_t debounceMs = 50) {
        _pin = pin;
        _debounceMs = debounceMs;
        pinMode(_pin, INPUT_PULLDOWN);
        _stable = digitalRead(_pin);
        _lastRaw = _stable;
        _armed = (_stable == HIGH);
        _changedMs = millis();
    }

    // Renvoie true une seule fois, au retrait après armement.
    bool update() {
        bool raw = digitalRead(_pin);
        uint32_t now = millis();
        if (raw != _lastRaw) {
            _lastRaw = raw;
            _changedMs = now;
        }
        if (now - _changedMs < _debounceMs || raw == _stable) return false;

        _stable = raw;
        if (_stable == HIGH) {
            _armed = true;
            return false;
        }
        if (_armed) {
            _armed = false;
            return true;
        }
        return false;
    }

    bool armed() const { return _armed; }

   private:
    uint8_t _pin = 0;
    uint32_t _debounceMs = 50;
    bool _stable = LOW;
    bool _lastRaw = LOW;
    bool _armed = false;
    uint32_t _changedMs = 0;
};
