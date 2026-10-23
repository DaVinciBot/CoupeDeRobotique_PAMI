#pragma once

#include <Arduino.h>

// Horloge du match, démarrée au retrait de la tirette.
class MatchClock {
   public:
    void startAt(uint32_t localMs, uint32_t elapsedAtThatTimeMs = 0) {
        _startMs = localMs - elapsedAtThatTimeMs;
        _started = true;
    }
    void reset() { _started = false; }
    bool started() const { return _started; }
    uint32_t elapsedMs() const { return _started ? millis() - _startMs : 0; }

   private:
    bool _started = false;
    uint32_t _startMs = 0;
};
