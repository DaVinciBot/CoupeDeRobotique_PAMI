#pragma once

#include <Arduino.h>

// Servo de modélisme (SG90) piloté en PWM matériel (LEDC), non bloquant.
class ServoPwm {
   public:
    void begin(uint8_t pin, uint8_t channel = 0, uint16_t minUs = 500,
               uint16_t maxUs = 2400);
    void write(uint8_t angleDeg);  // 0..180
    void release();                // coupe le signal (le servo ne force plus)
    uint8_t angle() const { return _angle; }

   private:
    uint8_t _pin = 0;
    uint8_t _channel = 0;
    uint16_t _minUs = 500;
    uint16_t _maxUs = 2400;
    uint8_t _angle = 0;
    bool _attached = false;
};
