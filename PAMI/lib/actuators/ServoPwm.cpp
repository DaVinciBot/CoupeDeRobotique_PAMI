#include "ServoPwm.h"

#include <esp_arduino_version.h>

static constexpr uint32_t SERVO_FREQ_HZ = 50;
static constexpr uint8_t SERVO_RESOLUTION_BITS = 14;
static constexpr uint32_t SERVO_PERIOD_US = 1000000 / SERVO_FREQ_HZ;

void ServoPwm::begin(uint8_t pin, uint8_t channel, uint16_t minUs,
                     uint16_t maxUs) {
    _pin = pin;
    _channel = channel;
    _minUs = minUs;
    _maxUs = maxUs;
}

void ServoPwm::write(uint8_t angleDeg) {
    if (angleDeg > 180) angleDeg = 180;
    _angle = angleDeg;

    if (!_attached) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcAttach(_pin, SERVO_FREQ_HZ, SERVO_RESOLUTION_BITS);
#else
        ledcSetup(_channel, SERVO_FREQ_HZ, SERVO_RESOLUTION_BITS);
        ledcAttachPin(_pin, _channel);
#endif
        _attached = true;
    }

    uint32_t pulseUs = _minUs + (uint32_t)(_maxUs - _minUs) * angleDeg / 180;
    uint32_t duty = pulseUs * ((1u << SERVO_RESOLUTION_BITS) - 1) / SERVO_PERIOD_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(_pin, duty);
#else
    ledcWrite(_channel, duty);
#endif
}

void ServoPwm::release() {
    if (!_attached) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(_pin, 0);
    ledcDetach(_pin);
#else
    ledcWrite(_channel, 0);
    ledcDetachPin(_pin);
#endif
    _attached = false;
}
