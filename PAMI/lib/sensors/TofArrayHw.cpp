// Partie matérielle de TofArray (robot seulement) : capteurs VL53L1X en I2C,
// bibliothèque Pololu (lib/VL53L1X).
#include <VL53L1X.h>
#include <Wire.h>

#include "TofArray.h"

static VL53L1X s_sensors[TofArray::MAX_SENSORS];

uint8_t TofArray::begin(const Config& cfg) {
    configure(cfg);
    Wire.begin(_cfg.sdaPin, _cfg.sclPin, _cfg.i2cHz);

    // Tous éteints, puis allumés un par un pour leur donner une adresse
    for (uint8_t i = 0; i < _cfg.count; ++i) {
        int8_t pin = _cfg.sensors[i].xshutPin;
        if (pin >= 0) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }
    delay(10);

    uint8_t ok = 0;
    for (uint8_t i = 0; i < _cfg.count; ++i) {
        int8_t pin = _cfg.sensors[i].xshutPin;
        if (pin >= 0) {
            digitalWrite(pin, HIGH);  // sortie de veille (XSHUT au niveau haut)
            delay(10);
        }
        VL53L1X& s = s_sensors[i];
        s.setBus(&Wire);
        s.setTimeout(50);
        if (!s.init()) {
            Serial.printf("[ToF] capteur %u : ne repond pas\n", i);
            _alive[i] = false;
            continue;
        }
        if (pin >= 0) s.setAddress(_cfg.firstAddress + i);
        s.setDistanceMode(VL53L1X::Short);  // jusqu'à ~1,3 m, moins sensible à la lumière
        s.setMeasurementTimingBudget(_cfg.timingBudgetMs * 1000);
        s.setROISize(_cfg.roiSpads, _cfg.roiSpads);  // cône plus étroit
        s.startContinuous(_cfg.timingBudgetMs);
        _alive[i] = true;
        ok++;
    }
    Serial.printf("[ToF] %u/%u capteurs actifs\n", ok, _cfg.count);
    return ok;
}

void TofArray::update() {
    for (uint8_t i = 0; i < _cfg.count; ++i) {
        if (!_alive[i]) continue;
        VL53L1X& s = s_sensors[i];
        if (!s.dataReady()) continue;  // rien de nouveau : on ne bloque pas
        uint16_t mm = s.read(false);
        bool ok = s.ranging_data.range_status == VL53L1X::RangeValid;
        setReading(i, mm, ok);
    }
    process();
}
