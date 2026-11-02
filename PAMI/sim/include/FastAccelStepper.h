// Simulation : remplace FastAccelStepper. Chaque moteur suit une rampe
// trapézoïdale (vitesse max, accélération) vers sa position cible, avec le
// même comportement que la bibliothèque pour moveTo / stopMove / forceStop.
// Les roues ne patinent pas : la position simulée est la position réelle.
#pragma once

#include <Arduino.h>

class FastAccelStepper {
   public:
    void setDirectionPin(uint8_t, bool = true, uint16_t = 0) {}
    void setEnablePin(uint8_t, bool = true) {}
    void setAutoEnable(bool) {}
    void setDelayToEnable(uint32_t) {}
    void setDelayToDisable(uint16_t) {}

    bool enableOutputs() {
        _enabled = true;
        return true;
    }
    bool disableOutputs() {
        _enabled = false;
        return true;
    }

    int8_t setSpeedInHz(uint32_t hz) {
        _speedHz = hz;
        return 0;
    }
    int8_t setAcceleration(int32_t accel) {
        _accel = accel;
        return 0;
    }
    void applySpeedAcceleration() {}

    int8_t moveTo(int32_t position, bool = false) {
        _target = position;
        return 0;
    }
    void stopMove();
    void forceStop();

    bool isRunning() const;
    int32_t getCurrentPosition() const { return (int32_t)lround(_pos); }

    // --- simulation ---
    void simStep(double dtS);
    int simOwner = 0;  // numéro du PAMI, pour le journal

   private:
    double _pos = 0.0;
    double _v = 0.0;  // pas/s
    int32_t _target = 0;
    uint32_t _speedHz = 1;
    int32_t _accel = 1;
    bool _enabled = false;
    bool _warnedDisabled = false;
};

class FastAccelStepperEngine {
   public:
    void init() {}
    FastAccelStepper* stepperConnectToPin(uint8_t pin);
};

// Fait avancer tous les moteurs de dtS secondes.
void simTickSteppers(double dtS);
