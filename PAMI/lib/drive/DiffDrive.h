#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>

#include "Pose.h"

// Base roulante différentielle à deux moteurs pas à pas.
//
// Les pas sont générés en matériel par FastAccelStepper (RMT de l'ESP32-S3) :
// rien ne bloque, et les Serial.print ne ralentissent plus les moteurs.
// Convention : pas positifs = la roue avance, angle positif = sens trigo.
// L'odométrie est calculée à partir des pas réellement émis.
class DiffDrive {
   public:
    struct Config {
        uint8_t leftStepPin, leftDirPin, leftEnPin;
        bool leftDirHighIsForward;
        uint8_t rightStepPin, rightDirPin, rightEnPin;
        bool rightDirHighIsForward;
        float stepsPerRev;  // pas moteur x micropas
        float wheelDiameterMm;
        float wheelBaseMm;
        float speedMmS;
        float accelMmS2;
    };

    bool begin(const Config& cfg);

    // À appeler à chaque tour de loop() : odométrie + fin de mouvement.
    void update();

    // Mouvements relatifs, non bloquants. Remplacent le mouvement en cours.
    void forward(float mm);
    void turn(float rad);
    // Arc de cercle : `lengthMm` parcourus par le centre des roues en
    // tournant de `rotationRad`. Les vitesses des deux roues sont dans le même
    // rapport que leurs distances, pour qu'elles finissent ensemble.
    void arc(float lengthMm, float rotationRad);

    void stop();           // décélère et abandonne la consigne
    void emergencyStop();  // arrêt immédiat (pertes de pas possibles)
    void pause();          // décélère en gardant la consigne
    void resume();         // repart vers la consigne

    bool isBusy() const;    // consigne en cours (même en pause) ou en mouvement
    bool isMoving() const;  // les moteurs tournent
    bool isPaused() const { return _paused; }
    bool powered() const { return _powered; }  // moteurs alimentés

    Pose pose() const { return _pose; }
    void setPose(const Pose& p);

    void setSpeed(float mmS);
    void setAccel(float mmS2);
    float speed() const { return _cfg.speedMmS; }
    float accel() const { return _cfg.accelMmS2; }

    void setWheelBase(float mm) { _cfg.wheelBaseMm = mm; }
    float wheelBase() const { return _cfg.wheelBaseMm; }

    void enableOutputs(bool enable);

   private:
    static constexpr uint32_t ENABLE_DELAY_US = 1000;   // EN actif -> premier pas
    static constexpr uint32_t POWER_OFF_DELAY_MS = 500;  // arrêt -> moteurs coupés

    void startRelative(int32_t dLeft, int32_t dRight);
    void powerOn();
    void applySpeed();

    FastAccelStepperEngine _engine;
    FastAccelStepper* _left = nullptr;
    FastAccelStepper* _right = nullptr;
    Config _cfg{};
    float _stepsPerMm = 1.0f;

    bool _hasTarget = false;
    bool _paused = false;
    int32_t _targetLeft = 0;
    int32_t _targetRight = 0;
    uint32_t _moveStartMs = 0;
    float _scaleLeft = 1.0f;  // part de la vitesse max donnée à chaque roue
    float _scaleRight = 1.0f;
    bool _powered = false;
    uint32_t _idleSinceMs = 0;

    int32_t _lastLeft = 0;
    int32_t _lastRight = 0;
    Pose _pose;
};
