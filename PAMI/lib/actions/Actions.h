#pragma once

#include <Arduino.h>

#include <functional>

#include "Action.h"
#include "DiffDrive.h"
#include "LidarGS2.h"
#include "MatchClock.h"
#include "ServoPwm.h"

// ------------------------------------------------------------------
// Attendre une durée
// ------------------------------------------------------------------
class Wait : public Action {
   public:
    explicit Wait(uint32_t ms) : _ms(ms) {}
    void start() override { _startMs = millis(); _finished = false; }
    void update() override { if (millis() - _startMs >= _ms) _finished = true; }
    void stop() override { _finished = true; }
    bool isFinished() const override { return _finished; }
    const char* name() const override { return "Wait"; }

   private:
    uint32_t _ms;
    uint32_t _startMs = 0;
    bool _finished = false;
};

// ------------------------------------------------------------------
// Attendre un instant du match (ex. début de la phase d'attaque)
// ------------------------------------------------------------------
class WaitMatchTime : public Action {
   public:
    WaitMatchTime(const MatchClock& clock, uint32_t matchMs)
        : _clock(clock), _matchMs(matchMs) {}
    void start() override { _finished = false; }
    void update() override {
        if (_clock.started() && _clock.elapsedMs() >= _matchMs) _finished = true;
    }
    void stop() override { _finished = true; }
    bool isFinished() const override { return _finished; }
    const char* name() const override { return "WaitMatchTime"; }

   private:
    const MatchClock& _clock;
    uint32_t _matchMs;
    bool _finished = false;
};

// ------------------------------------------------------------------
// Avancer / reculer d'une distance (pause si obstacle devant en marche avant)
// ------------------------------------------------------------------
class Forward : public Action {
   public:
    Forward(DiffDrive& drive, const LidarGS2* lidar, float mm)
        : _drive(drive), _lidar(lidar), _mm(mm) {}
    void start() override;
    void update() override;
    void stop() override;
    bool isFinished() const override { return _finished; }
    const char* name() const override { return "Forward"; }

   private:
    DiffDrive& _drive;
    const LidarGS2* _lidar;
    float _mm;
    bool _finished = false;
};

// ------------------------------------------------------------------
// Tourner d'un angle relatif (radians, sens trigo)
// ------------------------------------------------------------------
class Turn : public Action {
   public:
    Turn(DiffDrive& drive, float rad) : _drive(drive), _rad(rad) {}
    void start() override { _finished = false; _drive.turn(_rad); }
    void update() override { if (!_drive.isBusy()) _finished = true; }
    void stop() override { _drive.stop(); _finished = true; }
    bool isFinished() const override { return _finished; }
    const char* name() const override { return "Turn"; }

   private:
    DiffDrive& _drive;
    float _rad;
    bool _finished = false;
};

// ------------------------------------------------------------------
// Aller à un point de la table : tourner vers lui, puis avancer.
// La cible peut être mise à jour pendant le trajet (ex. PAMI adverse vu par
// le lidar) : si elle reste à peu près devant, le PAMI corrige sa direction
// en roulant ; sinon il s'arrête et se réoriente.
// ------------------------------------------------------------------
class GoTo : public Action {
   public:
    // Renvoie true et remplit `target` si une (nouvelle) cible est disponible.
    using TargetProvider = std::function<bool(const Pose& current, Pose& target)>;

    struct Options {
        bool pauseOnObstacle = true;  // évitement : attendre que ça se libère
        bool finalHeading = false;    // s'orienter selon target.theta à la fin
        // Ignorer le lidar à moins de X mm de la cible (murs du château,
        // pierres autour d'une douve) pour ne pas rester bloqué juste avant.
        float ignoreObstacleWithinMm = 0.0f;
        // S'arrêter à X mm de la cible au lieu d'aller dessus (0 = dessus).
        float stopWithinMm = 0.0f;
        uint32_t timeoutMs = 15000;  // 0 = pas de limite (fin du match)

        TargetProvider provider = nullptr;
        // Ne pas partir tant que `provider` n'a pas donné de cible.
        bool waitForTarget = false;
        uint32_t retargetPeriodMs = 500;
        float retargetMinMm = 50.0f;
        // Au-delà de cet écart d'angle, s'arrêter et se réorienter au lieu de
        // corriger en roulant.
        float maxSteerRad = 0.785f;  // 45°
    };

    GoTo(DiffDrive& drive, const LidarGS2* lidar, const Pose& target)
        : GoTo(drive, lidar, target, Options()) {}
    GoTo(DiffDrive& drive, const LidarGS2* lidar, const Pose& target,
         const Options& options)
        : _drive(drive), _lidar(lidar), _target(target), _opt(options) {}

    void start() override;
    void update() override;
    void stop() override;
    bool isFinished() const override { return _phase == Phase::Done; }
    const char* name() const override { return "GoTo"; }

   private:
    enum class Phase { Waiting, Turning, Driving, FinalTurn, Done };

    void planLeg();
    void finish();
    void checkRetarget();
    bool pollProvider(Pose& newTarget);
    bool steerToTarget();
    bool obstacleAhead() const;

    DiffDrive& _drive;
    const LidarGS2* _lidar;
    Pose _target;
    Options _opt;
    Phase _phase = Phase::Done;
    bool _replanPending = false;
    uint32_t _startMs = 0;
    uint32_t _lastRetargetMs = 0;
};

// ------------------------------------------------------------------
// Mettre un servo à un angle et attendre qu'il y soit
// ------------------------------------------------------------------
class ServoMove : public Action {
   public:
    ServoMove(ServoPwm& servo, uint8_t angleDeg, uint32_t settleMs = 400)
        : _servo(servo), _angle(angleDeg), _settleMs(settleMs) {}
    void start() override { _servo.write(_angle); _startMs = millis(); _finished = false; }
    void update() override { if (millis() - _startMs >= _settleMs) _finished = true; }
    void stop() override { _finished = true; }  // le servo garde sa position
    bool isFinished() const override { return _finished; }
    const char* name() const override { return "ServoMove"; }

   private:
    ServoPwm& _servo;
    uint8_t _angle;
    uint32_t _settleMs;
    uint32_t _startMs = 0;
    bool _finished = false;
};

// ------------------------------------------------------------------
// Faire osciller un servo (durationMs = 0 : jusqu'à stop())
// ------------------------------------------------------------------
class ServoSweep : public Action {
   public:
    ServoSweep(ServoPwm& servo, uint8_t a, uint8_t b, uint32_t halfPeriodMs,
               uint32_t durationMs)
        : _servo(servo), _a(a), _b(b), _halfPeriodMs(halfPeriodMs),
          _durationMs(durationMs) {}
    void start() override;
    void update() override;
    void stop() override { _finished = true; }
    bool isFinished() const override { return _finished; }
    const char* name() const override { return "ServoSweep"; }

   private:
    ServoPwm& _servo;
    uint8_t _a, _b;
    uint32_t _halfPeriodMs, _durationMs;
    uint32_t _startMs = 0, _lastToggleMs = 0;
    bool _atA = true;
    bool _finished = false;
};
