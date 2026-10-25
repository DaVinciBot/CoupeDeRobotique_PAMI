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
// Avancer / reculer d'une distance (pause si obstacle devant en marche avant).
// maxMs > 0 : s'arrêter au bout de ce temps (pauses comprises), même si la
// distance n'est pas parcourue.
// ------------------------------------------------------------------
// Filtre « obstacle connu de la carte » : point du lidar (repère lidar, PAMI
// en `robot`) sur une muraille, une carrière ou un bord.
using KnownObstacleFilter = std::function<bool(const Pose& robot, float lx, float ly)>;

// Le lidar voit-il dans son couloir autre chose qu'un obstacle connu ?
// (sans filtre : n'importe quoi)
bool unknownObstacleAhead(const LidarGS2& lidar, const Pose& robot,
                          const KnownObstacleFilter& isKnown);

class Forward : public Action {
   public:
    Forward(DiffDrive& drive, const LidarGS2* lidar, float mm, uint32_t maxMs = 0)
        : _drive(drive), _lidar(lidar), _mm(mm), _maxMs(maxMs) {}
    // Arrêté par un obstacle connu (carrière, muraille) : terminer tout de
    // suite au lieu d'attendre qu'il « se libère ».
    Forward& finishOnKnownObstacle(KnownObstacleFilter isKnown) {
        _isKnown = isKnown;
        return *this;
    }
    // Ne pas terminer avant ce temps (arrêté s'il le faut) : laisse au voisin
    // le temps de s'éloigner avant que ce PAMI ne tourne.
    Forward& minDuration(uint32_t ms) {
        _minMs = ms;
        return *this;
    }
    void start() override;
    void update() override;
    void stop() override;
    bool isFinished() const override { return _finished; }
    const char* name() const override { return "Forward"; }

   private:
    DiffDrive& _drive;
    const LidarGS2* _lidar;
    float _mm;
    uint32_t _maxMs;
    KnownObstacleFilter _isKnown = nullptr;
    uint32_t _minMs = 0;
    uint32_t _startMs = 0;
    bool _timeUp = false;
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
        // Ignorer le lidar à moins de X mm de la cible (rempart, pierres
        // autour d'une douve) pour ne pas rester bloqué juste avant.
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

        // Contournement : bloqué par un obstacle depuis detourAfterMs, le
        // PAMI recule, se décale (à droite de préférence, sinon à gauche ;
        // sans place sur la carte, il recule encore, jusqu'à 3 fois), attend
        // un court instant au hasard, puis repart vers la cible.
        // Le hasard évite que deux PAMI face à face se rebloquent en faisant
        // exactement la même chose. 0 = jamais (attendre sur place).
        uint32_t detourAfterMs = 0;
        float detourBackMm = 60.0f;      // recul (sans lidar derrière : rester court)
        float detourSideMm = 120.0f;     // pas de côté
        float detourAngleRad = 0.873f;   // 50° : décalage de ~90 mm sur le côté
        uint32_t detourWaitMinMs = 200;  // attente avant de repartir
        uint32_t detourWaitMaxMs = 1000;
        // Tenir sa droite : la cible est décalée de X mm vers la droite du
        // trajet (calculé au départ). Deux PAMI qui se croisent sur le même
        // point de passage restent ainsi chacun de son côté.
        float keepRightMm = 0.0f;
        // Position libre sur la carte (ni muraille, ni carrière, ni bord) ?
        // Sert à choisir le côté du décalage. Absent : on suppose que oui.
        std::function<bool(const Pose& pose)> isPoseFree = nullptr;
        // Point vu par le lidar (repère lidar, PAMI en `robot`) qui tombe sur
        // un obstacle connu de la carte (muraille, carrière, bord) ? Ces
        // points sont ignorés pour l'arrêt : les trajets les évitent déjà, et
        // en courbe le lidar, qui regarde tout droit, les voit à tort.
        // Le lidar ne sert plus alors qu'à ce qui bouge (les robots).
        std::function<bool(const Pose& robot, float lx, float ly)> isKnownObstacle = nullptr;

        // Esquive : un obstacle qui VIENT VERS NOUS (sa distance diminue plus
        // vite que nous n'avançons) est évité tout de suite, sans s'arrêter,
        // par un S vers la droite (toujours la droite : l'autre s'écarte à
        // la sienne). Droite bloquée : arrêt, on le laisse passer.
        bool dodgeOncoming = false;

        // Trajectoire en courbe : rejoindre la cible par un arc (avancer et
        // tourner en même temps) au lieu de pivoter puis avancer, si l'écart
        // d'angle reste sous smoothMaxRad et que l'arc ne touche rien sur la
        // carte (isPoseFree). Sinon : pivot, comme avant.
        bool smooth = false;
        // Au-delà, l'arc ferait une grande boucle : pivoter est plus court.
        float smoothMaxRad = 0.52f;  // 30°
        // Point de passage : terminé à cette distance, SANS freiner ; le
        // GoTo suivant enchaîne directement vers le point d'après.
        float passThroughMm = 0.0f;

        // Contournement direct : un obstacle qui ne vient pas vers nous
        // (immobile, ou qui traverse) est contourné tout de suite, en passant
        // à côté de lui au plus près, SANS reculer. Ses côtés sont estimés à
        // partir des points du lidar. Droite de préférence, sinon gauche.
        // Le recul (detour) ne sert plus que s'il n'y a de place d'aucun côté.
        bool bypass = false;
        float bypassClearanceMm = 35.0f;  // marge entre le PAMI et l'obstacle
        float bypassBeyondMm = 250.0f;    // point de passage : au-delà de sa face avant
        float halfWidthMm = 47.5f;        // demi-largeur du PAMI
        float lidarOffsetMm = 60.0f;      // lidar devant l'axe des roues
        // Le lidar ne voit qu'à 30 cm : à 2 x 300 mm/s, il faut s'écarter de
        // ~55 mm chacun en moins de 140 mm parcourus -> arcs courts et serrés.
        float dodgeArcMm = 150.0f;       // longueur de chaque arc du S
        float dodgeAngleRad = 0.9f;      // ~52° : décalage total ~125 mm
        float oncomingMinMm = 6.0f;      // rapprochement propre de l'obstacle par trame lidar (2 de suite)
        uint16_t dodgeAbortMm = 100;     // trop près pendant l'esquive : pause

        // Suivre un PAMI qui avance dans le même sens : ne s'arrêter que s'il
        // ralentit (on se rapproche de plus de followClosingMmS) ou si l'on
        // est à moins de followMinMm (mesuré au lidar). Sans ça, on s'arrête
        // à la distance d'arrêt du lidar (22 cm) derrière lui.
        bool follow = false;
        uint16_t followMinMm = 130;
        float followClosingMmS = 80.0f;
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
    enum class Phase {
        Waiting,
        Turning,
        Driving,
        DetourBack,  // contournement : recul
        DetourTurn,  //                 rotation vers le côté choisi
        DetourSide,  //                 pas de côté
        DetourWait,  //                 attente avant de repartir
        DodgeOut,    // esquive : premier arc (on s'écarte)
        DodgeBack,   //          second arc (on se redresse)
        FinalTurn,
        Done
    };

    void planLeg();
    void finish();
    void checkRetarget();
    bool pollProvider(Pose& newTarget);
    bool steerToTarget(float maxSteerRad, bool checkMap);
    bool steerToTarget() { return steerToTarget(_opt.maxSteerRad, false); }
    bool obstacleAhead() const;
    void startDetour();
    bool startBypass();
    bool lineIsFree(const Pose& from, const Pose& to) const;
    void chooseDetourSide();
    void waitThenReplan();
    uint32_t randomBetween(uint32_t lo, uint32_t hi);
    void trackFront();
    bool oncomingObstacle() const { return _oncomingFrames >= 2; }
    bool startDodge();
    void updateDodge();

    DiffDrive& _drive;
    const LidarGS2* _lidar;
    Pose _target;
    Options _opt;
    Phase _phase = Phase::Done;
    bool _replanPending = false;
    uint32_t _startMs = 0;
    uint32_t _lastRetargetMs = 0;
    uint32_t _blockedSinceMs = 0;  // 0 = pas bloqué
    uint16_t _blockedFrontMm = 0xFFFF;  // distance de l'obstacle au début du blocage
    uint32_t _detourWaitUntilMs = 0;
    uint8_t _detourBackups = 0;  // reculs supplémentaires faute de place
    static constexpr uint8_t MAX_DETOUR_BACKUPS = 3;
    uint32_t _rng = 0;

    // Esquive
    uint32_t _lastLidarFrame = 0;
    uint16_t _prevFrontMm = 0xFFFF;
    uint8_t _oncomingFrames = 0;
    uint32_t _prevFrontMs = 0;
    float _closingMmS = 0.0f;  // vitesse à laquelle l'obstacle devant se rapproche
    Pose _prevPose;
    float _dodgeSide = 0.0f;  // -1 droite, +1 gauche
    float _dodgeStartHeading = 0.0f;

    // Contournement direct : cible provisoire à côté de l'obstacle
    bool _bypassing = false;
    Pose _savedTarget;
    float _savedStopWithinMm = 0.0f;
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
