#include "Actions.h"

// ------------------------------------------------------------------
// Forward
// ------------------------------------------------------------------
void Forward::start() {
    _finished = false;
    _drive.forward(_mm);
}

void Forward::update() {
    if (_finished) return;

    // Le lidar ne voit que devant 
    if (_mm > 0 && _lidar != nullptr && _lidar->healthy()) {
        bool obstacle = _lidar->obstacle();
        if (obstacle && !_drive.isPaused()) _drive.pause();
        if (!obstacle && _drive.isPaused()) _drive.resume();
    }

    if (!_drive.isBusy()) _finished = true;
}

void Forward::stop() {
    _drive.stop();
    _finished = true;
}

// ------------------------------------------------------------------
// GoTo
// ------------------------------------------------------------------
void GoTo::start() {
    _startMs = millis();
    _lastRetargetMs = _startMs;
    _replanPending = false;
    if (_opt.waitForTarget && _opt.provider) {
        Serial.println("[GoTo] attente d'une cible");
        _lastRetargetMs = _startMs - _opt.retargetPeriodMs;  // chercher tout de suite
        _phase = Phase::Waiting;
        return;
    }
    Pose p = _drive.pose();
    Serial.printf("[GoTo] (%.0f, %.0f) -> (%.0f, %.0f)\n", p.x, p.y, _target.x,
                  _target.y);
    planLeg();
}

void GoTo::planLeg() {
    Pose p = _drive.pose();
    if (distanceMm(p, _target) <= max(2.0f, _opt.stopWithinMm)) {
        finish();
        return;
    }
    float dTheta = wrapPi(headingTo(p, _target) - p.theta);
    _drive.turn(dTheta);
    _phase = Phase::Turning;
}

// Arrivé : orientation finale si demandée, sinon terminé.
void GoTo::finish() {
    if (_opt.finalHeading) {
        _drive.turn(wrapPi(_target.theta - _drive.pose().theta));
        _phase = Phase::FinalTurn;
    } else {
        _phase = Phase::Done;
    }
}

bool GoTo::obstacleAhead() const {
    return _lidar != nullptr && _lidar->healthy() && _lidar->obstacle();
}

// Interroge `provider` au plus une fois par retargetPeriodMs.
bool GoTo::pollProvider(Pose& newTarget) {
    if (!_opt.provider || millis() - _lastRetargetMs < _opt.retargetPeriodMs) {
        return false;
    }
    _lastRetargetMs = millis();
    return _opt.provider(_drive.pose(), newTarget);
}

void GoTo::checkRetarget() {
    Pose newTarget;
    if (!pollProvider(newTarget)) return;
    if (distanceMm(newTarget, _target) < _opt.retargetMinMm) return;

    _target = newTarget;
    if (_phase == Phase::Driving && !_drive.isPaused() && steerToTarget()) {
        return;  // correction en roulant
    }
    Serial.printf("[GoTo] nouvelle cible (%.0f, %.0f) : reorientation\n",
                  _target.x, _target.y);
    _drive.stop();
    _replanPending = true;
}

// Arc de cercle qui part dans la direction actuelle et passe par la cible.
// Renvoie false si la cible est trop sur le côté ou déjà atteinte.
bool GoTo::steerToTarget() {
    Pose p = _drive.pose();
    float dx = _target.x - p.x, dy = _target.y - p.y;
    float c = cosf(p.theta), s = sinf(p.theta);
    float ahead = c * dx + s * dy;  // repère robot
    float left = -s * dx + c * dy;
    float alpha = atan2f(left, ahead);  // écart d'angle vers la cible
    if (fabsf(alpha) > _opt.maxSteerRad) return false;

    float dist = hypotf(ahead, left);
    float remaining = dist - _opt.stopWithinMm;
    if (remaining <= 0.0f) return false;

    // Arc tangent à la direction actuelle jusqu'à la cible : il tourne de
    // 2 x alpha et mesure dist x alpha / sin(alpha). On n'en parcourt que la
    // part qui s'arrête à stopWithinMm de la cible.
    float arcLen = fabsf(alpha) < 1e-3f ? dist : dist * alpha / sinf(alpha);
    float part = remaining / dist;
    _drive.arc(arcLen * part, 2.0f * alpha * part);
    return true;
}

void GoTo::update() {
    if (_phase == Phase::Done) return;

    if (_opt.timeoutMs > 0 && millis() - _startMs > _opt.timeoutMs) {
        Serial.println("[GoTo] timeout");
        stop();
        return;
    }

    if (_phase == Phase::Waiting) {
        Pose newTarget;
        if (pollProvider(newTarget)) {
            _target = newTarget;
            Pose p = _drive.pose();
            Serial.printf("[GoTo] cible (%.0f, %.0f) -> (%.0f, %.0f)\n", p.x, p.y,
                          _target.x, _target.y);
            planLeg();
        }
        return;
    }

    if (_phase != Phase::FinalTurn) checkRetarget();

    if (_replanPending) {
        if (_drive.isMoving()) return;  // attendre la fin du freinage
        _replanPending = false;
        planLeg();
        return;
    }

    switch (_phase) {
        case Phase::Turning:
            if (!_drive.isBusy()) {
                float d = distanceMm(_drive.pose(), _target);
                _drive.forward(max(0.0f, d - _opt.stopWithinMm));
                _phase = Phase::Driving;
            }
            break;

        case Phase::Driving: {
            float d = distanceMm(_drive.pose(), _target);
            if (_opt.stopWithinMm > 0.0f && d <= _opt.stopWithinMm) {
                Serial.println("[GoTo] cible atteinte");
                _drive.stop();
                finish();
                return;
            }
            bool obstacle = obstacleAhead() && d >= _opt.ignoreObstacleWithinMm;
            if (_opt.pauseOnObstacle) {
                if (obstacle && !_drive.isPaused()) _drive.pause();
                if (!obstacle && _drive.isPaused()) _drive.resume();
            }
            if (!_drive.isBusy()) finish();
            break;
        }

        case Phase::FinalTurn:
            if (!_drive.isBusy()) _phase = Phase::Done;
            break;

        case Phase::Waiting:
        case Phase::Done:
            break;
    }
}

void GoTo::stop() {
    _drive.stop();
    _phase = Phase::Done;
}

// ------------------------------------------------------------------
// ServoSweep
// ------------------------------------------------------------------
void ServoSweep::start() {
    _finished = false;
    _atA = true;
    _servo.write(_a);
    _startMs = _lastToggleMs = millis();
}

void ServoSweep::update() {
    if (_finished) return;
    uint32_t now = millis();
    if (_durationMs > 0 && now - _startMs >= _durationMs) {
        _finished = true;
        return;
    }
    if (now - _lastToggleMs >= _halfPeriodMs) {
        _atA = !_atA;
        _servo.write(_atA ? _a : _b);
        _lastToggleMs = now;
    }
}
