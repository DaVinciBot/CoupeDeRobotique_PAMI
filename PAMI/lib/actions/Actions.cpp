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
    Pose p = _drive.pose();
    Serial.printf("[GoTo] (%.0f, %.0f) -> (%.0f, %.0f)\n", p.x, p.y, _target.x,
                  _target.y);
    planLeg();
}

void GoTo::planLeg() {
    Pose p = _drive.pose();
    if (distanceMm(p, _target) < 2.0f) {
        if (_opt.finalHeading) {
            _drive.turn(wrapPi(_target.theta - p.theta));
            _phase = Phase::FinalTurn;
        } else {
            _phase = Phase::Done;
        }
        return;
    }
    float dTheta = wrapPi(headingTo(p, _target) - p.theta);
    _drive.turn(dTheta);
    _phase = Phase::Turning;
}

bool GoTo::obstacleAhead() const {
    return _lidar != nullptr && _lidar->healthy() && _lidar->obstacle();
}

void GoTo::checkRetarget() {
    if (!_opt.provider || millis() - _lastRetargetMs < _opt.retargetPeriodMs) {
        return;
    }
    _lastRetargetMs = millis();

    Pose newTarget;
    if (!_opt.provider(_drive.pose(), newTarget)) return;
    if (distanceMm(newTarget, _target) < _opt.retargetMinMm) return;

    Serial.printf("[GoTo] nouvelle cible (%.0f, %.0f)\n", newTarget.x,
                  newTarget.y);
    _target = newTarget;
    _drive.stop();
    _replanPending = true;
}

void GoTo::update() {
    if (_phase == Phase::Done) return;

    if (millis() - _startMs > _opt.timeoutMs) {
        Serial.println("[GoTo] timeout");
        stop();
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
                _drive.forward(distanceMm(_drive.pose(), _target));
                _phase = Phase::Driving;
            }
            break;

        case Phase::Driving: {
            bool obstacle = obstacleAhead() &&
                            distanceMm(_drive.pose(), _target) >= _opt.ignoreObstacleWithinMm;
            if (_opt.finishOnObstacle && obstacle) {
                Serial.println("[GoTo] contact obstacle -> fin");
                _drive.stop();
                _phase = Phase::Done;
                return;
            }
            if (_opt.pauseOnObstacle) {
                if (obstacle && !_drive.isPaused()) _drive.pause();
                if (!obstacle && _drive.isPaused()) _drive.resume();
            }
            if (!_drive.isBusy()) {
                if (_opt.finalHeading) {
                    _drive.turn(wrapPi(_target.theta - _drive.pose().theta));
                    _phase = Phase::FinalTurn;
                } else {
                    _phase = Phase::Done;
                }
            }
            break;
        }

        case Phase::FinalTurn:
            if (!_drive.isBusy()) _phase = Phase::Done;
            break;

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
