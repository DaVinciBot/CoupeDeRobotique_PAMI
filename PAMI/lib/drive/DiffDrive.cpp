#include "DiffDrive.h"

bool DiffDrive::begin(const Config& cfg) {
    _cfg = cfg;
    _stepsPerMm = _cfg.stepsPerRev / (_cfg.wheelDiameterMm * (float)M_PI);

    _engine.init();
    _left = _engine.stepperConnectToPin(_cfg.leftStepPin);
    _right = _engine.stepperConnectToPin(_cfg.rightStepPin);
    if (_left == nullptr || _right == nullptr) {
        Serial.println("[Drive] ERREUR : impossible d'attacher les moteurs");
        return false;
    }

    // dirHighCountsUp : position qui augmente = roue qui avance
    _left->setDirectionPin(_cfg.leftDirPin, _cfg.leftDirHighIsForward);
    _right->setDirectionPin(_cfg.rightDirPin, _cfg.rightDirHighIsForward);
    // A4988 : EN actif à l'état bas. Moteurs alimentés seulement pendant les
    // mouvements : sinon ils chauffent et vident la batterie pendant les 85 s
    // d'attente (couper EN ne fait pas perdre la position du micropas).
    for (FastAccelStepper* s : {_left, _right}) {
        s->setEnablePin(s == _left ? _cfg.leftEnPin : _cfg.rightEnPin, true);
        s->setAutoEnable(true);
        s->setDelayToEnable(1000);  // µs
        s->setDelayToDisable(500);  // ms
    }

    applySpeed();

    _lastLeft = _left->getCurrentPosition();
    _lastRight = _right->getCurrentPosition();
    return true;
}

void DiffDrive::update() {
    if (_left == nullptr) return;

    // Odométrie à partir des pas réellement émis
    int32_t l = _left->getCurrentPosition();
    int32_t r = _right->getCurrentPosition();
    float dl = (l - _lastLeft) / _stepsPerMm;
    float dr = (r - _lastRight) / _stepsPerMm;
    _lastLeft = l;
    _lastRight = r;

    if (dl != 0.0f || dr != 0.0f) {
        float ds = 0.5f * (dl + dr);
        float dTheta = (dr - dl) / _cfg.wheelBaseMm;
        float midTheta = _pose.theta + 0.5f * dTheta;
        _pose.x += ds * cosf(midTheta);
        _pose.y += ds * sinf(midTheta);
        _pose.theta = wrapPi(_pose.theta + dTheta);
    }

    // Fin de mouvement : les deux moteurs arrêtés (après un court délai pour
    // laisser FastAccelStepper démarrer la rampe).
    if (_hasTarget && !_paused && !isMoving() &&
        (millis() - _moveStartMs > 50 ||
         (l == _targetLeft && r == _targetRight))) {
        _hasTarget = false;
    }
}

void DiffDrive::forward(float mm) {
    int32_t steps = lroundf(mm * _stepsPerMm);
    startRelative(steps, steps);
}

void DiffDrive::turn(float rad) {
    float arcMm = rad * 0.5f * _cfg.wheelBaseMm;
    int32_t steps = lroundf(arcMm * _stepsPerMm);
    startRelative(-steps, steps);
}

void DiffDrive::startRelative(int32_t dLeft, int32_t dRight) {
    // Repart de la position actuelle (si un mouvement est en cours, il est
    // remplacé ; FastAccelStepper enchaîne les rampes).
    _targetLeft = _left->getCurrentPosition() + dLeft;
    _targetRight = _right->getCurrentPosition() + dRight;
    _hasTarget = true;
    _paused = false;
    _moveStartMs = millis();
    _left->moveTo(_targetLeft);
    _right->moveTo(_targetRight);
}

void DiffDrive::stop() {
    _hasTarget = false;
    _paused = false;
    _left->stopMove();
    _right->stopMove();
}

void DiffDrive::emergencyStop() {
    _hasTarget = false;
    _paused = false;
    _left->forceStop();
    _right->forceStop();
}

void DiffDrive::pause() {
    if (!_hasTarget || _paused) return;
    _paused = true;
    _left->stopMove();
    _right->stopMove();
}

void DiffDrive::resume() {
    if (!_paused) return;
    _paused = false;
    _moveStartMs = millis();
    _left->moveTo(_targetLeft);
    _right->moveTo(_targetRight);
}

bool DiffDrive::isBusy() const {
    return _hasTarget || isMoving();
}

bool DiffDrive::isMoving() const {
    return _left->isRunning() || _right->isRunning();
}

void DiffDrive::setPose(const Pose& p) {
    _pose = p;
    _lastLeft = _left->getCurrentPosition();
    _lastRight = _right->getCurrentPosition();
}

void DiffDrive::setSpeed(float mmS) {
    _cfg.speedMmS = mmS;
    applySpeed();
}

void DiffDrive::setAccel(float mmS2) {
    _cfg.accelMmS2 = mmS2;
    applySpeed();
}

void DiffDrive::applySpeed() {
    uint32_t hz = (uint32_t)lroundf(_cfg.speedMmS * _stepsPerMm);
    int32_t accel = lroundf(_cfg.accelMmS2 * _stepsPerMm);
    for (FastAccelStepper* s : {_left, _right}) {
        s->setSpeedInHz(hz);
        s->setAcceleration(accel);
        if (s->isRunning()) s->applySpeedAcceleration();
    }
}

void DiffDrive::enableOutputs(bool enable) {
    if (enable) {
        _left->enableOutputs();
        _right->enableOutputs();
    } else {
        _left->disableOutputs();
        _right->disableOutputs();
    }
}
