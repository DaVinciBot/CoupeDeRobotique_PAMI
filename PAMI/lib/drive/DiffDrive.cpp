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
    // A4988 : EN actif à l'état bas. Moteurs alimentés seulement pendant un
    // mouvement, pause comprise : sinon ils chauffent et vident la batterie
    // pendant les 85 s d'attente (couper EN ne fait pas perdre la position du
    // micropas). Géré ici et pas par l'auto-enable de FastAccelStepper, qui
    // couperait les moteurs pendant une pause : un PAMI poussé devant un
    // obstacle perdrait alors sa position.
    _left->setEnablePin(_cfg.leftEnPin, true);
    _right->setEnablePin(_cfg.rightEnPin, true);
    enableOutputs(false);

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

    // Coupure des moteurs après un court délai sans consigne (une pause
    // garde la consigne, donc les moteurs restent alimentés).
    bool idle = !_hasTarget && !isMoving();
    if (!idle) {
        _idleSinceMs = millis();
    } else if (_powered && millis() - _idleSinceMs > POWER_OFF_DELAY_MS) {
        enableOutputs(false);
    }
}

void DiffDrive::powerOn() {
    if (_powered) return;
    enableOutputs(true);
    delayMicroseconds(ENABLE_DELAY_US);  // seulement au départ d'un mouvement
}

void DiffDrive::forward(float mm) {
    arc(mm, 0.0f);
}

void DiffDrive::turn(float rad) {
    arc(0.0f, rad);
}

void DiffDrive::arc(float lengthMm, float rotationRad) {
    float halfMm = rotationRad * 0.5f * _cfg.wheelBaseMm;
    startRelative(lroundf((lengthMm - halfMm) * _stepsPerMm),
                  lroundf((lengthMm + halfMm) * _stepsPerMm));
}

void DiffDrive::startRelative(int32_t dLeft, int32_t dRight) {
    // Vitesse et accélération de chaque roue proportionnelles à sa distance :
    // partant de l'arrêt, les deux roues suivent la même rampe dans le temps
    // et l'arc est respecté. Si un mouvement est en cours, il est remplacé
    // (FastAccelStepper enchaîne les rampes) : la trajectoire s'écarte un peu
    // de l'arc pendant la transition, mais l'odométrie compte les pas émis.
    int32_t most = max(abs(dLeft), abs(dRight));
    _scaleLeft = most > 0 ? (float)abs(dLeft) / most : 1.0f;
    _scaleRight = most > 0 ? (float)abs(dRight) / most : 1.0f;
    applySpeed();

    _targetLeft = _left->getCurrentPosition() + dLeft;
    _targetRight = _right->getCurrentPosition() + dRight;
    _hasTarget = true;
    _paused = false;
    _moveStartMs = millis();
    powerOn();
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
    powerOn();
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
    float hz = _cfg.speedMmS * _stepsPerMm;
    float accel = _cfg.accelMmS2 * _stepsPerMm;
    auto apply = [hz, accel](FastAccelStepper* s, float scale) {
        // FastAccelStepper refuse 0 : au moins 1 pas/s
        s->setSpeedInHz((uint32_t)max(1L, lroundf(hz * scale)));
        s->setAcceleration((int32_t)max(1L, lroundf(accel * scale)));
        if (s->isRunning()) s->applySpeedAcceleration();
    };
    apply(_left, _scaleLeft);
    apply(_right, _scaleRight);
}

void DiffDrive::enableOutputs(bool enable) {
    if (enable) {
        _left->enableOutputs();
        _right->enableOutputs();
    } else {
        _left->disableOutputs();
        _right->disableOutputs();
    }
    _powered = enable;
}
