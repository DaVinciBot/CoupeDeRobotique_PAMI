#include "Actions.h"

// ------------------------------------------------------------------
// Forward
// ------------------------------------------------------------------
void Forward::start() {
    _finished = false;
    _timeUp = false;
    _startMs = millis();
    _drive.forward(_mm);
}

void Forward::update() {
    if (_finished) return;

    // Temps écoulé : freiner, puis terminer une fois arrêté
    if (_maxMs > 0 && !_timeUp && millis() - _startMs >= _maxMs) {
        _timeUp = true;
        _drive.stop();
    }
    if (_timeUp) {
        if (!_drive.isMoving() && millis() - _startMs >= _minMs) _finished = true;
        return;
    }

    // Le lidar ne voit que devant
    if (_mm > 0 && _lidar != nullptr && _lidar->healthy()) {
        bool obstacle = _lidar->obstacle();
        // Ce n'est qu'un obstacle connu (carrière...) : il ne bougera pas,
        // inutile d'attendre, on s'arrête là et on passe à la suite.
        if (obstacle && _isKnown && !unknownObstacleAhead(*_lidar, _drive.pose(), _isKnown)) {
            _timeUp = true;
            _drive.stop();
            return;
        }
        if (obstacle && !_drive.isPaused()) _drive.pause();
        if (!obstacle && _drive.isPaused()) _drive.resume();
    }

    if (!_drive.isBusy() && millis() - _startMs >= _minMs) _finished = true;
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
    _blockedSinceMs = 0;
    // Graine du tirage au hasard : différente d'un PAMI à l'autre
    if (_rng == 0) _rng = ((uint32_t)(uintptr_t)this ^ (_startMs * 2654435761u)) | 1u;
    if (_opt.keepRightMm != 0.0f) {
        Pose p = _drive.pose();
        float heading = headingTo(p, _target);
        // droite du trajet = direction tournée de -90°
        _target.x += _opt.keepRightMm * sinf(heading);
        _target.y -= _opt.keepRightMm * cosf(heading);
        _opt.keepRightMm = 0.0f;  // une seule fois, même si l'action est relancée
    }
    if (_opt.waitForTarget && _opt.provider) {
        Serial.println("[GoTo] attente d'une cible");
        _lastRetargetMs = _startMs - _opt.retargetPeriodMs;  // chercher tout de suite
        _phase = Phase::Waiting;
        return;
    }
    Pose p = _drive.pose();
    Serial.printf("[GoTo] (%.0f, %.0f) -> (%.0f, %.0f)\n", p.x, p.y, _target.x,
                  _target.y);
    // En courbe si possible (le PAMI peut être encore en mouvement : l'arc
    // remplace le mouvement en cours sans s'arrêter), sinon pivot.
    if (_opt.smooth && distanceMm(p, _target) > max(2.0f, _opt.stopWithinMm) &&
        steerToTarget(_opt.smoothMaxRad, true)) {
        _phase = Phase::Driving;
        _prevFrontMm = 0xFFFF;
        return;
    }
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

bool unknownObstacleAhead(const LidarGS2& lidar, const Pose& robot,
                          const KnownObstacleFilter& isKnown) {
    if (!isKnown) return true;
    LidarPoint pts[LidarGS2::POINT_COUNT];
    lidar.copyPoints(pts);
    const float half = lidar.corridorHalfWidth();
    const float depth = lidar.obstacleThreshold();
    for (const LidarPoint& lp : pts) {
        if (!lp.valid || !lp.confirmed) continue;
        if (fabsf(lp.y) > half || lp.x >= depth) continue;
        if (!isKnown(robot, lp.x, lp.y)) return true;
    }
    return false;
}

bool GoTo::obstacleAhead() const {
    if (_lidar == nullptr || !_lidar->healthy() || !_lidar->obstacle()) return false;
    return unknownObstacleAhead(*_lidar, _drive.pose(), _opt.isKnownObstacle);
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
static Pose afterArc(const Pose& from, float length, float rotation);

bool GoTo::steerToTarget(float maxSteerRad, bool checkMap) {
    Pose p = _drive.pose();
    float dx = _target.x - p.x, dy = _target.y - p.y;
    float c = cosf(p.theta), s = sinf(p.theta);
    float ahead = c * dx + s * dy;  // repère robot
    float left = -s * dx + c * dy;
    float alpha = atan2f(left, ahead);  // écart d'angle vers la cible
    if (fabsf(alpha) > maxSteerRad) return false;

    float dist = hypotf(ahead, left);
    float remaining = dist - _opt.stopWithinMm;
    if (remaining <= 0.0f) return false;

    // Arc tangent à la direction actuelle jusqu'à la cible : il tourne de
    // 2 x alpha et mesure dist x alpha / sin(alpha). On n'en parcourt que la
    // part qui s'arrête à stopWithinMm de la cible.
    float arcLen = fabsf(alpha) < 1e-3f ? dist : dist * alpha / sinf(alpha);
    float part = remaining / dist;
    float length = arcLen * part, rotation = 2.0f * alpha * part;

    // L'arc s'écarte de la ligne droite : vérifier qu'il ne frôle rien
    if (checkMap && _opt.isPoseFree) {
        for (int k = 1; k <= 6; ++k) {
            float f = k / 6.0f;
            if (!_opt.isPoseFree(afterArc(p, length * f, rotation * f))) return false;
        }
    }
    _drive.arc(length, rotation);
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

    if (_phase == Phase::Turning || _phase == Phase::Driving) checkRetarget();

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
                _prevFrontMm = 0xFFFF;  // repartir de zéro pour l'esquive
            }
            break;

        case Phase::Driving: {
            float d = distanceMm(_drive.pose(), _target);
            // Point de contournement atteint : reprendre vers la vraie cible
            if (_bypassing && (d <= 60.0f || !_drive.isBusy())) {
                _bypassing = false;
                _target = _savedTarget;
                _opt.stopWithinMm = _savedStopWithinMm;
                _blockedSinceMs = 0;
                _prevFrontMm = 0xFFFF;
                if (!steerToTarget(_opt.smoothMaxRad, true)) planLeg();
                return;
            }
            if (_opt.stopWithinMm > 0.0f && d <= _opt.stopWithinMm) {
                Serial.println("[GoTo] cible atteinte");
                _drive.stop();
                finish();
                return;
            }
            trackFront();
            // Un PAMI vient vers nous : l'esquiver tout de suite, sans s'arrêter
            if (_opt.dodgeOncoming && d >= _opt.ignoreObstacleWithinMm && oncomingObstacle() &&
                startDodge()) {
                return;
            }
            bool obstacle = obstacleAhead() && d >= _opt.ignoreObstacleWithinMm;
            // Suivi : un PAMI devant qui avance aussi vite que nous n'est pas
            // une raison de s'arrêter, tant qu'on reste à followMinMm.
            if (obstacle && _opt.follow && !_drive.isPaused() && _lidar != nullptr &&
                _lidar->frontMinMm() >= _opt.followMinMm &&
                _closingMmS < _opt.followClosingMmS) {
                obstacle = false;
            }
            if (_opt.pauseOnObstacle) {
                if (obstacle && !_drive.isPaused()) _drive.pause();
                if (!obstacle && _drive.isPaused()) _drive.resume();

                // Bloqué trop longtemps par un obstacle qui ne s'éloigne pas :
                // contourner. Un obstacle qui s'éloigne est un PAMI qu'on suit :
                // on attend qu'il avance.
                uint16_t front = _lidar != nullptr ? _lidar->frontMinMm() : 0xFFFF;
                if (!obstacle) {
                    _blockedSinceMs = 0;
                } else if (_blockedSinceMs == 0) {
                    _blockedSinceMs = millis();
                    _blockedFrontMm = front;
                } else if (front != 0xFFFF && front > _blockedFrontMm + 10) {
                    _blockedSinceMs = millis();  // il s'éloigne : on repart de zéro
                    _blockedFrontMm = front;
                } else if (_opt.detourAfterMs > 0 &&
                           millis() - _blockedSinceMs >= _opt.detourAfterMs) {
                    // Passer à côté si possible ; reculer seulement sinon
                    if (!(_opt.bypass && startBypass())) startDetour();
                    return;
                }
            }
            // Point de passage : enchaîner sur le suivant sans freiner
            if (_opt.passThroughMm > 0.0f && d <= _opt.passThroughMm && !_drive.isPaused()) {
                _phase = Phase::Done;
                return;
            }
            if (!_drive.isBusy()) finish();
            break;
        }

        case Phase::DodgeOut:
        case Phase::DodgeBack:
            updateDodge();
            break;

        case Phase::DetourBack:
            if (!_drive.isBusy()) chooseDetourSide();
            break;

        case Phase::DetourTurn:
            if (!_drive.isBusy()) {
                _drive.forward(_opt.detourSideMm);
                _phase = Phase::DetourSide;
            }
            break;

        case Phase::DetourSide:
            if (obstacleAhead()) {  // pas de place : on s'arrête là
                _drive.stop();
                waitThenReplan();
            } else if (!_drive.isBusy()) {
                waitThenReplan();
            }
            break;

        case Phase::DetourWait:
            if (!_drive.isBusy() && (int32_t)(millis() - _detourWaitUntilMs) >= 0) {
                _blockedSinceMs = 0;
                planLeg();
            }
            break;

        case Phase::FinalTurn:
            if (!_drive.isBusy()) _phase = Phase::Done;
            break;

        case Phase::Waiting:
        case Phase::Done:
            break;
    }
}

// ------------------------------------------------------------------
// Contournement direct (sans reculer)
// ------------------------------------------------------------------

// Le segment est-il libre sur la carte (PAMI orienté le long du segment) ?
bool GoTo::lineIsFree(const Pose& from, const Pose& to) const {
    if (!_opt.isPoseFree) return true;
    float heading = headingTo(from, to);
    for (int k = 1; k <= 6; ++k) {
        float f = k / 6.0f;
        Pose q = {from.x + f * (to.x - from.x), from.y + f * (to.y - from.y), heading};
        if (!_opt.isPoseFree(q)) return false;
    }
    return true;
}

bool GoTo::startBypass() {
    if (_lidar == nullptr || !_lidar->healthy()) return false;
    if (_bypassing) {  // de nouveau bloqué en contournant : recalculer d'ici
        _bypassing = false;
        _target = _savedTarget;
        _opt.stopWithinMm = _savedStopWithinMm;
    }

    // Où est l'obstacle ? Points du lidar (hors obstacles connus) devant nous
    LidarPoint pts[LidarGS2::POINT_COUNT];
    _lidar->copyPoints(pts);
    const Pose p = _drive.pose();
    float front = 1e9f, yMin = 1e9f, yMax = -1e9f;
    for (const LidarPoint& lp : pts) {
        if (!lp.valid || !lp.confirmed) continue;
        if (_opt.isKnownObstacle && _opt.isKnownObstacle(p, lp.x, lp.y)) continue;
        if (fabsf(lp.y) > 250.0f) continue;
        front = min(front, lp.x);
    }
    if (front > 1e8f) {
        Serial.println("[GoTo] contournement : obstacle introuvable dans le lidar");
        return false;
    }
    for (const LidarPoint& lp : pts) {  // ses côtés : points proches de sa face avant
        if (!lp.valid || !lp.confirmed || lp.x > front + 120.0f || fabsf(lp.y) > 250.0f) continue;
        if (_opt.isKnownObstacle && _opt.isKnownObstacle(p, lp.x, lp.y)) continue;
        yMin = min(yMin, lp.y);
        yMax = max(yMax, lp.y);
    }

    // Point de passage à côté de lui, au-delà de sa face avant (repère PAMI).
    // On essaie d'abord large, puis plus serré (plus près de l'obstacle,
    // retour plus tôt sur la route), à droite puis à gauche.
    const float c = cosf(p.theta), s = sinf(p.theta);
    const float clearances[2] = {_opt.bypassClearanceMm, _opt.bypassClearanceMm * 0.4f};
    const float beyonds[2] = {_opt.bypassBeyondMm, _opt.bypassBeyondMm * 0.6f};
    for (int variant = 0; variant < 4; ++variant)
    for (float side : {-1.0f, 1.0f}) {  // droite d'abord (y < 0 = droite)
        const float ahead = _opt.lidarOffsetMm + front + beyonds[variant / 2];
        const float offset = _opt.halfWidthMm + clearances[variant % 2];
        float lateral = side < 0 ? yMin - offset : yMax + offset;
        Pose b = {p.x + c * ahead - s * lateral, p.y + s * ahead + c * lateral, 0.0f};
        if (!lineIsFree(p, b) || !lineIsFree(b, _target)) continue;
        Serial.printf("[GoTo] obstacle immobile : contournement par la %s\n",
                      side < 0 ? "droite" : "gauche");
        _bypassing = true;
        _savedTarget = _target;
        _savedStopWithinMm = _opt.stopWithinMm;
        _target = b;
        _opt.stopWithinMm = 0.0f;
        _blockedSinceMs = 0;
        _prevFrontMm = 0xFFFF;
        if (_drive.isPaused()) _drive.stop();
        if (!steerToTarget(_opt.smoothMaxRad, true)) planLeg();
        return true;
    }
    Serial.println("[GoTo] contournement : pas de place a cote (carte)");
    return false;  // pas de place : reculer (detour)
}

// ------------------------------------------------------------------
// Contournement en reculant (s'il n'y a pas la place de passer à côté)
// ------------------------------------------------------------------
void GoTo::startDetour() {
    Serial.println("[GoTo] bloque : contournement");
    _blockedSinceMs = 0;
    _detourBackups = 0;
    _drive.forward(-_opt.detourBackMm);  // remplace la consigne en pause
    _phase = Phase::DetourBack;
}

// Pas de côté à droite si la place est libre sur la carte, sinon à gauche,
// sinon on reste reculé et on attend.
void GoTo::chooseDetourSide() {
    Pose p = _drive.pose();
    for (float side : {-1.0f, 1.0f}) {  // -1 = droite (sens horaire)
        float heading = p.theta + side * _opt.detourAngleRad;
        Pose end = {p.x + _opt.detourSideMm * cosf(heading),
                    p.y + _opt.detourSideMm * sinf(heading), heading};
        if (_opt.isPoseFree && !_opt.isPoseFree(end)) continue;
        Serial.printf("[GoTo] decalage a %s\n", side < 0 ? "droite" : "gauche");
        _drive.turn(side * _opt.detourAngleRad);
        _phase = Phase::DetourTurn;
        return;
    }
    // Pas de place (passage étroit) : reculer encore pour en sortir
    if (_detourBackups < MAX_DETOUR_BACKUPS) {
        _detourBackups++;
        _drive.forward(-_opt.detourBackMm);
        _phase = Phase::DetourBack;
        return;
    }
    Serial.println("[GoTo] pas de place pour se decaler : attente");
    waitThenReplan();
}

void GoTo::waitThenReplan() {
    _detourWaitUntilMs = millis() + randomBetween(_opt.detourWaitMinMs, _opt.detourWaitMaxMs);
    _phase = Phase::DetourWait;
}

// ------------------------------------------------------------------
// Esquive en S
// ------------------------------------------------------------------

// À chaque nouvelle trame lidar : à quelle vitesse l'obstacle devant se
// rapproche-t-il, et se rapproche-t-il plus que la distance que nous avons
// parcourue (il vient vers nous) ?
void GoTo::trackFront() {
    if (_lidar == nullptr || !_lidar->healthy()) return;
    uint32_t frame = _lidar->stats().frames;
    if (frame == _lastLidarFrame) return;
    _lastLidarFrame = frame;

    uint16_t front = _lidar->frontMinMm();
    Pose p = _drive.pose();
    uint32_t now = millis();
    if (front != 0xFFFF && _prevFrontMm != 0xFFFF) {
        float closing = (float)_prevFrontMm - (float)front;
        float travelled = distanceMm(p, _prevPose);
        uint32_t dt = now - _prevFrontMs;
        _closingMmS = dt > 0 ? closing * 1000.0f / dt : 0.0f;
        // Seulement en roulant à peu près droit : en tournant, le couloir
        // balaie les murs et leur distance chute sans qu'ils bougent.
        bool straight = fabsf(wrapPi(p.theta - _prevPose.theta)) < 0.03f;  // < 1,7° par trame
        bool approaching = straight && closing - travelled > _opt.oncomingMinMm;
        // Deux trames de suite, pour ignorer un coin de PAMI qui pivote
        _oncomingFrames = approaching ? (uint8_t)(_oncomingFrames + 1) : 0;
    } else {
        _oncomingFrames = 0;
        _closingMmS = 0.0f;
    }
    _prevFrontMm = front;
    _prevPose = p;
    _prevFrontMs = now;
}

// Pose atteinte après un arc (longueur, rotation) partant de `from`.
static Pose afterArc(const Pose& from, float length, float rotation) {
    float fx = length, fy = 0.0f;  // repère du PAMI
    if (fabsf(rotation) > 1e-4f) {
        float radius = length / rotation;
        fx = radius * sinf(rotation);
        fy = radius * (1.0f - cosf(rotation));
    }
    float c = cosf(from.theta), s = sinf(from.theta);
    return {from.x + c * fx - s * fy, from.y + s * fx + c * fy, wrapPi(from.theta + rotation)};
}

bool GoTo::startDodge() {
    Pose p = _drive.pose();
    const float L = _opt.dodgeArcMm, a = _opt.dodgeAngleRad;
    // Toujours à droite : si chacun s'écarte à sa droite, deux PAMI face à
    // face se croisent. Si notre droite est bloquée (bord, muraille), on ne
    // part pas à gauche (on irait du côté où l'autre s'écarte) : on s'arrête
    // et on le laisse passer.
    for (float side : {-1.0f}) {
        Pose half = afterArc(p, L / 2, side * a / 2);
        Pose mid = afterArc(p, L, side * a);
        Pose end = afterArc(mid, L, -side * a);
        if (_opt.isPoseFree &&
            !(_opt.isPoseFree(half) && _opt.isPoseFree(mid) && _opt.isPoseFree(end))) {
            continue;
        }
        Serial.printf("[GoTo] PAMI en face : esquive a %s\n", side < 0 ? "droite" : "gauche");
        _dodgeSide = side;
        _dodgeStartHeading = p.theta;
        _oncomingFrames = 0;
        _blockedSinceMs = 0;
        _drive.arc(L, side * a);  // remplace le mouvement en cours, sans arrêt
        _phase = Phase::DodgeOut;
        return true;
    }
    return false;  // pas de place : arrêt et contournement habituels
}

void GoTo::updateDodge() {
    // Sécurité : s'il est malgré tout trop près, pause le temps qu'il passe
    if (_lidar != nullptr && _lidar->healthy()) {
        bool tooClose = _lidar->frontMinMm() < _opt.dodgeAbortMm;
        if (tooClose && !_drive.isPaused()) _drive.pause();
        if (!tooClose && _drive.isPaused()) _drive.resume();
    }

    // Enchaîner les arcs sans s'arrêter : on lance le suivant juste avant la
    // fin du précédent (repéré par l'angle déjà tourné).
    float turned = fabsf(wrapPi(_drive.pose().theta - _dodgeStartHeading));
    if (_phase == Phase::DodgeOut) {
        if (turned >= 0.9f * _opt.dodgeAngleRad || !_drive.isBusy()) {
            _drive.arc(_opt.dodgeArcMm, -_dodgeSide * _opt.dodgeAngleRad);
            _phase = Phase::DodgeBack;
        }
    } else if (turned <= 0.1f * _opt.dodgeAngleRad || !_drive.isBusy()) {
        // Redressé : reprendre vers la cible
        _prevFrontMm = 0xFFFF;
        _blockedSinceMs = 0;
        _phase = Phase::Driving;
        if (!steerToTarget(_opt.maxSteerRad, true)) planLeg();  // arc vérifié sur la carte
    }
}

uint32_t GoTo::randomBetween(uint32_t lo, uint32_t hi) {
    _rng ^= _rng << 13;  // xorshift32
    _rng ^= _rng >> 17;
    _rng ^= _rng << 5;
    return hi > lo ? lo + _rng % (hi - lo + 1) : lo;
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
