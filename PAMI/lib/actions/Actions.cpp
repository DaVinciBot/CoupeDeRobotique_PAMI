#include "Actions.h"

// Pose atteinte après un arc (longueur, rotation) partant de `from` (plus bas).
static Pose afterArc(const Pose& from, float length, float rotation);

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

    // Les capteurs ne voit que devant
    if (_mm > 0 && _sensor != nullptr && _sensor->healthy()) {
        bool obstacle = _sensor->obstacle();
        // Ce n'est qu'un obstacle connu (carrière...) : il ne bougera pas,
        // inutile d'attendre, on s'arrête là et on passe à la suite.
        if (obstacle && _isKnown && !unknownObstacleAhead(*_sensor, _drive.pose(), _isKnown)) {
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
    _bypassTries = 0;
    // Graine du tirage au hasard : différente d'un PAMI à l'autre (ils ne
    // sont jamais au même endroit), et reproductible en simulation
    if (_rng == 0) {
        Pose p0 = _drive.pose();
        _rng = ((uint32_t)lroundf(p0.x) * 73856093u ^ (uint32_t)lroundf(p0.y) * 19349663u ^
                _startMs * 2654435761u) | 1u;
    }
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
    // Reprise (après une esquive, un contournement, une attente) : en courbe
    // si possible, plus large qu'au départ, pour ne pas s'arrêter pivoter.
    bool pivotNow = _pivotNext;
    _pivotNext = false;
    if (!pivotNow && _opt.smooth && _opt.replanSmoothMaxRad > 0.0f &&
        steerToTarget(_opt.replanSmoothMaxRad, true)) {
        _phase = Phase::Driving;
        _prevFrontMm = 0xFFFF;
        return;
    }
    float dTheta = wrapPi(headingTo(p, _target) - p.theta);
    // Tourner en roulant (arc de rayon turnRadiusMm) plutôt que pivoter sur
    // place : pas d'arrêt. Seulement si l'arc ne touche rien sur la carte et
    // pas pour un contournement (obstacle juste devant : pivoter).
    _arcTurning = false;
    // (près de la cible, un virage de rayon fixe tournerait autour : pivoter)
    if (!pivotNow && _opt.turnRadiusMm > 0.0f &&
        distanceMm(p, _target) >= 2.0f * _opt.turnRadiusMm) {
        float length = _opt.turnRadiusMm * fabsf(dTheta);
        bool free = true;
        for (int k = 1; k <= 6 && free && _opt.isPoseFree; ++k) {
            free = _opt.isPoseFree(afterArc(p, length * k / 6, dTheta * k / 6));
        }
        if (free) {
            _drive.arc(length, dTheta);
            _arcTurning = true;
            _phase = Phase::Turning;
            return;
        }
    }
    _drive.turn(dTheta);
    _phase = Phase::Turning;
}

// Arrivé : orientation finale si demandée, sinon terminé.
void GoTo::finish() {
    Pose p = _drive.pose();
    if (_opt.faceTarget && distanceMm(p, _target) > 2.0f) {
        float dTheta = wrapPi(headingTo(p, _target) - p.theta);
        if (fabsf(dTheta) > 0.05f) {  // ~3° : inutile de tourner pour moins
            _drive.turn(dTheta);
            _phase = Phase::FinalTurn;
            return;
        }
        _phase = Phase::Done;
    } else if (_opt.finalHeading) {
        _drive.turn(wrapPi(_target.theta - p.theta));
        _phase = Phase::FinalTurn;
    } else {
        _phase = Phase::Done;
    }
}

bool unknownObstacleAhead(const ObstacleSensor& sensor, const Pose& robot,
                          const KnownObstacleFilter& isKnown) {
    if (!isKnown) return true;
    ObstaclePoint pts[ObstacleSensor::MAX_POINTS] = {};  // au-delà des points : non valides
    sensor.copyPoints(pts);
    const float half = sensor.corridorHalfWidth();
    const float depth = sensor.obstacleThreshold();
    for (const ObstaclePoint& lp : pts) {
        if (!lp.valid || !lp.confirmed) continue;
        if (fabsf(lp.y) > half || lp.x >= depth) continue;
        if (!isKnown(robot, lp.x, lp.y)) return true;
    }
    return false;
}

bool GoTo::obstacleAhead() const {
    if (_sensor == nullptr || !_sensor->healthy() || !_sensor->obstacle()) return false;
    return unknownObstacleAhead(*_sensor, _drive.pose(), _opt.isKnownObstacle);
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
            if (_arcTurning) {
                // Virage en roulant : il avance, donc on surveille devant
                if (obstacleAhead()) {
                    _drive.stop();
                    _arcTurning = false;
                    _pivotNext = true;
                    _replanPending = true;
                    break;
                }
                // Presque face à la cible : enchaîner sans s'arrêter
                Pose p = _drive.pose();
                if (fabsf(wrapPi(headingTo(p, _target) - p.theta)) < 0.15f || !_drive.isBusy()) {
                    _arcTurning = false;
                    _prevFrontMm = 0xFFFF;
                    if (steerToTarget(0.6f, true)) {
                        _phase = Phase::Driving;
                    } else if (!_drive.isBusy()) {
                        planLeg();
                    }
                }
                break;
            }
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
                _bypassTries = 0;
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
            // Coincé sans que les capteurs voient l'obstacle : contourner
            if (stalled()) {
                Serial.println("[GoTo] bloque sans obstacle visible (capteurs figes) : contournement");
                _blockedSinceMs = 0;
                if (!(_opt.bypass && startBypass())) startDetour();
                return;
            }
            // Un PAMI vient vers nous : l'esquiver tout de suite, sans s'arrêter
            if (_opt.dodgeOncoming && d >= _opt.ignoreObstacleWithinMm && oncomingObstacle() &&
                startDodge()) {
                return;
            }
            bool obstacle = obstacleAhead() && d >= _opt.ignoreObstacleWithinMm;
            // Suivi : un PAMI devant qui avance aussi vite que nous n'est pas
            // une raison de s'arrêter, tant qu'on reste à followMinMm.
            if (obstacle && _opt.follow && !_drive.isPaused() && _sensor != nullptr &&
                _sensor->frontMinMm() >= _opt.followMinMm &&
                _closingMmS < _opt.followClosingMmS) {
                obstacle = false;
            }
            // Où va l'obstacle ? S'il traverse et sort bientôt du couloir, on
            // l'attend (sans contourner), et on repart dès que sa position
            // prévue est dehors, sans attendre qu'il soit complètement passé.
            trackObstacle();
            bool clearSoon = false;
            bool crossing = obstacle && crossingObstacle(clearSoon);
            if (crossing && clearSoon && _sensor != nullptr && _sensor->frontMinMm() >= 100) {
                obstacle = false;  // (pas s'il est tout près : la carrosserie le toucherait)
            }
            if (_opt.pauseOnObstacle) {
                if (obstacle && !_drive.isPaused()) _drive.pause();
                if (!obstacle && _drive.isPaused()) _drive.resume();

                // Bloqué trop longtemps par un obstacle qui ne s'éloigne pas :
                // contourner. Un obstacle qui s'éloigne est un PAMI qu'on suit :
                // on attend qu'il avance. Un obstacle qui traverse : on attend
                // qu'il passe.
                uint16_t front = _sensor != nullptr ? _sensor->frontMinMm() : 0xFFFF;
                if (!obstacle) {
                    _blockedSinceMs = 0;
                } else if (crossing) {
                    _blockedSinceMs = millis();  // il passe : ne pas contourner
                    _blockedFrontMm = front;
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
    if (_sensor == nullptr || !_sensor->healthy()) return false;
    if (_bypassing) {  // de nouveau bloqué en contournant : recalculer d'ici
        _bypassing = false;
        _target = _savedTarget;
        _opt.stopWithinMm = _savedStopWithinMm;
    }
    // Plusieurs contournements de suite sans avancer : il n'y a pas la
    // place de passer à côté, reculer d'abord (detour).
    if (++_bypassTries > MAX_BYPASS_TRIES) {
        Serial.println("[GoTo] contournement sans effet : recul");
        _bypassTries = 0;
        return false;
    }

    // Où est l'obstacle ? Points des capteurs (hors obstacles connus) devant nous
    ObstaclePoint pts[ObstacleSensor::MAX_POINTS] = {};  // au-delà des points : non valides
    _sensor->copyPoints(pts);
    const Pose p = _drive.pose();
    float front = 1e9f, yMin = 1e9f, yMax = -1e9f;
    for (const ObstaclePoint& lp : pts) {
        if (!lp.valid || !lp.confirmed) continue;
        if (_opt.isKnownObstacle && _opt.isKnownObstacle(p, lp.x, lp.y)) continue;
        if (fabsf(lp.y) > 250.0f) continue;
        front = min(front, lp.x);
    }
    if (front > 1e8f) {
        Serial.println("[GoTo] contournement : obstacle introuvable dans les capteurs");
        return false;
    }
    for (const ObstaclePoint& lp : pts) {  // ses côtés : points proches de sa face avant
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
    // Côté : à droite d'abord ; mais un obstacle qui traverse lentement se
    // contourne par derrière (du côté d'où il vient), pas devant lui.
    float first = -1.0f;
    if (_opt.trackMotion && _obsValid) {
        const float vLat = -s * _obsVx + c * _obsVy;
        if (fabsf(vLat) >= _opt.crossMinSpeedMmS) first = vLat > 0 ? -1.0f : 1.0f;
    }
    for (int variant = 0; variant < 4; ++variant)
    for (float side : {first, -first}) {  // y < 0 = droite
        const float ahead = _opt.sensorOffsetMm + front + beyonds[variant / 2];
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
        // Pivoter vers le point de passage avant d'avancer : en courbe,
        // l'obstacle resterait dans le couloir surveillé et le PAMI se
        // remettrait aussitôt en pause, sans jamais bouger.
        _drive.stop();
        _replanPending = true;
        _pivotNext = true;
        return true;
    }
    Serial.println("[GoTo] contournement : pas de place a cote (carte)");
    return false;  // pas de place : reculer (detour)
}

// ------------------------------------------------------------------
// Contournement en reculant (s'il n'y a pas la place de passer à côté)
// ------------------------------------------------------------------
// Recul possible sans toucher la carte (bord, muraille) ? Aucun capteur
// derrière : on vérifie au moins la carte. Renvoie la distance de recul
// (want, sinon la moitié), 0 s'il n'y a pas la place.
float GoTo::safeBackMm(float want) const {
    if (!_opt.isPoseFree) return want;
    Pose p = _drive.pose();
    for (float d : {want, want / 2}) {
        Pose back = {p.x - d * cosf(p.theta), p.y - d * sinf(p.theta), p.theta};
        if (_opt.isPoseFree(back)) return d;
    }
    return 0.0f;
}

void GoTo::startDetour() {
    Serial.println("[GoTo] bloque : contournement");
    _blockedSinceMs = 0;
    _detourBackups = 0;
    float back = safeBackMm(_opt.detourBackMm);
    if (back > 0.0f) {
        _drive.forward(-back);  // remplace la consigne en pause
    } else {
        _drive.stop();  // pas la place de reculer : se décaler d'ici
    }
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
    float back = safeBackMm(_opt.detourBackMm);
    if (_detourBackups < MAX_DETOUR_BACKUPS && back > 0.0f) {
        _detourBackups++;
        _drive.forward(-back);
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

// À chaque nouvelle trame des capteurs : à quelle vitesse l'obstacle devant se
// rapproche-t-il, et se rapproche-t-il plus que la distance que nous avons
// parcourue (il vient vers nous) ?
void GoTo::trackFront() {
    if (_sensor == nullptr || !_sensor->healthy()) return;
    uint32_t frame = _sensor->frames();
    if (frame == _lastSensorFrame) return;
    _lastSensorFrame = frame;

    uint16_t front = _sensor->frontMinMm();
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

// Blocage invisible : l'odométrie a avancé de stallTravelMm, mais au moins 2
// capteurs voient toujours les mêmes distances -> le PAMI n'a pas bougé
// (roues qui patinent, moteurs qui sautent des pas). Il faut au moins 2
// capteurs : un seul pourrait suivre un PAMI qui roule à notre vitesse, ou
// longer un bord parallèle.
bool GoTo::stalled() {
    if (!_opt.detectStall || _sensor == nullptr || !_sensor->healthy()) return false;
    uint32_t frame = _sensor->frames();
    if (frame == _stallFrame) return false;
    _stallFrame = frame;

    ObstaclePoint pts[ObstacleSensor::MAX_POINTS] = {};
    uint8_t n = _sensor->copyPoints(pts);
    Pose p = _drive.pose();
    if (_drive.isPaused() || !_drive.isMoving() || !_stallRefSet) {
        _stallRefSet = !_drive.isPaused() && _drive.isMoving();
        _stallRefPose = p;
        for (uint8_t i = 0; i < n; ++i) {
            _stallRefValid[i] = pts[i].valid;
            _stallRefRange[i] = hypotf(pts[i].x, pts[i].y);
        }
        return false;
    }
    if (distanceMm(p, _stallRefPose) < _opt.stallTravelMm) return false;

    uint8_t same = 0, compared = 0;
    for (uint8_t i = 0; i < n; ++i) {
        if (!pts[i].valid || !_stallRefValid[i]) continue;
        ++compared;
        if (fabsf(hypotf(pts[i].x, pts[i].y) - _stallRefRange[i]) <= _opt.stallToleranceMm) ++same;
    }
    _stallRefSet = false;  // nouvelle référence à la prochaine trame
    return compared >= 2 && same == compared;
}

// Suivi de l'obstacle devant : à chaque trame des capteurs, son point le plus
// proche (hors obstacles connus) placé sur la table ; s'il est à moins de
// 12 cm du précédent, c'est le même objet : sa vitesse est mise à jour.
void GoTo::trackObstacle() {
    if (!_opt.trackMotion || _sensor == nullptr || !_sensor->healthy()) return;
    uint32_t frame = _sensor->frames();
    if (frame == _obsFrame) return;
    _obsFrame = frame;

    ObstaclePoint pts[ObstacleSensor::MAX_POINTS] = {};
    _sensor->copyPoints(pts);
    const Pose p = _drive.pose();
    float best = 1e9f, bx = 0, by = 0;
    for (const ObstaclePoint& lp : pts) {
        if (!lp.valid || !lp.confirmed || fabsf(lp.y) > 250.0f) continue;
        if (_opt.isKnownObstacle && _opt.isKnownObstacle(p, lp.x, lp.y)) continue;
        float d = lp.x * lp.x + lp.y * lp.y;
        if (d < best) {
            best = d;
            bx = lp.x;
            by = lp.y;
        }
    }
    uint32_t now = millis();
    if (best > 1e8f) {
        if (now - _obsMs > 300) _obsValid = false;
        return;
    }
    const float x = bx + _opt.sensorOffsetMm, c = cosf(p.theta), s = sinf(p.theta);
    const float tx = p.x + c * x - s * by, ty = p.y + s * x + c * by;
    float dt = (now - _obsMs) / 1000.0f;
    if (_obsValid && dt > 0.01f && dt < 0.3f && hypotf(tx - _obsX, ty - _obsY) < 120.0f) {
        _obsVx = 0.6f * _obsVx + 0.4f * (tx - _obsX) / dt;
        _obsVy = 0.6f * _obsVy + 0.4f * (ty - _obsY) / dt;
    } else {
        _obsVx = _obsVy = 0.0f;  // nouvel objet : vitesse inconnue
    }
    _obsX = tx;
    _obsY = ty;
    _obsMs = now;
    _obsValid = true;
}

bool GoTo::crossingObstacle(bool& clearSoon) const {
    clearSoon = false;
    if (!_opt.trackMotion || !_obsValid || _sensor == nullptr) return false;
    const Pose p = _drive.pose();
    const float c = cosf(p.theta), s = sinf(p.theta);
    // Repère du PAMI : latéral (+ à gauche) de la position et de la vitesse
    const float lat = -s * (_obsX - p.x) + c * (_obsY - p.y);
    const float vLat = -s * _obsVx + c * _obsVy;
    if (fabsf(vLat) < _opt.crossMinSpeedMmS) return false;
    // Il sort du couloir du côté où il va : distance qu'il lui reste
    const float edge = _sensor->corridorHalfWidth() + _opt.obstacleHalfMm;
    const float dir = vLat > 0 ? 1.0f : -1.0f;
    const float remaining = edge - lat * dir;
    if (remaining <= 0.0f) {
        clearSoon = true;
        return true;
    }
    const float tClear = remaining / fabsf(vLat);
    if (tClear > _opt.crossWaitMaxS) return false;  // trop lent : contourner
    // Position prévue dans anticipateS : déjà hors du couloir -> repartir
    clearSoon = remaining - fabsf(vLat) * _opt.anticipateS <= 0.0f;
    return true;
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
    if (_sensor != nullptr && _sensor->healthy()) {
        bool tooClose = _sensor->frontMinMm() < _opt.dodgeAbortMm;
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
// KeepDistance
// ------------------------------------------------------------------
void KeepDistance::start() {
    _finished = false;
    _pivoting = false;
    _lastMs = millis() - _opt.periodMs;  // première consigne tout de suite
}

void KeepDistance::update() {
    if (_finished) return;
    if (_pivoting) {  // finir le pivot avant de recalculer
        if (_drive.isBusy()) return;
        _pivoting = false;
    }
    uint32_t now = millis();
    if (now - _lastMs < _opt.periodMs) return;
    _lastMs = now;

    Pose p = _drive.pose();
    Pose target;
    if (!_opt.provider || !_opt.provider(p, target)) {
        if (_drive.isBusy()) _drive.stop();  // cible perdue : on reste là
        return;
    }
    float d = distanceMm(p, target);
    float gap = d - _opt.frontAheadMm;  // avant du PAMI <-> face de la cible
    float alpha = wrapPi(headingTo(p, target) - p.theta);
    // La position de la cible donne un écart prudent (son orientation n'est
    // pas connue) ; ce que mesurent les capteurs devant nous est l'écart réel
    // avec ce qu'il y a de plus proche : on le préfère quand ils le voient.
    if (_sensor != nullptr && _sensor->healthy() && _sensor->frontMinMm() != 0xFFFF &&
        fabsf(alpha) < _opt.maxHeadingRad) {
        gap = _sensor->frontMinMm() - (_opt.frontAheadMm - _opt.sensorAheadMm);
    }
    float error = gap - _opt.gapMm;  // > 0 : trop loin

    // Trop près (elle s'approche, ou il faut pivoter) : reculer, si la carte le permet
    bool needPivot = fabsf(alpha) > _opt.maxHeadingRad;
    if (error < -_opt.toleranceMm || (needPivot && gap < _opt.gapMm + 30.0f)) {
        float back = fminf(_opt.gapMm + 30.0f - gap, _opt.maxStepMm);
        Pose after = {p.x - back * cosf(p.theta), p.y - back * sinf(p.theta), p.theta};
        if (back > 2.0f && (!_opt.isPoseFree || _opt.isPoseFree(after))) _drive.forward(-back);
        return;
    }
    if (needPivot) {  // cible sur le côté : pivoter face à elle
        _drive.turn(alpha);
        _pivoting = true;
        return;
    }
    if (error > _opt.toleranceMm) {  // trop loin : avancer en courbe vers elle
        // ... sauf vers un obstacle qui n'est pas la cible (coéquipier)
        if (_sensor != nullptr && _sensor->healthy() && _sensor->obstacle() &&
            unknownObstacleAhead(*_sensor, p, _opt.isIgnored)) {
            if (_drive.isBusy()) _drive.stop();
            return;
        }
        // Arc tangent au cap actuel qui passe par la cible (comme GoTo),
        // limité à maxStepMm et arrêté à gapMm de sa face.
        float part = fminf(error, _opt.maxStepMm) / d;
        float arcLen = fabsf(alpha) < 1e-3f ? d : d * alpha / sinf(alpha);
        float length = arcLen * part, rotation = 2.0f * alpha * part;
        // Pas sur la carte (muraille, bord, zone d'un coéquipier) : rester là
        if (_opt.isPoseFree) {
            for (int k = 1; k <= 4; ++k) {
                if (!_opt.isPoseFree(afterArc(p, length * k / 4, rotation * k / 4))) {
                    if (_drive.isBusy()) _drive.stop();
                    return;
                }
            }
        }
        _drive.arc(length, rotation);
        return;
    }
    // À la bonne distance : seulement rester face à elle
    if (fabsf(alpha) > 0.08f) {  // ~5°
        _drive.turn(alpha);
        _pivoting = true;
    } else if (_drive.isBusy()) {
        _drive.stop();
    }
}

void KeepDistance::stop() {
    _drive.stop();
    _finished = true;
}

// ------------------------------------------------------------------
// FollowPath
// ------------------------------------------------------------------
void FollowPath::start() {
    _startMs = millis();
    _lastGoalMs = 0;
    _lastSteerMs = 0;
    _replanNow = true;
    _pathCount = 0;
    _sensed = false;
    _phase = _opt.waitForGoal && _opt.goalProvider ? Phase::Waiting : Phase::Following;
    if (_phase == Phase::Waiting) Serial.println("[Chemin] attente d'un but");
}

Pose FollowPath::center() const {
    Pose p = _drive.pose();
    return {p.x + _opt.centerAheadMm * cosf(p.theta), p.y + _opt.centerAheadMm * sinf(p.theta),
            p.theta};
}

// Obstacle devant vu par les capteurs, que le filtre ne reconnaît pas : il
// devient un obstacle pour le calcul du chemin.
bool FollowPath::blockedBySensors(const Pose& axle) {
    if (_sensor == nullptr || !_sensor->healthy() || !_sensor->obstacle()) return false;
    if (!unknownObstacleAhead(*_sensor, axle, _opt.isIgnored)) return false;
    ObstaclePoint pts[ObstacleSensor::MAX_POINTS] = {};
    _sensor->copyPoints(pts);
    float best = 1e9f;
    const float c = cosf(axle.theta), s = sinf(axle.theta);
    for (const ObstaclePoint& lp : pts) {
        if (!lp.valid || !lp.confirmed) continue;
        if (_opt.isIgnored && _opt.isIgnored(axle, lp.x, lp.y)) continue;
        float d = lp.x * lp.x + lp.y * lp.y;
        if (d >= best) continue;
        best = d;
        // le point vu est une face : le robot est un peu derrière
        float x = lp.x + _opt.sensorAheadMm + 60.0f;
        _sensedX = axle.x + c * x - s * lp.y;
        _sensedY = axle.y + s * x + c * lp.y;
    }
    if (best < 1e8f) {
        if (!_sensed || millis() - _sensedMs > 300) _replanNow = true;
        _sensed = true;
        _sensedMs = millis();
    }
    return true;
}

void FollowPath::replan(const Pose& c) {
    _lastPlanMs = millis();
    _replanNow = false;
    _planner.clearObstacles();
    if (_opt.obstacles) _opt.obstacles(_planner, _goal);
    if (_sensed && millis() - _sensedMs <= _opt.sensedObstacleMs) {
        _planner.addObstacle({_sensedX, _sensedY, 0.0f, 0.0f, _opt.sensedObstacleRadiusMm});
    }
    int n = _planner.plan({c.x, c.y}, {_goal.x, _goal.y});
    if (n > 0) {
        _pathCount = n;
        for (int i = 0; i < n; ++i) _path[i] = _planner.waypoints()[i];
    } else {  // pas de chemin : tout droit vers le but (les capteurs veillent)
        _pathCount = 1;
        _path[0] = {_goal.x, _goal.y};
    }
    if (_opt.onPath) _opt.onPath(_path, _pathCount);
}

// Point visé : le premier point du chemin pas encore atteint.
bool FollowPath::lookahead(const Pose& c, GridPlanner::Point& out) const {
    for (int k = 0; k < _pathCount; ++k) {
        if (k == _pathCount - 1 || hypotf(_path[k].x - c.x, _path[k].y - c.y) > _opt.lookaheadMm / 3) {
            out = _path[k];
            return true;
        }
    }
    return false;
}

void FollowPath::arrive() {
    _drive.stop();
    Pose p = _drive.pose();
    if (_opt.finalHeading) {
        _drive.turn(wrapPi(_goal.theta - p.theta));
        _phase = Phase::FinalTurn;
        return;
    }
    if (_opt.faceGoal) {
        Pose c = center();
        float alpha = wrapPi(atan2f(_goal.y - c.y, _goal.x - c.x) - p.theta);
        if (fabsf(alpha) > 0.05f) {
            _drive.turn(alpha);
            _phase = Phase::FinalTurn;
            return;
        }
    }
    _phase = Phase::Done;
}

void FollowPath::update() {
    if (_phase == Phase::Done) return;
    uint32_t now = millis();
    if (_opt.timeoutMs > 0 && now - _startMs > _opt.timeoutMs) {
        Serial.println("[Chemin] timeout");
        stop();
        return;
    }
    Pose c = center();

    // But qui bouge : relu régulièrement
    if (_opt.goalProvider && now - _lastGoalMs >= _opt.replanMs) {
        _lastGoalMs = now;
        Pose g;
        if (_opt.goalProvider(c, g)) {
            _goal = g;
            if (_phase == Phase::Waiting) {
                Serial.printf("[Chemin] but (%.0f, %.0f)\n", g.x, g.y);
                _phase = Phase::Following;
                _replanNow = true;
            }
        }
    }
    if (_phase == Phase::Waiting) return;
    if (_phase == Phase::Pivot || _phase == Phase::FinalTurn) {
        if (_drive.isBusy()) return;
        if (_phase == Phase::FinalTurn) {
            _phase = Phase::Done;
            return;
        }
        _phase = Phase::Following;
        _lastSteerMs = 0;
    }

    if (hypotf(_goal.x - c.x, _goal.y - c.y) <= _opt.stopWithinMm) {
        Serial.println("[Chemin] but atteint");
        arrive();
        return;
    }
    const Pose axle = _drive.pose();
    bool blocked = blockedBySensors(axle);
    if (blocked && _drive.isBusy() && !_drive.isPaused()) _drive.stop();
    if (_replanNow || now - _lastPlanMs >= _opt.replanMs) replan(c);
    if (now - _lastSteerMs < _opt.steerMs) return;
    _lastSteerMs = now;

    GridPlanner::Point L;
    if (!lookahead(c, L)) L = {_goal.x, _goal.y};
    float dx = L.x - c.x, dy = L.y - c.y;
    float dist = hypotf(dx, dy);
    if (dist < 1.0f) return;
    float alpha = wrapPi(atan2f(dy, dx) - axle.theta);
    // Obstacle devant et chemin qui tourne : pivoter (avancer le toucherait)
    if (blocked && fabsf(alpha) > 0.35f) {
        _drive.turn(alpha);
        _phase = Phase::Pivot;
        return;
    }
    if (blocked) return;  // le nouveau chemin passe encore devant : attendre
    // Trop sur le côté : tourner en roulant (arc serré), consigne renouvelée
    // à chaque pas, au lieu de s'arrêter pour pivoter
    if (fabsf(alpha) > _opt.pivotAboveRad) {
        // Près du point visé, un virage de rayon fixe tournerait autour sans
        // jamais l'atteindre : pivoter
        if (_opt.turnRadiusMm <= 0.0f || dist < 2.0f * _opt.turnRadiusMm) {
            _drive.turn(alpha);
            _phase = Phase::Pivot;
            return;
        }
        float rot = fmaxf(-0.6f, fminf(0.6f, alpha));
        if (!arcFree(axle, _opt.turnRadiusMm * fabsf(rot), rot)) {
            _drive.turn(alpha);  // pas la place (muraille, bord) : pivoter
            _phase = Phase::Pivot;
            return;
        }
        _drive.arc(_opt.turnRadiusMm * fabsf(rot), rot);
        return;
    }

    // L'axe vise le point tel que le centre arrive sur L ; arc tangent au cap
    float ax = L.x - _opt.centerAheadMm * dx / dist, ay = L.y - _opt.centerAheadMm * dy / dist;
    float cs = cosf(axle.theta), sn = sinf(axle.theta);
    float ahead = cs * (ax - axle.x) + sn * (ay - axle.y);
    float left = -sn * (ax - axle.x) + cs * (ay - axle.y);
    float a2 = atan2f(left, ahead), d2 = hypotf(ahead, left);
    if (d2 < 5.0f) return;
    if (fabsf(a2) > 1.3f) {
        float rot = fmaxf(-0.6f, fminf(0.6f, a2));
        if (_opt.turnRadiusMm > 0.0f && d2 >= 2.0f * _opt.turnRadiusMm &&
            arcFree(axle, _opt.turnRadiusMm * fabsf(rot), rot)) {
            _drive.arc(_opt.turnRadiusMm * fabsf(rot), rot);
        } else {
            _drive.turn(a2);
            _phase = Phase::Pivot;
        }
        return;
    }
    float remaining = hypotf(_goal.x - c.x, _goal.y - c.y) - _opt.stopWithinMm;
    float step = fminf(fminf(d2, _opt.maxStepMm), fmaxf(remaining, 5.0f));
    float part = step / d2;
    float arcLen = fabsf(a2) < 1e-3f ? d2 : d2 * a2 / sinf(a2);
    _drive.arc(arcLen * part, 2.0f * a2 * part);
}

bool FollowPath::arcFree(const Pose& from, float length, float rotation) const {
    if (!_opt.isPoseFree) return true;
    for (int k = 1; k <= 4; ++k) {
        if (!_opt.isPoseFree(afterArc(from, length * k / 4, rotation * k / 4))) return false;
    }
    return true;
}

void FollowPath::stop() {
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
