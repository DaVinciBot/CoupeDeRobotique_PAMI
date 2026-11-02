#include "Strategy.h"

void Strategy::add(Action* action) {
    if (action != nullptr) _actions.emplace_back(action);
}

void Strategy::clear() {
    stop();
    _actions.clear();
    _index = 0;
}

void Strategy::start() {
    _index = 0;
    _running = !_actions.empty();
    if (_running) startCurrent();
}

void Strategy::startCurrent() {
    Action* a = _actions[_index].get();
    Serial.printf("[Strategy] %u/%u %s\n", (unsigned)(_index + 1),
                  (unsigned)_actions.size(), a->name());
    a->start();
}

void Strategy::update() {
    if (!_running) return;

    Action* a = _actions[_index].get();
    a->update();
    if (!a->isFinished()) return;

    if (++_index >= _actions.size()) {
        _running = false;
        Serial.println("[Strategy] terminee");
        return;
    }
    startCurrent();
}

void Strategy::stop() {
    if (_running && _index < _actions.size()) _actions[_index]->stop();
    _running = false;
}
