#pragma once

#include <Arduino.h>

#include <memory>
#include <vector>

#include "Action.h"

// Enchaîne des actions l'une après l'autre (repris du code 2026).
class Strategy {
   public:
    void add(Action* action);  // la Strategy devient propriétaire
    void clear();

    void start();
    void update();
    void stop();

    bool isRunning() const { return _running; }
    bool isFinished() const { return !_running; }
    size_t size() const { return _actions.size(); }

   private:
    void startCurrent();

    std::vector<std::unique_ptr<Action>> _actions;
    size_t _index = 0;
    bool _running = false;
};
