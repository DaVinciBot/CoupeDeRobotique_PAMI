#pragma once

// Action élémentaire exécutée par une Strategy.
// Règle : aucune méthode ne bloque. start() lance, update() fait avancer,
// isFinished() dit quand passer à la suivante.
class Action {
   public:
    virtual ~Action() = default;
    virtual void start() = 0;
    virtual void update() = 0;
    virtual void stop() = 0;
    virtual bool isFinished() const = 0;
    virtual const char* name() const = 0;
};
