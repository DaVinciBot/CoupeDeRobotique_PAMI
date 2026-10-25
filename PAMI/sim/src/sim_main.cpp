// Simulateur de match PAMI : fait tourner le vrai code des 6 PAMI
// (pami_app.cpp, strategies.cpp, lib/actions, lib/drive...) sur PC, en 6
// contre 6 : l'adversaire fait tourner le même code, dans l'autre couleur.
// Moteurs et lidar simulés ; le déroulé est écrit dans un fichier lu par
// sim/viewer.html.
//
// Lancement : python sim/run.py   (voir sim/README.md)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "sim_log.h"
#include "sim_world.h"

static void usage() {
    printf(
        "Options :\n"
        "  --team bleu|jaune     equipe de nos PAMI (defaut : bleu)\n"
        "  --no-enemies          sans les 6 PAMI adverses\n"
        "  --speed MM_S          vitesse max des PAMI (defaut : config.h)\n"
        "  --out FICHIER         fichier de sortie (defaut : sim/out/sim_data.js)\n"
        "La taille du PAMI et la position de l'axe se reglent a la compilation\n"
        "(python sim/run.py --size L W --axle A).\n");
}

int main(int argc, char** argv) {
    SimOptions opt;
    const char* out = "sim/out/sim_data.js";
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--team") && i + 1 < argc) {
            opt.team = !strcmp(argv[++i], "jaune") ? Team::Yellow : Team::Blue;
        } else if (!strcmp(argv[i], "--no-enemies")) {
            opt.enemies = false;
        } else if (!strcmp(argv[i], "--speed") && i + 1 < argc) {
            opt.speedMmS = (float)atof(argv[++i]);
        } else if (!strcmp(argv[i], "--out") && i + 1 < argc) {
            out = argv[++i];
        } else {
            usage();
            return 1;
        }
    }

    SimWorld world(opt);
    HardwareSerial lidarSerial(1);

    // Nos 6 PAMI, puis les 6 de l'adversaire : même code, autre couleur.
    // Chacun a son propre état, comme 6 + 6 cartes.
    Team enemyTeam = opt.team == Team::Blue ? Team::Yellow : Team::Blue;
    for (int side = 0; side < (opt.enemies ? 2 : 1); ++side) {
        bool ours = side == 0;
        for (const PamiProfile& p : PAMI_PROFILES) {
            App* a = new App();
            int logId = ours ? p.id : 100 + p.id;
            g_currentPami = logId;
            appBegin(*a, p.id, lidarSerial);
            if (opt.speedMmS > 0.0f) a->drive.setSpeed(opt.speedMmS);
            world.addPami(a, logId, ours, ours ? opt.team : enemyTeam);
        }
    }

    // Départ simultané (tirette retirée à t = 0). Chaque PAMI lit sa couleur
    // sur sa tirette couleur au moment du départ.
    for (SimPami& p : world.pamis()) {
        g_currentPami = p.logId;
        g_pinLevels[COLOR_PIN_READ] = p.team == Team::Yellow ? HIGH : LOW;
        startMatch(*p.app, 0, "simulation");
    }

    // Boucle : pas de 1 ms jusqu'à la fin du match
    for (uint32_t t = 1; t <= MATCH_DURATION_MS; ++t) {
        g_simMicros = (uint64_t)t * 1000;
        simTickSteppers(0.001);
        for (SimPami& p : world.pamis()) {
            g_currentPami = p.logId;
            p.app->drive.update();
            matchUpdate(*p.app);
        }
        world.step(t);
    }

    world.printSummary();
    if (!world.writeOutput(out, opt.enemies ? "avec" : "sans")) {
        fprintf(stderr, "Impossible d'ecrire %s\n", out);
        return 1;
    }
    printf("\nDeroule ecrit dans %s\n", out);
    return 0;
}
