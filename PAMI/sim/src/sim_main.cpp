// Simulateur de match PAMI : fait tourner le vrai code des 6 PAMI
// (pami_app.cpp, strategies.cpp, lib/actions, lib/drive...) sur PC, avec des
// moteurs, un lidar et des adversaires simulés, puis écrit le déroulé dans un
// fichier lu par sim/viewer.html.
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
        "  --no-enemies          sans PAMI adverses\n"
        "  --size L W            taille d'un PAMI en mm (defaut : 190 95)\n"
        "  --out FICHIER         fichier de sortie (defaut : sim/out/sim_data.js)\n");
}

int main(int argc, char** argv) {
    SimOptions opt;
    const char* out = "sim/out/sim_data.js";
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--team") && i + 1 < argc) {
            opt.team = !strcmp(argv[++i], "jaune") ? Team::Yellow : Team::Blue;
        } else if (!strcmp(argv[i], "--no-enemies")) {
            opt.enemies = false;
        } else if (!strcmp(argv[i], "--size") && i + 2 < argc) {
            opt.pamiLengthMm = (float)atof(argv[++i]);
            opt.pamiWidthMm = (float)atof(argv[++i]);
        } else if (!strcmp(argv[i], "--out") && i + 1 < argc) {
            out = argv[++i];
        } else {
            usage();
            return 1;
        }
    }

    SimWorld world(opt);
    HardwareSerial lidarSerial(1);

    // Nos 6 PAMI, chacun avec son propre état (comme 6 cartes)
    for (const PamiProfile& p : PAMI_PROFILES) {
        App* a = new App();
        g_currentPami = p.id;
        appBegin(*a, p.id, lidarSerial);
        world.addOurPami(a, p.id);
    }

    // Adversaires : les PAMI douve adverses vont vers nos douves, en ligne
    // droite, au moment où le règlement les autorise à sortir.
    if (opt.enemies) {
        Team enemyTeam = opt.team == Team::Blue ? Team::Yellow : Team::Blue;
        int n = 0;
        for (const PamiProfile& p : PAMI_PROFILES) {
            if (p.role != Role::Douve) continue;
            EnemyPami e;
            char name[16];
            snprintf(name, sizeof(name), "Adv. %d", ++n);
            e.name = name;
            e.pose = table::forTeam(p.startBlue, enemyTeam);
            e.target = table::forTeam(
                table::ENEMY_DOUVES[p.targetIndex % table::ENEMY_DOUVE_COUNT], enemyTeam);
            e.startMs = ATTACK_PHASE_START_MS + 300 + p.startDelayMs;
            e.speedMmS = 220.0f;
            world.addEnemy(e);
        }
    }

    // Couleur lue sur la tirette couleur
    g_pinLevels[COLOR_PIN_READ] = opt.team == Team::Yellow ? HIGH : LOW;

    // Départ simultané (tirette retirée à t = 0)
    for (OurPami& p : world.ours()) {
        g_currentPami = p.id;
        startMatch(*p.app, 0, "simulation");
    }

    // Boucle : pas de 1 ms jusqu'à la fin du match
    for (uint32_t t = 1; t <= MATCH_DURATION_MS; ++t) {
        g_simMicros = (uint64_t)t * 1000;
        simTickSteppers(0.001);
        for (OurPami& p : world.ours()) {
            g_currentPami = p.id;
            p.app->drive.update();
            matchUpdate(*p.app);
        }
        world.step(t);
    }

    world.printSummary();
    if (!world.writeOutput(out)) {
        fprintf(stderr, "Impossible d'ecrire %s\n", out);
        return 1;
    }
    printf("\nDeroule ecrit dans %s\n", out);
    return 0;
}
