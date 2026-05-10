#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

/* Structure imposée par le cahier des charges */
struct Drone {
    int   id;
    float x, y, z;
};

/* Résultat : les deux drones les plus proches */
typedef struct {
    struct Drone *a;
    struct Drone *b;
    float dist;
} Paire;

/* Distance euclidienne 3D */
float distance_3d(struct Drone *a, struct Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* ---- Tri par fusion par coordonnée X ---- */

void fusionner(struct Drone **arr, struct Drone **tmp, int g, int m, int d) {
    int i = g, j = m + 1, k = g;

    while (i <= m && j <= d) {
        if ((*(arr+i))->x <= (*(arr+j))->x)
            *(tmp+k++) = *(arr+i++);
        else
            *(tmp+k++) = *(arr+j++);
    }
    while (i <= m) *(tmp+k++) = *(arr+i++);
    while (j <= d) *(tmp+k++) = *(arr+j++);

    for (int n = g; n <= d; n++)
        *(arr+n) = *(tmp+n);
}

void tri_fusion(struct Drone **arr, struct Drone **tmp, int g, int d) {
    if (g >= d) return;
    int m = g + (d - g) / 2;
    tri_fusion(arr, tmp, g,   m);
    tri_fusion(arr, tmp, m+1, d);
    fusionner (arr, tmp, g,   m, d);
}

/* ---- FORCE BRUTE (pour petits groupes <= 3) ---- */

Paire force_brute(struct Drone **tab, int n) {
    Paire best = { NULL, NULL, FLT_MAX };
    for (int i = 0; i < n; i++) {
        for (int j = i+1; j < n; j++) {
            float d = distance_3d(*(tab+i), *(tab+j));
            if (d < best.dist) {
                best.dist = d;
                best.a = *(tab+i);
                best.b = *(tab+j);
            }
        }
    }
    return best;
}

/* ---- Diviser et Conquerir ---- */

Paire diviser_conquerir(struct Drone **tri_x, int n, struct Drone **bande) {
    /* Cas de base */
    if (n <= 3)
        return force_brute(tri_x, n);

    int m = n / 2;
    float x_median = (*(tri_x + m))->x;

    /* Appels récursifs sur les deux moitiés */
    Paire gauche = diviser_conquerir(tri_x,     m,   bande);
    Paire droite = diviser_conquerir(tri_x + m, n-m, bande);

    /* On garde la meilleure des deux */
    Paire best = (gauche.dist < droite.dist) ? gauche : droite;
    float delta = best.dist;

    /* Construction de la bande autour de la ligne médiane */
    int nb = 0;
    for (int i = 0; i < n; i++) {
        struct Drone *d = *(tri_x + i);
        if (fabsf(d->x - x_median) < delta)
            *(bande + nb++) = d;
    }

    /* Tri de la bande par Y (tri insertion) */
    for (int i = 1; i < nb; i++) {
        struct Drone *key = *(bande + i);
        int j = i - 1;
        while (j >= 0 && (*(bande+j))->y > key->y) {
            *(bande+j+1) = *(bande+j);
            j--;
        }
        *(bande+j+1) = key;
    }

    /* Vérification dans la bande (max 7 comparaisons par drone) */
    for (int i = 0; i < nb; i++) {
        for (int j = i+1; j < nb; j++) {
            if ((*(bande+j))->y - (*(bande+i))->y >= delta) break;
            float d = distance_3d(*(bande+i), *(bande+j));
            if (d < best.dist) {
                best.dist = d;
                best.a = *(bande+i);
                best.b = *(bande+j);
                delta = d;
            }
        }
    }

    return best;
}

/* ---- FONCTION PRINCIPALE ---- */

Paire trouver_paire_proche(struct Drone *essaim, int N) {
    Paire vide = { NULL, NULL, FLT_MAX };

    /* Tableau de pointeurs (on trie les pointeurs, pas les données) */
    struct Drone **tri_x = malloc(N * sizeof(struct Drone *));
    struct Drone **tmp   = malloc(N * sizeof(struct Drone *));
    struct Drone **bande = malloc(N * sizeof(struct Drone *));

    if (!tri_x || !tmp || !bande) {
        fprintf(stderr, "Erreur malloc\n");
        free(tri_x); free(tmp); free(bande);
        return vide;
    }

    /* Initialisation avec arithmétique de pointeurs */
    for (int i = 0; i < N; i++)
        *(tri_x + i) = essaim + i;

    /* Tri par X : O(n log n) */
    tri_fusion(tri_x, tmp, 0, N-1);

    /* Divide & Conquer : O(n log^2 n) */
    Paire resultat = diviser_conquerir(tri_x, N, bande);

    free(tri_x); free(tmp); free(bande);
    return resultat;
}

/* ---- MAIN ---- */

int main(void) {
    const int N = 10000;

    /* Allocation de l'entrepôt */
    struct Drone *essaim = malloc(N * sizeof(struct Drone));
    if (!essaim) { fprintf(stderr, "Erreur malloc\n"); return 1; }

    /* Génération aléatoire des positions */
    srand((unsigned)time(NULL));
    for (int i = 0; i < N; i++) {
        (essaim + i)->id = i;
        (essaim + i)->x  = (float)(rand() % 100000) / 100.0f;
        (essaim + i)->y  = (float)(rand() % 100000) / 100.0f;
        (essaim + i)->z  = (float)(rand() % 100000) / 100.0f;
    }

    /* Injection d'une paire connue pour validation */
    (essaim + 0)->x = 500.0f; (essaim + 0)->y = 500.0f; (essaim + 0)->z = 500.0f;
    (essaim + 1)->x = 500.1f; (essaim + 1)->y = 500.1f; (essaim + 1)->z = 500.1f;

    /* Exécution + mesure du temps */
    clock_t debut = clock();
    Paire res = trouver_paire_proche(essaim, N);
    double ms = 1000.0 * (clock() - debut) / CLOCKS_PER_SEC;

    /* Affichage */
    printf("Drone A : ID=%d (%.2f, %.2f, %.2f)\n", res.a->id, res.a->x, res.a->y, res.a->z);
    printf("Drone B : ID=%d (%.2f, %.2f, %.2f)\n", res.b->id, res.b->x, res.b->y, res.b->z);
    printf("Distance minimale : %.6f m\n", res.dist);
    printf("Temps : %.3f ms\n", ms);

    free(essaim);
    return 0;
}
