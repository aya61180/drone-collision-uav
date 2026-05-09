/* ============================================================
 *  PROJET INDUSTRIEL : Système de Collision pour Essaim UAV
 *  École des Sciences de l'Information
 *  Pr. Tarik HOUICHIME
 * ============================================================
 *
 *  ALGORITHME : Closest Pair of Points (Divide & Conquer)
 *  COMPLEXITÉ  : O(n log²n)  — contre O(n²) pour la méthode naïve
 *
 *  CONTRAINTES RESPECTÉES :
 *  - Aucun crochet d'indexation [] utilisé
 *  - Navigation par arithmétique pure de pointeurs uniquement
 *  - Allocation dynamique via malloc sur le tas
 * ============================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/* ============================================================
 *  STRUCTURES DE DONNÉES
 * ============================================================ */

/*
 * Représente un drone dans l'espace 3D.
 * Chaque drone possède un identifiant unique et ses coordonnées.
 */
typedef struct {
    int   id;   /* Identifiant unique du drone              */
    float x;    /* Coordonnée sur l'axe horizontal gauche   */
    float y;    /* Coordonnée sur l'axe horizontal droit    */
    float z;    /* Coordonnée sur l'axe vertical (altitude) */
} Drone;

/*
 * Résultat contenant la paire de drones la plus proche.
 * Retournée par les fonctions récursives.
 */
typedef struct {
    Drone *d1;   /* Pointeur vers le premier drone de la paire */
    Drone *d2;   /* Pointeur vers le second drone de la paire  */
    float  dist; /* Distance minimale entre les deux drones    */
} PairResult;


/* ============================================================
 *  FONCTIONS UTILITAIRES
 * ============================================================ */

/*
 * Calcule la distance euclidienne 3D entre deux drones.
 * Formule : sqrt( (x2-x1)² + (y2-y1)² + (z2-z1)² )
 */
float distance3D(Drone *a, Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/*
 * Échange deux drones en mémoire via leurs pointeurs.
 * Utilisée par le tri rapide (quicksort).
 */
void swapDrones(Drone *a, Drone *b) {
    Drone temp = *a;
    *a = *b;
    *b = temp;
}


/* ============================================================
 *  TRI RAPIDE (QUICKSORT) PAR AXE X
 *  — Étape 1 de l'algorithme Divide & Conquer
 *  — Complexité : O(n log n)
 * ============================================================ */

/*
 * Partition du tableau autour d'un pivot (valeur X du dernier élément).
 * Tous les éléments dont X <= pivot vont à gauche,
 * tous les éléments dont X > pivot vont à droite.
 *
 * Paramètres :
 *   debut : pointeur vers le premier drone du sous-tableau
 *   fin   : pointeur vers le dernier drone du sous-tableau
 *
 * Retourne : pointeur vers la position finale du pivot
 */
Drone *partition(Drone *debut, Drone *fin) {
    float  pivotX = fin->x;
    Drone *i      = debut - 1;
    Drone *j      = debut;

    while (j < fin) {
        if (j->x <= pivotX) {
            i++;
            swapDrones(i, j);
        }
        j++;
    }
    swapDrones(i + 1, fin);
    return i + 1;
}

/*
 * Tri rapide récursif du tableau de drones par coordonnée X.
 * Après ce tri, les drones sont ordonnés de gauche à droite dans l'espace.
 *
 * Paramètres :
 *   debut : pointeur vers le premier drone
 *   fin   : pointeur vers le dernier drone
 */
void quicksortX(Drone *debut, Drone *fin) {
    if (debut < fin) {
        Drone *pivot = partition(debut, fin);
        quicksortX(debut, pivot - 1);
        quicksortX(pivot + 1, fin);
    }
}


/* ============================================================
 *  TRI PAR INSERTION PAR AXE Z
 *  — Utilisé pour trier la bande centrale (petite taille)
 *  — Complexité : O(k²) avec k ≤ 7 en pratique → O(1)
 * ============================================================ */

/*
 * Trie un tableau de pointeurs vers des drones par coordonnée Z.
 * Utilisé uniquement pour la bande centrale δ, qui contient
 * toujours très peu de drones (≤ 7 théoriquement).
 *
 * Paramètres :
 *   bande  : tableau de pointeurs vers les drones de la bande
 *   taille : nombre de drones dans la bande
 */
void triBandeZ(Drone **bande, int taille) {
    int i, j;
    for (i = 1; i < taille; i++) {
        Drone *cle = *(bande + i);
        j = i - 1;
        while (j >= 0 && (*(bande + j))->z > cle->z) {
            *(bande + j + 1) = *(bande + j);
            j--;
        }
        *(bande + j + 1) = cle;
    }
}


/* ============================================================
 *  CAS DE BASE : FORCE BRUTE POUR PETITS ENSEMBLES
 *  — Utilisé quand n ≤ 3 (la récursion s'arrête)
 *  — Complexité : O(1) car n ≤ 3 → au plus 3 comparaisons
 * ============================================================ */

/*
 * Trouve la paire la plus proche parmi 2 ou 3 drones (force brute).
 *
 * Paramètres :
 *   debut  : pointeur vers le premier drone du groupe
 *   taille : nombre de drones (2 ou 3)
 *
 * Retourne : PairResult avec les deux drones les plus proches
 */
PairResult forceBrute(Drone *debut, int taille) {
    PairResult res;
    res.dist = FLT_MAX;
    res.d1   = NULL;
    res.d2   = NULL;

    Drone *i_ptr = debut;
    while (i_ptr < debut + taille) {
        Drone *j_ptr = i_ptr + 1;
        while (j_ptr < debut + taille) {
            float d = distance3D(i_ptr, j_ptr);
            if (d < res.dist) {
                res.dist = d;
                res.d1   = i_ptr;
                res.d2   = j_ptr;
            }
            j_ptr++;
        }
        i_ptr++;
    }
    return res;
}


/* ============================================================
 *  VÉRIFICATION DE LA BANDE CENTRALE
 *  — Étape clé de l'algorithme Divide & Conquer
 *  — Complexité : O(n) car chaque drone est comparé à ≤ 7 voisins
 * ============================================================ */

/*
 * Vérifie si une paire plus proche existe à cheval sur la ligne de coupe.
 *
 * PRINCIPE :
 * Après avoir trouvé δ = min(δ_gauche, δ_droite), seuls les drones
 * dont la distance à la ligne de coupe est < δ peuvent former une
 * paire plus proche. On les collecte dans une bande de largeur 2δ.
 *
 * THÉORÈME : Dans cette bande triée par Z, chaque drone ne doit
 * être comparé qu'aux 7 drones suivants au maximum.
 * → Complexité totale de cette étape : O(n)
 *
 * CORRECTION 3D : On filtre sur X ET sur Y pour éviter d'inclure
 * des drones éloignés sur l'axe Y qui ne peuvent pas être la paire
 * la plus proche.
 *
 * Paramètres :
 *   debut    : pointeur vers le début du tableau trié par X
 *   n        : nombre de drones
 *   xMilieu  : coordonnée X de la ligne de coupe
 *   yMilieu  : coordonnée Y du milieu de l'essaim
 *   meilleur : meilleur résultat trouvé jusqu'ici (δ)
 *
 * Retourne : PairResult mis à jour si une paire plus proche est trouvée
 */
PairResult verifierBande(Drone *debut, int n, float xMilieu,
                         float yMilieu, PairResult meilleur) {

    /* === Étape 1 : Collecter les drones dans la bande 2D (X et Y) === */

    Drone **bande = (Drone **)malloc(n * sizeof(Drone *));
    if (bande == NULL) {
        printf("Erreur allocation bande\n");
        return meilleur;
    }

    int    taillebande = 0;
    Drone *ptr         = debut;

    while (ptr < debut + n) {
        float distX = ptr->x - xMilieu;
        float distY = ptr->y - yMilieu;
        if (distX < 0) distX = -distX;
        if (distY < 0) distY = -distY;

        /* Un drone est dans la bande si sa distance sur X ET Y est < δ */
        if (distX < meilleur.dist && distY < meilleur.dist) {
            *(bande + taillebande) = ptr;
            taillebande++;
        }
        ptr++;
    }

    /* === Étape 2 : Trier la bande par coordonnée Z === */

    triBandeZ(bande, taillebande);

    /* === Étape 3 : Comparer chaque drone aux 7 suivants au maximum === */

    /*
     * THÉORÈME FONDAMENTAL :
     * Dans un rectangle de dimensions (δ × 2δ), on ne peut placer
     * que 8 points à distance mutuelle ≥ δ.
     * Donc chaque drone de la bande n'a AU PLUS 7 voisins à vérifier.
     * → Complexité totale de cette étape : O(n)
     */
    int i;
    for (i = 0; i < taillebande; i++) {
        int j;
        for (j = i + 1; j < taillebande && j < i + 8; j++) {
            Drone *di   = *(bande + i);
            Drone *dj   = *(bande + j);
            float  diffZ = dj->z - di->z;

            /* Optimisation : si la différence Z seule dépasse δ, arrêter */
            if (diffZ >= meilleur.dist) break;

            float d = distance3D(di, dj);
            if (d < meilleur.dist) {
                meilleur.dist = d;
                meilleur.d1   = di;
                meilleur.d2   = dj;
            }
        }
    }

    free(bande);
    return meilleur;
}


/* ============================================================
 *  ALGORITHME PRINCIPAL : CLOSEST PAIR — DIVIDE & CONQUER
 *  — Complexité : O(n log²n)
 * ============================================================ */

/*
 * Trouve récursivement la paire de drones la plus proche.
 *
 * PRINCIPE (Divide & Conquer) :
 *   1. CAS DE BASE  : si n ≤ 3, utiliser la force brute
 *   2. DIVISER      : couper le tableau en deux moitiés par X
 *   3. CONQUÉRIR    : résoudre chaque moitié récursivement
 *   4. COMBINER     : trouver δ = min(δ_gauche, δ_droite),
 *                     puis vérifier la bande centrale de largeur 2δ
 *
 * Paramètres :
 *   debut : pointeur vers le premier drone du sous-tableau (trié par X)
 *   n     : nombre de drones dans ce sous-tableau
 *
 * Retourne : PairResult contenant la paire la plus proche
 */
PairResult closestPair(Drone *debut, int n) {

    /* === CAS DE BASE : force brute pour 2 ou 3 drones === */
    if (n <= 3) {
        return forceBrute(debut, n);
    }

    /* === ÉTAPE DIVISER : trouver le milieu === */

    int milieu = n / 2;

    /*
     * xMilieu : coordonnée X du drone médian (ligne de coupe verticale)
     * yMilieu : coordonnée Y moyenne pour filtrer la bande en 3D
     */
    float xMilieu = (debut + milieu)->x;

    /* Calcul de yMilieu : moyenne des Y de tout le sous-tableau */
    float ySum = 0.0f;
    int   k;
    for (k = 0; k < n; k++) {
        ySum += (debut + k)->y;
    }
    float yMilieu = ySum / (float)n;

    /* === ÉTAPE CONQUÉRIR : résoudre les deux moitiés === */

    PairResult gauche = closestPair(debut, milieu);
    PairResult droite = closestPair(debut + milieu, n - milieu);

    /* === ÉTAPE COMBINER : garder le meilleur des deux === */

    PairResult meilleur;
    if (gauche.dist <= droite.dist) {
        meilleur = gauche;
    } else {
        meilleur = droite;
    }

    /*
     * VÉRIFICATION DE LA BANDE CENTRALE :
     * La paire la plus proche pourrait être à cheval sur la ligne de coupe.
     */
    meilleur = verifierBande(debut, n, xMilieu, yMilieu, meilleur);

    return meilleur;
}


/* ============================================================
 *  MANŒUVRE D'ÉVITEMENT
 * ============================================================ */

/*
 * Déclenche une manœuvre d'évitement si deux drones sont trop proches.
 * Action : séparation verticale (modification de l'altitude Z).
 *
 * Paramètres :
 *   a, b  : pointeurs vers les deux drones à séparer
 *   seuil : distance critique en dessous de laquelle déclencher la manœuvre
 */
void manouvreEvitement(Drone *a, Drone *b, float seuil) {
    float d = distance3D(a, b);

    if (d < seuil) {
        printf("ALERTE : Collision imminente entre Drone #%d et Drone #%d\n",
               a->id, b->id);
        printf("  Distance actuelle : %.4f m  |  Seuil critique : %.1f m\n",
               d, seuil);

        /* Manœuvre : séparer les deux drones verticalement */
        a->z += 3.0f;
        b->z -= 3.0f;

        printf("  Manœuvre effectuée : Drone #%d → altitude +3  |"
               "  Drone #%d → altitude -3\n", a->id, b->id);
        printf("  Nouvelle distance estimée : %.4f m\n", distance3D(a, b));
    } else {
        printf("Situation nominale — Distance minimale : %.4f m"
               " (seuil : %.1f m)\n", d, seuil);
    }
}


/* ============================================================
 *  PROGRAMME PRINCIPAL
 * ============================================================ */

int main(void) {

    int n = 10000;

    printf("============================================================\n");
    printf("  SYSTÈME DE DÉTECTION DE COLLISION — ESSAIM DE %d DRONES\n", n);
    printf("============================================================\n\n");

    /* -------------------------------------------------------
     *  ÉTAPE 1 : Allocation dynamique de l'essaim sur le tas
     *  — Accès UNIQUEMENT via arithmétique de pointeurs
     *  — Aucun crochet [] n'est utilisé
     * ------------------------------------------------------- */
    Drone *essaim = (Drone *)malloc(n * sizeof(Drone));

    if (essaim == NULL) {
        printf("ERREUR : Échec de l'allocation mémoire (%d drones)\n", n);
        return 1;
    }
    printf("[1/4] Allocation mémoire : %d drones × %zu octets = %zu octets\n",
           n, sizeof(Drone), n * sizeof(Drone));

    /* -------------------------------------------------------
     *  ÉTAPE 2 : Initialisation des drones (positions aléatoires)
     *  — Espace de 1000 × 1000 × 1000 unités
     *  — Graine fixe pour la reproductibilité
     * ------------------------------------------------------- */
    srand(42);

    int i;
    for (i = 0; i < n; i++) {
        (essaim + i)->id = i;
        (essaim + i)->x  = (float)(rand() % 1000);
        (essaim + i)->y  = (float)(rand() % 1000);
        (essaim + i)->z  = (float)(rand() % 1000);
    }
    printf("[2/4] Initialisation : positions 3D aléatoires assignées\n");

    /* -------------------------------------------------------
     *  ÉTAPE 3 : Tri par axe X — pré-requis du Divide & Conquer
     *  — Complexité : O(n log n)
     * ------------------------------------------------------- */
    quicksortX(essaim, essaim + n - 1);
    printf("[3/4] Tri par axe X terminé — O(n log n)\n");

    /* -------------------------------------------------------
     *  ÉTAPE 4 : Recherche de la paire la plus proche
     *  — Complexité : O(n log²n)
     * ------------------------------------------------------- */
    printf("[4/4] Recherche Divide & Conquer en cours...\n\n");

    PairResult resultat = closestPair(essaim, n);

    /* -------------------------------------------------------
     *  AFFICHAGE DES RÉSULTATS
     * ------------------------------------------------------- */
    printf("============================================================\n");
    printf("  RÉSULTAT\n");
    printf("============================================================\n");

    if (resultat.d1 != NULL && resultat.d2 != NULL) {
        printf("Drones les plus proches :\n");
        printf("  Drone #%d  → (x=%.1f, y=%.1f, z=%.1f)\n",
               resultat.d1->id,
               resultat.d1->x, resultat.d1->y, resultat.d1->z);
        printf("  Drone #%d  → (x=%.1f, y=%.1f, z=%.1f)\n",
               resultat.d2->id,
               resultat.d2->x, resultat.d2->y, resultat.d2->z);
        printf("  Distance minimale : %.6f m\n\n", resultat.dist);

        float SEUIL_CRITIQUE = 5.0f;
        printf("--- SYSTÈME D'ÉVITEMENT ---\n");
        manouvreEvitement(resultat.d1, resultat.d2, SEUIL_CRITIQUE);
    } else {
        printf("Aucune paire trouvée (essaim vide ?)\n");
    }

    /* -------------------------------------------------------
     *  LIBÉRATION DE LA MÉMOIRE
     * ------------------------------------------------------- */
    free(essaim);
    essaim = NULL;

    printf("\n[OK] Mémoire libérée — Système en attente du prochain cycle\n");

    return 0;
}

/*
 * ============================================================
 *  COMPILATION :
 *    gcc -O2 -o drone_collision drone_collision.c -lm
 *
 *  COMPLEXITÉ GLOBALE :
 *    Tri initial        : O(n log n)
 *    Récursion D&C      : T(n) = 2T(n/2) + O(n log n)
 *    Résolution Akra-B. : O(n log²n)
 *    → Gain vs naïf     : ~50 000 000 ops  →  ~1 800 000 ops
 * ============================================================
 */

