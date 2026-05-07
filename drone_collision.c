 /*
 * ============================================================
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
 *  STRUCTURE DE DONNÉES
 * ============================================================ */

/*
 * Structure représentant un drone dans l'espace 3D.
 * Chaque drone possède un identifiant unique et ses coordonnées.
 */
typedef struct {
    int   id;   /* Identifiant unique du drone              */
    float x;    /* Coordonnée sur l'axe horizontal gauche   */
    float y;    /* Coordonnée sur l'axe horizontal droit    */
    float z;    /* Coordonnée sur l'axe vertical (altitude) */
} Drone;

/*
 * Structure résultat contenant la paire de drones la plus proche.
 * Retournée par les fonctions récursives.
 */
typedef struct {
    Drone *d1;      /* Pointeur vers le premier drone de la paire  */
    Drone *d2;      /* Pointeur vers le second drone de la paire   */
    float  dist;    /* Distance minimale entre les deux drones      */
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
 * Tous les éléments dont X < pivot vont à gauche,
 * tous les éléments dont X >= pivot vont à droite.
 *
 * Paramètres :
 *   debut  : pointeur vers le premier drone du sous-tableau
 *   fin    : pointeur vers le dernier drone du sous-tableau
 *
 * Retourne : pointeur vers la position finale du pivot
 */
Drone *partition(Drone *debut, Drone *fin) {
    /* Le pivot est le dernier élément du sous-tableau */
    float pivotX = fin->x;

    /*
     * i pointe vers la "frontière gauche" :
     * tout ce qui est à gauche de i a un X <= pivotX
     */
    Drone *i = debut - 1;
    Drone *j = debut;

    /* Parcours du sous-tableau sans crochets */
    while (j < fin) {
        if (j->x <= pivotX) {
            i++;                  /* Avancer la frontière */
            swapDrones(i, j);     /* Placer l'élément à gauche */
        }
        j++;
    }

    /* Placer le pivot à sa position définitive */
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
        /* Trouver la position du pivot */
        Drone *pivot = partition(debut, fin);

        /* Trier récursivement la moitié gauche */
        quicksortX(debut, pivot - 1);

        /* Trier récursivement la moitié droite */
        quicksortX(pivot + 1, fin);
    }
}


/* ============================================================
 *  TRI PAR INSERTION PAR AXE Z
 *  — Utilisé pour trier la bande centrale (petite taille)
 *  — Complexité : O(k²) avec k très petit (~7 au maximum)
 * ============================================================ */

/*
 * Trie un tableau de pointeurs vers des drones par coordonnée Z.
 * Ce tri est utilisé uniquement pour la bande centrale δ,
 * qui contient toujours très peu de drones (≤ 7 théoriquement).
 *
 * Paramètres :
 *   bande : tableau de pointeurs vers les drones de la bande
 *   taille : nombre de drones dans la bande
 */void triBandeZ(Drone **bande, int taille) {
    int i, j;
    for (i = 1; i < taille; i++) {
        Drone *cle = *(bande + i);
        j = i - 1;
        /* Décaler les éléments plus grands vers la droite */
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
 * Ce cas de base arrête la récursion et est appelé très fréquemment
 * sur de tout petits sous-tableaux.
 *
 * Paramètres :
 *   debut  : pointeur vers le premier drone du groupe
 *   taille : nombre de drones (2 ou 3)
 *
 * Retourne : PairResult avec les deux drones les plus proches
 */
PairResult forceBrute(Drone *debut, int taille) {
    PairResult res;
    res.dist = FLT_MAX;  /* Initialiser avec la distance maximale possible */
    res.d1   = NULL;
    res.d2   = NULL;

    /* Comparer chaque paire — au plus 3 paires pour taille=3 */
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
 * EXPLICATION :
 * Après avoir trouvé δ = min(δ_gauche, δ_droite),
 * seuls les drones dont la distance à la ligne de coupe est < δ
 * peuvent potentiellement former une paire plus proche.
 * On les collecte dans une "bande" de largeur 2δ centrée sur la ligne.
 *
 * THÉORÈME : Dans cette bande triée par Z, chaque drone
 * ne doit être comparé qu'aux 7 drones suivants au maximum.
 *
 * Paramètres :
 *   debut      : pointeur vers le début du tableau trié par X
 *   n          : nombre de drones
 *   xMilieu    : coordonnée X de la ligne de coupe
 *   meilleur   : meilleur résultat trouvé jusqu'ici (δ)
 *
 * Retourne : PairResult mis à jour si une paire plus proche est trouvée
 */
PairResult verifierBande(Drone *debut, int n, float xMilieu, PairResult meilleur) {

    /* === Étape 1 : Collecter les drones dans la bande de largeur 2δ === */

    /*
     * Allouer un tableau de pointeurs pour les drones de la bande.
     * Au pire cas, tous les drones sont dans la bande.
     */
    Drone **bande = (Drone **)malloc(n * sizeof(Drone *));
    if (bande == NULL) {
        printf("Erreur allocation bande\n");
        return meilleur;
    }

    int taillebande = 0;
    Drone *ptr = debut;

    /* Parcourir tous les drones et sélectionner ceux dans la bande */
    while (ptr < debut + n) {
        /* Un drone est dans la bande si sa distance à xMilieu est < δ */
        float distX = ptr->x - xMilieu;
        if (distX < 0) distX = -distX;  /* Valeur absolue sans fabsf */

        if (distX < meilleur.dist) {
            /* Ajouter ce drone à la bande (via pointeur) */
            *(bande + taillebande) = ptr;
            taillebande++;
        }
        ptr++;
    }

    /* === Étape 2 : Trier la bande par coordonnée Z === */

    /*
     * On trie par Z pour pouvoir arrêter les comparaisons dès que
     * la différence de Z dépasse δ (optimisation clé).
     */
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
            Drone *di = *(bande + i);
            Drone *dj = *(bande + j);

            /* Optimisation : si la différence Z seule dépasse δ, arrêter */
            float diffZ = dj->z - di->z;
            if (diffZ >= meilleur.dist) break;

            float d = distance3D(di, dj);
            if (d < meilleur.dist) {
                meilleur.dist = d;
                meilleur.d1   = di;
                meilleur.d2   = dj;
            }
        }
    }

    /* Libérer la mémoire temporaire de la bande */
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
 *   2. DIVISER      : couper le tableau en deux moitiés (gauche/droite par X)
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
     * xMilieu est la coordonnée X du drone médian.
     * C'est la "ligne de coupe" verticale qui sépare les deux moitiés.
     * On utilise l'arithmétique de pointeurs : (debut + milieu)->x
     */
    float xMilieu = (debut + milieu)->x;

    /* === ÉTAPE CONQUÉRIR : résoudre les deux moitiés === */

    /* Résoudre la moitié gauche [debut .. debut+milieu-1] */
    PairResult gauche = closestPair(debut, milieu);

    /* Résoudre la moitié droite [debut+milieu .. debut+n-1] */
    PairResult droite = closestPair(debut + milieu, n - milieu);

    /* === ÉTAPE COMBINER : garder le meilleur des deux === */

    /*
     * δ = distance minimale trouvée dans les deux moitiés.
     * On ne garde qu'un seul résultat (le meilleur).
     */
    PairResult meilleur;
    if (gauche.dist <= droite.dist) {
        meilleur = gauche;
    } else {
        meilleur = droite;
    }

    /*
     * VÉRIFICATION DE LA BANDE CENTRALE :
     * La paire la plus proche pourrait être à cheval sur la ligne de coupe.
     * On vérifie uniquement les drones dans la bande [xMilieu-δ, xMilieu+δ].
     */
    meilleur = verifierBande(debut, n, xMilieu, meilleur);

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
        a->z += 3.0f;   /* Drone A monte de 3 unités  */
        b->z -= 3.0f;   /* Drone B descend de 3 unités */

        printf("  Manœuvre effectuée : Drone #%d → altitude +3  |"
               "  Drone #%d → altitude -3\n", a->id, b->id);
        printf("  Nouvelle distance estimée : %.4f m\n", distance3D(a, b));
    } else {
        printf("Situation nominale — Distance minimale : %.4f m (seuil : %.1f m)\n",
               d, seuil);
    }
}


/* ============================================================
 *  PROGRAMME PRINCIPAL
 * ============================================================ */

int main(void) {

    /* Nombre de drones dans l'essaim */
    int n = 10000;

    printf("============================================================\n");
    printf("  SYSTÈME DE DÉTECTION DE COLLISION — ESSAIM DE %d DRONES\n", n);
    printf("============================================================\n\n");

    /* -------------------------------------------------------
     *  ÉTAPE 1 : Allocation dynamique de l'essaim sur le tas
     * ------------------------------------------------------- */

    /*
     * On alloue un bloc continu de n structures Drone.
     * L'accès se fera UNIQUEMENT via arithmétique de pointeurs.
     * Aucun crochet [] n'est utilisé.
     */
    Drone *essaim = (Drone *)malloc(n * sizeof(Drone));

    if (essaim == NULL) {
        printf("ERREUR : Échec de l'allocation mémoire (%d drones)\n", n);
        return 1;
    }
    printf("[1/4] Allocation mémoire : %d drones × %zu octets = %zu octets\n",
           n, sizeof(Drone), n * sizeof(Drone));

    /* -------------------------------------------------------
     *  ÉTAPE 2 : Initialisation des drones (positions aléatoires)
     * ------------------------------------------------------- */

    /*
     * Utilisation de l'arithmétique de pointeurs : (essaim + i)->champ
     * Aucun crochet [] n'est utilisé.
     * Les coordonnées sont générées dans un espace de 1000×1000×1000 unités.
     */
    srand(42);  /* Graine fixe pour la reproductibilité */

    int i;
    for (i = 0; i < n; i++) {
        (essaim + i)->id = i;
        (essaim + i)->x  = (float)(rand() % 1000);
        (essaim + i)->y  = (float)(rand() % 1000);
        (essaim + i)->z  = (float)(rand() % 1000);
    }
    printf("[2/4] Initialisation : positions 3D aléatoires assignées\n");

    /* -------------------------------------------------------
     *  ÉTAPE 3 : Tri par axe X (pré-requis du Divide & Conquer)
     * ------------------------------------------------------- */

    /*
     * Le tri par X est INDISPENSABLE avant la récursion.
     * Il permet de diviser l'espace en deux moitiés bien séparées
     * et de concentrer les comparaisons sur les voisins réels.
     * Complexité : O(n log n)
     */
    quicksortX(essaim, essaim + n - 1);
    printf("[3/4] Tri par axe X terminé — O(n log n)\n");

    /* -------------------------------------------------------
     *  ÉTAPE 4 : Recherche de la paire la plus proche
     * ------------------------------------------------------- */

    /*
     * Application de l'algorithme Closest Pair of Points.
     * Complexité : O(n log²n) — bien inférieure à O(n²) naïf.
     */
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
               resultat.d1->id, resultat.d1->x, resultat.d1->y, resultat.d1->z);
        printf("  Drone #%d  → (x=%.1f, y=%.1f, z=%.1f)\n",
               resultat.d2->id, resultat.d2->x, resultat.d2->y, resultat.d2->z);
        printf("  Distance minimale : %.6f m\n\n", resultat.dist);

        /* Déclencher la manœuvre si la distance est critique */
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
    essaim = NULL;  /* Bonne pratique : éviter les pointeurs fantômes */

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
 *    → Gain vs naïf     : 50 000 000 ops  →  ~1 800 000 ops
 * ============================================================
 */

