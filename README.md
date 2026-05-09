# Système de Détection de Collision — Essaim UAV

**École des Sciences de l'Information — Pr. Tarik HOUICHIME**

---

## C'est quoi ce projet ?

Un algorithme qui surveille **10 000 drones** en vol et détecte en temps réel les deux drones les plus proches l'un de l'autre, puis déclenche une manœuvre d'évitement automatique.

---

## Pourquoi pas une double boucle simple ?

La double boucle naïve fait **~50 millions d'opérations** pour 10 000 drones — trop lent pour un système embarqué temps réel.

Notre algorithme **Divide & Conquer** fait seulement **~1 800 000 opérations** grâce à une complexité O(n log²n) au lieu de O(n²).

---

## Comment ça marche ?

```
1. malloc()              →  allouer 10 000 drones en mémoire
2. quicksortX()          →  trier par axe X           O(n log n)
3. closestPair()         →  trouver la paire la plus proche  O(n log²n)
4. manouvreEvitement()   →  séparer les deux drones verticalement
```

### L'idée de closestPair

```
[--------10 000 drones--------]
          ↙              ↘
  [gauche 5000]     [droite 5000]
      ↙    ↘             ↙    ↘
  [2500] [2500]      [2500] [2500]
    ...    ...          ...   ...
   [3]    [2]          [3]   [2]   ← forceBrute() ici
```

On divise jusqu'à avoir 2 ou 3 drones, on compare directement, puis on remonte le meilleur résultat vers le haut.

---

## Contraintes respectées

- **Zéro crochet `[]`** — navigation uniquement par pointeurs : `(essaim + i)->x`
- **malloc sur le tas** — tout l'essaim alloué dynamiquement
- **free()** — mémoire libérée proprement à la fin

---

## Les fonctions principales

**`quicksortX()`** — trie tous les drones par coordonnée X en O(n log n). C'est le pré-requis indispensable avant la récursion.

**`forceBrute()`** — le cas de base. Appelée quand il reste 2 ou 3 drones, elle compare directement toutes les paires en O(1).

**`verifierBande()`** — après chaque division, vérifie si une paire plus proche existe à cheval sur la ligne de coupe, en O(n).

**`closestPair()`** — l'algorithme principal récursif. Divise, conquiert, combine. Complexité globale O(n log²n).

**`manouvreEvitement()`** — si la distance est sous le seuil critique (5m), monte un drone de +3 et descend l'autre de -3.

---

## Compilation & Exécution

```bash
gcc -O2 -o drone_collision drone_collision.c -lm
./drone_collision
```
```
