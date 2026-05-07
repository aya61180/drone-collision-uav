 Drone Collision Detection — UAV Swarm

** Aya Kourti
ESI Rabat
Pr. Tarik HOUICHIME

---

 Description

Algorithme en C qui détecte les **2 drones les plus proches** parmi 10 000 drones en temps réel.

##  Pourquoi pas la double boucle ?

| Méthode | Complexité | Résultat |
|---|---|---|
| Double boucle | O(n²) | ❌ 50M opérations — Timeout |
| **Divide & Conquer** | **O(n log²n)** | **✅ 1.8M opérations** |

## 🔧 Algorithme

```
1. Trier par X      →  QuickSort       O(n log n)
2. Diviser          →  Moitié G / D    O(1)
3. Conquérir        →  Récursion       O(n log²n)
4. Bande centrale   →  Vérifier δ      O(n)
```

##  Contraintes

- Aucun crochet `[]` — pointeurs uniquement
- Allocation dynamique `malloc`
- Code entièrement commenté

## Compilation

```bash
gcc -O2 -o drone drone_collision.c -lm
./drone
```
