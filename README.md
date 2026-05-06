# Projet Industriel : Système de Collision pour Essaim Autonome (UAV)

**Auteur :** Aya Kourti  
**Institution :** École des Sciences de l'Information (ESI)  
**Encadrant :** Pr. Tarik HOUICHIME  

---

## 📌 Description du Projet

Ce projet industriel a pour but de concevoir un algorithme embarqué en langage C permettant d'identifier instantanément les deux drones les plus proches dans un essaim de 10 000 unités. L'objectif est d'éviter les collisions en vol en temps réel (prévention de crash en chaîne) tout en respectant des contraintes matérielles strictes.

## ⚙️ Cahier des Charges & Contraintes

* **Topologie de la donnée :** Utilisation d'une structure hétérogène `struct Drone` contenant un identifiant et des coordonnées spatiales en virgule flottante 3D (x, y, z).
* **Gestion de la mémoire :** Allocation dynamique contiguë via `malloc`.
* **Sécurité & Compilation :** L'utilisation des crochets d'indexation (`[]`) est strictement interdite. Toutes les manipulations se font via l'arithmétique pure des pointeurs.

## 📂 Livrables de l'Ingénieur

### 1. Code Source Industriel
Le code source est disponible dans le fichier `drone_collision.c`. Il implémente le calcul de la distance euclidienne en $O(n^2)$ :

$$T(n) = \frac{n(n-1)}{2} \in O(n^2)$$

### 2. Conception Technique
* Optimisation des calculs de distance grâce à la fonction mathématique `sqrtf`.
* Utilisation des pointeurs pour parcourir la mémoire contiguë sans indexation.

### 3. Audit Asymptotique et Preuve
Pour un volume de $N = 10 000$ drones, le nombre de calculs est de l'ordre de $5 \times 10^7$ opérations, optimisé pour s'exécuter dans les délais du processeur embarqué.
