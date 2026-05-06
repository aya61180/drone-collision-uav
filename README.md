# 🚀 UAV Collision Detection System

## 📌 Description
This project implements a **Closest Pair of Points** algorithm using the Divide & Conquer method to detect potential collisions in a 3D drone swarm. 

## ⚙️ Technical Features
- **3D Euclidean distance** calculation.
- **Optimized Complexity:** $O(n \log^2 n)$ compared to the naive $O(n^2)$ approach.
- **Strict Constraints:** Pointer arithmetic only (no array indexing), and dynamic heap allocation.
- **Collision Avoidance System:** Automatic vertical separation when drones are closer than the critical threshold.

## 🛠️ Compilation and Execution

### Compilation
```bash
gcc -O2 -o drone_collision drone_collision.c -lm