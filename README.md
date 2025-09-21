# Path Planning with RRT, RRT*, and PRM

**Author:** Almeqdad Khaled Alanesi  
**LinkedIn:** https://www.linkedin.com/in/almeqdad-alanesi-066191255/
**Email:** khalidalmegdad@gmail.com

**Purpose:** A concise, reproducible guide explaining RRT, RRT*, and PRM with runnable Python code for each algorithm. Use this as a GitHub README and explanatory document for your repository.

---

## Table of Contents
1. [Introduction](#1-introduction)  
2. [Common Definitions](#2-common-definitions)  
3. [Rapidly-Exploring Random Tree (RRT)](#3-rapidly-exploring-random-tree-rrt)  
   - [Intuition & Math](#intuition--math)  
   - [Algorithm Steps](#algorithm-steps)  
4. [RRT* (Optimal RRT)](#4-rrt-asymptotically-optimal-rrt)  
   - [Intuition & Math](#intuition--math-1)  
   - [Algorithm Steps](#algorithm-steps-1)  
5. [Probabilistic Roadmap (PRM)](#5-probabilistic-roadmap-prm)  
   - [Intuition & Math](#intuition--math-2)  
   - [Algorithm Steps](#algorithm-steps-2)  
6. [How to Run the Code](#6-how-to-run-the-code)  
7. [Visualization & Examples](#7-visualization--examples)  
8. [Extensions & Next Steps](#8-extensions--next-steps)  
9. [References](#9-references)  

---

## 1. Introduction
This repository demonstrates three widely used **sampling-based path-planning algorithms**:  

- **RRT (Rapidly-Exploring Random Tree)**  
- **RRT\*** (asymptotically optimal RRT)  
- **PRM (Probabilistic Roadmap)**  

Each algorithm is explained mathematically and implemented in Python with 2D visualizations for obstacles. The implementations are educational and focus on clarity over extreme optimization.

**Assumptions:**  
- 2D workspace, point robot (no orientation or footprint)  
- Circular or rectangular obstacles  
- Collision checking via sampling along straight-line segments  

---

## 2. Common Definitions
| Symbol | Meaning |
|--------|---------|
| `q_rand` | Random sample in free space `C_free` |
| `q_near` | Nearest node in the tree or graph to `q_rand` |
| `q_new` | New node steered toward `q_rand` |
| `delta` | Step size for extending the tree |
| `C_free` | Collision-free workspace |

**Helper Functions Across Algorithms**  
- Euclidean distance: `dist(p1, p2) = sqrt((x2-x1)^2 + (y2-y1)^2)`  
- Collision checking for segment `q1 -> q2` via sampling along the segment.

---

## 3. Rapidly-Exploring Random Tree (RRT)

### Intuition & Math
RRT grows a tree from the start point, randomly sampling the space and extending the nearest tree node toward each sample.

**Key Math:**
- Nearest neighbor: `q_near = argmin_{q in T} ||q - q_rand||`  
- Steer: `q_new = q_near + delta * (q_rand - q_near)/||q_rand - q_near||`  
- Collision check: `q(t) = q_near + t*(q_new - q_near) ∈ C_free, t ∈ [0,1]`  

**Property:** Probabilistically complete — as iterations → ∞, probability of finding a path → 1.

### Algorithm Steps
1. Initialize tree `T` with root `q_start`  
2. Repeat until goal reached or max iterations:  
   - Sample `q_rand ∈ C_free`  
   - Find nearest node `q_near ∈ T`  
   - Compute `q_new` by steering toward `q_rand`  
   - If collision-free, add `q_new` to `T`  
3. Extract path from start to goal  

---

## 4. RRT* (asymptotically optimal RRT)

### Intuition & Math
RRT* improves RRT by **rewiring** the tree to reduce path cost.  

**Key Math:**
- Near nodes: `Near(q_new) = { q ∈ T | ||q - q_new|| ≤ r(n) }`  
- Cost-to-come: `cost(q_new) = min_{q ∈ Near(q_new)} [ cost(q) + ||q - q_new|| ]`  
- Rewiring: if `cost(q_new) + ||q_new - q|| < cost(q)`, reassign `q`'s parent to `q_new`  

### Algorithm Steps
Same as RRT but with:  
- Choose lowest-cost parent  
- Rewire neighbors  

**Property:** Asymptotically optimal — path cost converges to optimal as samples → ∞.

---

## 5. Probabilistic Roadmap (PRM)

### Intuition & Math
PRM is a multi-query method:
1. Sample points (vertices) in free space  
2. Connect neighbors with collision-free edges to form a graph  
3. Solve start → goal queries via graph search  

**Key Math:**
- Vertices: `V = { q_i ∈ C_free | i = 1..N }`  
- Edges: `E = { (u,v) | u,v ∈ V, ||u-v|| ≤ r, segment uv ⊂ C_free }`  
- Shortest path: `path = argmin_{p in paths} sum(||u-v|| for (u,v) in p)`

### Algorithm Steps
1. Sample N collision-free points → vertices  
2. Connect each vertex to neighbors within radius if collision-free → edges  
3. Add start and goal, connect to roadmap  
4. Run Dijkstra/A* to find path  

**Property:** Efficient for multiple queries in the same static environment.

---

## 6. How to Run the Code

1. Install dependencies:  
```bash
pip install numpy matplotlib networkx scipy
```

```

2. Save scripts in the same folder: 
``` 
- `rrt_animated.py`  
- `rrt_star_animated.py`  
- `prm_animated.py`  

3. Run scripts individually in terminal/command prompt:  
```bash
python rrt_animated.py
python rrt_star_animated.py
python prm_animated.py
