"""
prm_animated.py
Probabilistic Roadmap (PRM) with obstacles.
Visualizes the sampled roadmap and the final shortest path.

Run: python prm_animated.py

Dependencies:
    pip install numpy matplotlib networkx
"""
import random
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import networkx as nx

# ---------- PARAMETERS ----------
X_LIMITS = (0, 10)
Y_LIMITS = (0, 10)
NUM_SAMPLES = 200       # number of random samples
CONNECTION_RADIUS = 1.5 # max distance to connect neighbors
# ---------------------------------

# Obstacles (mix of circles and rectangles)
obstacles = [
    {'type': 'circle', 'x': 3.0, 'y': 3.0, 'r': 1.0},
    {'type': 'circle', 'x': 6.0, 'y': 6.0, 'r': 0.8},
    {'type': 'circle', 'x': 4.5, 'y': 6.5, 'r': 0.6},
    {'type': 'rect', 'x': 1.0, 'y': 7.0, 'w': 2.2, 'h': 1.2},
    {'type': 'rect', 'x': 7.2, 'y': 1.5, 'w': 1.6, 'h': 2.1},
]

start = (0.5, 0.5)
goal = (9.0, 9.0)

# ---------- Collision checking ----------
def point_in_circle(x, y, cx, cy, r):
    return math.hypot(x - cx, y - cy) <= r

def point_in_rect(x, y, rx, ry, rw, rh):
    return (rx <= x <= rx + rw) and (ry <= y <= ry + rh)

def collision_free_point(p):
    x, y = p
    for obs in obstacles:
        if obs['type'] == 'circle':
            if point_in_circle(x, y, obs['x'], obs['y'], obs['r']):
                return False
        elif obs['type'] == 'rect':
            if point_in_rect(x, y, obs['x'], obs['y'], obs['w'], obs['h']):
                return False
    return True

def collision_free_segment(p1, p2, step=0.05):
    (x1, y1), (x2, y2) = p1, p2
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    steps = max(2, int(length / step))
    for i in range(steps + 1):
        t = i / steps
        x = x1 + t * dx
        y = y1 + t * dy
        if not collision_free_point((x, y)):
            return False
    return True

# ---------- Build PRM ----------
samples = []
while len(samples) < NUM_SAMPLES:
    p = (random.uniform(X_LIMITS[0], X_LIMITS[1]),
         random.uniform(Y_LIMITS[0], Y_LIMITS[1]))
    if collision_free_point(p):
        samples.append(p)

# add start and goal
samples.append(start)
samples.append(goal)

G = nx.Graph()
for i, p in enumerate(samples):
    G.add_node(i, pos=p)

# connect neighbors
for i in range(len(samples)):
    for j in range(i+1, len(samples)):
        p1, p2 = samples[i], samples[j]
        d = math.hypot(p1[0]-p2[0], p1[1]-p2[1])
        if d <= CONNECTION_RADIUS and collision_free_segment(p1, p2):
            G.add_edge(i, j, weight=d)

# shortest path with Dijkstra
start_idx = len(samples)-2
goal_idx = len(samples)-1
try:
    path_nodes = nx.shortest_path(G, source=start_idx, target=goal_idx, weight='weight')
    path = [samples[i] for i in path_nodes]
    print(f"Path found with {len(path)} nodes, length={nx.shortest_path_length(G, start_idx, goal_idx, weight='weight'):.2f}")
except nx.NetworkXNoPath:
    path = None
    print("No path found!")

# ---------- Visualization ----------
fig, ax = plt.subplots(figsize=(7,7))
ax.set_xlim(X_LIMITS)
ax.set_ylim(Y_LIMITS)
ax.set_title("PRM Roadmap")

# draw obstacles
for obs in obstacles:
    if obs['type'] == 'circle':
        circ = patches.Circle((obs['x'], obs['y']), obs['r'], color='gray', alpha=0.7)
        ax.add_patch(circ)
    else:
        rect = patches.Rectangle((obs['x'], obs['y']), obs['w'], obs['h'], color='gray', alpha=0.7)
        ax.add_patch(rect)

# draw roadmap
for (i,j) in G.edges():
    p1, p2 = samples[i], samples[j]
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='lightgreen', linewidth=0.7)

# draw samples
xs, ys = zip(*samples)
ax.scatter(xs, ys, color='blue', s=10)
ax.plot(start[0], start[1], marker='o', color='red', markersize=8, label='Start')
ax.plot(goal[0], goal[1], marker='*', color='orange', markersize=10, label='Goal')

# draw path if found
if path is not None:
    px, py = zip(*path)
    ax.plot(px, py, color='purple', linewidth=2.5, label='Shortest Path')

ax.legend()
plt.show()
