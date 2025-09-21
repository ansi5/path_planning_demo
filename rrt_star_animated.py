"""
rrt_star_animated.py
Animated RRT* (2D) with circular and rectangular obstacles and rewiring.

Run: python rrt_star_animated.py

Dependencies:
    pip install numpy matplotlib
"""
import math
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# ---------- PARAMETERS ----------
X_LIMITS = (-1, 10)
Y_LIMITS = (-1, 10)
STEP_SIZE = 0.5
MAX_ITER = 3500
GOAL_SAMPLE_RATE = 0.06
GOAL_THRESHOLD = 0.5
NEIGHBOR_RADIUS = 1.2   # radius to search neighbors for rewiring
ANIM_INTERVAL_MS = 12
SAMPLES_PER_FRAME = 3
# --------------------------------

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

def dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def steer(from_node, to_point, step_size):
    dx = to_point[0] - from_node.x
    dy = to_point[1] - from_node.y
    d = math.hypot(dx, dy)
    if d <= step_size:
        new = Node(to_point[0], to_point[1])
    else:
        theta = math.atan2(dy, dx)
        new = Node(from_node.x + step_size * math.cos(theta),
                   from_node.y + step_size * math.sin(theta))
    return new

def point_in_circle(x, y, cx, cy, r):
    return math.hypot(x - cx, y - cy) <= r

def point_in_rect(x, y, rx, ry, rw, rh):
    return (rx <= x <= rx + rw) and (ry <= y <= ry + rh)

def collision_free_segment(p1, p2, obstacles, step=0.02):
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    length = math.hypot(dx, dy)
    steps = max(2, int(length / step))
    for i in range(steps + 1):
        t = i / steps
        x = p1.x + t * dx
        y = p1.y + t * dy
        for obs in obstacles:
            if obs['type'] == 'circle':
                if point_in_circle(x, y, obs['x'], obs['y'], obs['r']):
                    return False
            elif obs['type'] == 'rect':
                if point_in_rect(x, y, obs['x'], obs['y'], obs['w'], obs['h']):
                    return False
    return True

# ---------- Workspace ----------
obstacles = [
    {'type': 'circle', 'x': 3.0, 'y': 3.0, 'r': 1.0},
    {'type': 'circle', 'x': 6.0, 'y': 6.0, 'r': 0.8},
    {'type': 'circle', 'x': 4.5, 'y': 6.5, 'r': 0.6},
    {'type': 'rect', 'x': 1.0, 'y': 7.0, 'w': 2.2, 'h': 1.2},
    {'type': 'rect', 'x': 7.2, 'y': 1.5, 'w': 1.6, 'h': 2.1},
]
start = Node(0.0, 0.0)
goal = Node(8.5, 8.0)
# ---------------------------------

nodes = [start]
edges = []  # (x1,y1,x2,y2)
found_goal = False
iter_count = 0

# matplotlib setup
fig, ax = plt.subplots(figsize=(7,7))
ax.set_xlim(X_LIMITS)
ax.set_ylim(Y_LIMITS)
ax.set_title("RRT* Animated (green: tree, blue: start, red: goal/path)")
ax.plot(start.x, start.y, marker='o', color='blue', markersize=8)
ax.plot(goal.x, goal.y, marker='*', color='red', markersize=10)

# draw obstacles
for obs in obstacles:
    if obs['type'] == 'circle':
        p = patches.Circle((obs['x'], obs['y']), obs['r'], color='gray', alpha=0.7)
    else:
        p = patches.Rectangle((obs['x'], obs['y']), obs['w'], obs['h'], color='gray', alpha=0.7)
    ax.add_patch(p)

lines = []
path_line, = ax.plot([], [], color='orange', linewidth=2.5)

def sample_random():
    if random.random() < GOAL_SAMPLE_RATE:
        return (goal.x, goal.y)
    else:
        return (random.uniform(X_LIMITS[0], X_LIMITS[1]), random.uniform(Y_LIMITS[0], Y_LIMITS[1]))

def near_nodes(new_node, nodes_list, radius):
    return [n for n in nodes_list if math.hypot(n.x - new_node.x, n.y - new_node.y) <= radius]

def extend_rrt_star():
    global found_goal, iter_count
    if found_goal or iter_count >= MAX_ITER:
        return
    iter_count += 1

    rnd = sample_random()
    nearest = min(nodes, key=lambda n: math.hypot(n.x - rnd[0], n.y - rnd[1]))
    new = steer(nearest, rnd, STEP_SIZE)
    if not collision_free_segment(nearest, new, obstacles):
        return
    # neighbors for parent selection and rewiring
    neighbors = near_nodes(new, nodes, NEIGHBOR_RADIUS)
    # choose best parent (min cost)
    best_parent = nearest
    best_cost = nearest.cost + dist(nearest, new)
    for n in neighbors:
        if collision_free_segment(n, new, obstacles):
            c = n.cost + dist(n, new)
            if c < best_cost:
                best_cost = c
                best_parent = n
    new.parent = best_parent
    new.cost = best_cost
    nodes.append(new)
    edges.append((best_parent.x, best_parent.y, new.x, new.y))
    # rewire neighbors through new if it improves their cost
    for n in neighbors:
        if n is new.parent:
            continue
        if collision_free_segment(new, n, obstacles):
            new_cost = new.cost + dist(new, n)
            if new_cost < n.cost:
                # change parent and update cost (this won't recursively update descendants' costs fully, but works for visualization)
                old_parent = n.parent
                n.parent = new
                n.cost = new_cost
                # update edges list: remove edge from old_parent->n and add new->n
                try:
                    edges.remove((old_parent.x, old_parent.y, n.x, n.y))
                except ValueError:
                    # sometimes the exact tuple not present due to floating differences; do a search
                    for e in edges:
                        if abs(e[0]-old_parent.x)<1e-6 and abs(e[1]-old_parent.y)<1e-6 and abs(e[2]-n.x)<1e-6 and abs(e[3]-n.y)<1e-6:
                            edges.remove(e)
                            break
                edges.append((new.x, new.y, n.x, n.y))
    # check goal
    if dist(new, goal) <= GOAL_THRESHOLD and collision_free_segment(new, goal, obstacles):
        goal.parent = new
        goal.cost = new.cost + dist(new, goal)
        nodes.append(goal)
        edges.append((new.x, new.y, goal.x, goal.y))
        found_goal = True
        print(f"Goal connected at iter {iter_count}, goal cost {goal.cost:.2f}")

def build_path():
    if not found_goal:
        return None
    path = []
    node = goal
    while node is not None:
        path.append((node.x, node.y))
        node = node.parent
    path.reverse()
    return path

def update(frame):
    # run several iterations per frame
    for _ in range(SAMPLES_PER_FRAME):
        extend_rrt_star()
    # redraw edges
    for ln in lines:
        ln.remove()
    lines.clear()
    for (x1,y1,x2,y2) in edges:
        ln, = ax.plot([x1,x2], [y1,y2], color='green', linewidth=0.8)
        lines.append(ln)
    # draw path if exists
    if found_goal:
        path = build_path()
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        path_line.set_data(xs, ys)
    return lines + [path_line]

ani = FuncAnimation(fig, update, frames=2000, interval=ANIM_INTERVAL_MS, blit=False, repeat=False)
plt.show()
