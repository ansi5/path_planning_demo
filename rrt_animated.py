"""
rrt_animated.py
Animated RRT (2D) with circular and rectangular obstacles.

Run: python rrt_animated.py

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
STEP_SIZE = 0.4
MAX_ITER = 4000
GOAL_SAMPLE_RATE = 0.05   # probability of sampling the goal (goal bias)
GOAL_THRESHOLD = 0.5
ANIM_INTERVAL_MS = 10     # animation update interval in ms
SAMPLES_PER_FRAME = 4     # how many RRT iterations per animation frame
# --------------------------------

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

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
    new.parent = from_node
    return new

def point_in_circle(x, y, cx, cy, r):
    return math.hypot(x - cx, y - cy) <= r

def point_in_rect(x, y, rx, ry, rw, rh):
    # axis-aligned rectangle whose corner is (rx, ry) and width rw, height rh
    return (rx <= x <= rx + rw) and (ry <= y <= ry + rh)

def collision_free_segment(p1, p2, obstacles, step=0.02):
    # sample along segment and ensure no sample is inside any obstacle
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

# ---------- Workspace / Obstacles ----------
# Use mixture of circles and rectangles
obstacles = [
    {'type': 'circle', 'x': 3.0, 'y': 3.0, 'r': 1.0},
    {'type': 'circle', 'x': 6.0, 'y': 6.0, 'r': 0.8},
    {'type': 'circle', 'x': 4.5, 'y': 6.5, 'r': 0.6},
    {'type': 'rect', 'x': 1.0, 'y': 7.0, 'w': 2.2, 'h': 1.2},
    {'type': 'rect', 'x': 7.2, 'y': 1.5, 'w': 1.6, 'h': 2.1},
]

start = Node(0.0, 0.0)
goal = Node(8.5, 8.0)
# -------------------------------------------

# Basic RRT state
tree = [start]
edges = []  # list of (x1,y1,x2,y2) for plotting
found_goal = False
iter_count = 0

# Matplotlib setup
fig, ax = plt.subplots(figsize=(7,7))
ax.set_xlim(X_LIMITS)
ax.set_ylim(Y_LIMITS)
ax.set_title("RRT Animated (green: tree, blue: start, red: goal/path)")
start_plot = ax.plot(start.x, start.y, marker='o', color='blue', markersize=8)[0]
goal_plot = ax.plot(goal.x, goal.y, marker='*', color='red', markersize=10)[0]

# Draw obstacles
patches_list = []
for obs in obstacles:
    if obs['type'] == 'circle':
        p = patches.Circle((obs['x'], obs['y']), obs['r'], color='gray', alpha=0.7)
    else:
        p = patches.Rectangle((obs['x'], obs['y']), obs['w'], obs['h'], color='gray', alpha=0.7)
    patches_list.append(p)
    ax.add_patch(p)

# lines for tree edges
lines = []
# final path line (when found)
path_line, = ax.plot([], [], color='orange', linewidth=2.5)

def sample_random():
    if random.random() < GOAL_SAMPLE_RATE:
        return (goal.x, goal.y)
    else:
        return (random.uniform(X_LIMITS[0], X_LIMITS[1]), random.uniform(Y_LIMITS[0], Y_LIMITS[1]))

def extend_rrt():
    global found_goal, iter_count
    if found_goal or iter_count >= MAX_ITER:
        return
    iter_count += 1

    rnd = sample_random()
    nearest = min(tree, key=lambda n: math.hypot(n.x - rnd[0], n.y - rnd[1]))
    new_node = steer(nearest, rnd, STEP_SIZE)
    if collision_free_segment(nearest, new_node, obstacles):
        tree.append(new_node)
        edges.append((nearest.x, nearest.y, new_node.x, new_node.y))
        # check goal
        if dist(new_node, goal) <= GOAL_THRESHOLD and collision_free_segment(new_node, goal, obstacles):
            goal.parent = new_node
            tree.append(goal)
            edges.append((new_node.x, new_node.y, goal.x, goal.y))
            found_goal = True
            print(f"Goal reached in {iter_count} iterations")

def build_path():
    # reconstruct path from goal to start if found
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
    # perform multiple RRT iterations per animation frame for speed
    for _ in range(SAMPLES_PER_FRAME):
        extend_rrt()
    # redraw tree edges (clear old)
    for ln in lines:
        ln.remove()
    lines.clear()
    for (x1,y1,x2,y2) in edges:
        ln, = ax.plot([x1,x2], [y1,y2], color='green', linewidth=0.8)
        lines.append(ln)
    # if path found, draw it
    if found_goal:
        path = build_path()
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        path_line.set_data(xs, ys)
    return lines + [path_line]

# Create animation
ani = FuncAnimation(fig, update, frames=2000, interval=ANIM_INTERVAL_MS, blit=False, repeat=False)
plt.show()
