#!/usr/bin/env python3

import rospy
import pygame
import random
import time
import os
import csv
import math
from std_msgs.msg import Bool
from collections import deque

# ROS node
rospy.init_node('target_hit_game_node')
pub = rospy.Publisher('/target_hit', Bool, queue_size=10)
entropy_pub = rospy.Publisher('/start_entropy', Bool, queue_size=1)

# Pygame setup
pygame.init()
width, height = 800, 600
win = pygame.display.set_mode((width, height))
pygame.display.set_caption("Target Maze Game")
font = pygame.font.Font(None, 36)
clock = pygame.time.Clock()

# Maze params
cols, rows = 20, 15
cell_size = 40
maze = [[1 for _ in range(cols)] for _ in range(rows)]
visited = [[False for _ in range(cols)] for _ in range(rows)]

# Maze generation (Perfect Maze using DFS)
def generate_maze(x, y):
    visited[y][x] = True
    maze[y][x] = 0
    directions = [(0,1),(1,0),(0,-1),(-1,0)]
    random.shuffle(directions)
    for dx, dy in directions:
        nx, ny = x + dx*2, y + dy*2
        if 0 <= nx < cols and 0 <= ny < rows and not visited[ny][nx]:
            maze[y + dy][x + dx] = 0
            generate_maze(nx, ny)

# Validate goal reachability using BFS
def is_reachable(start, goal):
    queue = deque([start])
    visited_bfs = set()
    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < cols and 0 <= ny < rows and maze[ny][nx] == 0 and (nx, ny) not in visited_bfs:
                visited_bfs.add((nx, ny))
                queue.append((nx, ny))
    return False

# Game state
duration = 180  # 3 minutes

# Results path
timestamp = time.strftime("%Y%m%d_%H%M%S")
result_dir = f"/home/ros/eyegaze_ws/results/target_game_result/session_{timestamp}"
os.makedirs(result_dir, exist_ok=True)

# Track path
path_log = []

# Countdown
for i in range(3, 0, -1):
    win.fill((0, 0, 0))
    text = font.render(str(i), True, (255, 255, 255))
    win.blit(text, (width // 2 - 10, height // 2 - 10))
    pygame.display.update()
    time.sleep(1)

text = font.render("GO!", True, (0, 255, 0))
win.fill((0, 0, 0))
win.blit(text, (width // 2 - 30, height // 2 - 10))
pygame.display.update()
    
# Init
generate_maze(0, 0)
player_x, player_y = 0, 0

# Pick valid target
def pick_valid_target():
    possible_targets = [(x, y) for y in range(rows) for x in range(cols) if maze[y][x] == 0 and (x, y) != (player_x, player_y)]
    random.shuffle(possible_targets)
    for tx, ty in possible_targets:
        if is_reachable((player_x, player_y), (tx, ty)):
            return tx, ty
    return player_x, player_y  # fallback

target_x, target_y = pick_valid_target()

# Helper to draw maze
def draw_maze():
    for y in range(rows):
        for x in range(cols):
            rect = pygame.Rect(x * cell_size, y * cell_size, cell_size, cell_size)
            color = (0, 0, 0) if maze[y][x] == 0 else (100, 100, 100)
            pygame.draw.rect(win, color, rect)

# Helper to replay path visually
def draw_path():
    if len(path_log) < 2:
        return
    for i in range(1, len(path_log)):
        pygame.draw.line(win, (0, 0, 255), path_log[i - 1], path_log[i], 2)

# Entropy signal
rospy.set_param("/current_game_type", "target")
entropy_pub.publish(True)

start_time = time.time()
running = True
goals_reached = 0

while running and not rospy.is_shutdown():
    win.fill((0, 0, 0))
    draw_maze()
    draw_path()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    dx, dy = 0, 0
    if keys[pygame.K_LEFT]: dx = -1
    elif keys[pygame.K_RIGHT]: dx = 1
    elif keys[pygame.K_UP]: dy = -1
    elif keys[pygame.K_DOWN]: dy = 1

    if dx or dy:
        nx, ny = player_x + dx, player_y + dy
        if 0 <= nx < cols and 0 <= ny < rows and maze[ny][nx] == 0:
            player_x, player_y = nx, ny
            path_log.append((player_x * cell_size + cell_size // 2, player_y * cell_size + cell_size // 2))

    # Draw target and player
    pygame.draw.circle(win, (255, 0, 0), (target_x * cell_size + cell_size // 2, target_y * cell_size + cell_size // 2), 12)
    pygame.draw.circle(win, (0, 255, 0), (player_x * cell_size + cell_size // 2, player_y * cell_size + cell_size // 2), 8)

    if player_x == target_x and player_y == target_y:
        pub.publish(True)
        goals_reached += 1
        target_x, target_y = pick_valid_target()

    pygame.display.update()
    clock.tick(15)

    if time.time() - start_time > duration:
        running = False

# End game
pygame.quit()
entropy_pub.publish(False)

# Save result
with open(os.path.join(result_dir, "summary.txt"), "w") as f:
    f.write(f"Session time: {duration} seconds\n")
    f.write(f"Targets reached: {goals_reached}\n")

with open(os.path.join(result_dir, "path_trace.csv"), "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])
    writer.writerows(path_log)

print(f"\n\U0001F3C1 Maze Game Ended\n\U0001F4C1 Saved results to: {result_dir}")
