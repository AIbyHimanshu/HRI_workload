#!/usr/bin/env python3

import rospy
import pygame
import random
import time
import os
import csv
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from collections import deque
from utils import create_session_folder

# ROS node
rospy.init_node('target_hit_game_node')
pub = rospy.Publisher('/target_hit', Bool, queue_size=10)
entropy_pub = rospy.Publisher('/start_entropy', Bool, queue_size=1)

# Pygame setup
pygame.init()
width, height = 800, 600
win = pygame.display.set_mode((width, height))
pygame.display.set_caption("Target Maze Game")
font = pygame.font.Font(None, 50)
clock = pygame.time.Clock()

# Joystick support
pygame.joystick.init()
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Joystick detected: ", joystick.get_name())
else:
    joystick = None
    print("⚠️ No joystick/game controller detected. Keyboard fallback enabled.")

levels = {
    1: {"cols": 20, "rows": 15, "regen": False},
    2: {"cols": 30, "rows": 20, "regen": False},
    3: {"cols": None, "rows": None, "regen": True}  # Random size each goal
}

for level, config in levels.items():
    for i in range(5, 0, -1):
        win.fill((0, 0, 0))
        countdown_text = font.render(f"Level {level} in {i}s", True, (255, 255, 255))
        text_rect = countdown_text.get_rect(center=(400, 300))
        win.blit(countdown_text, text_rect)
        pygame.display.update()
        time.sleep(1)

    win.fill((0, 0, 0))
    start_text = font.render(f"GO! - Level {level}", True, (0, 255, 0))
    text_rect = start_text.get_rect(center=(400, 300))
    win.blit(start_text, text_rect)
    pygame.display.update()
    time.sleep(1)

    rospy.set_param("/current_game_type", f"target_lvl{level}")
    entropy_pub.publish(True)

    duration = 120
    start_time = time.time()
    result_dir = create_session_folder("target")
    rospy.set_param("/current_session_dir", result_dir)

    if config["cols"] and config["rows"]:
        cols, rows = config["cols"], config["rows"]
    else:
        cols = random.randint(20, 30)
        rows = random.randint(15, 20)

    cell_size = min(width // cols, height // rows)
    maze = [[1 for _ in range(cols)] for _ in range(rows)]
    visited = [[False for _ in range(cols)] for _ in range(rows)]

    def generate_maze(x, y):
        visited[y][x] = True
        maze[y][x] = 0
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        random.shuffle(directions)
        for dx, dy in directions:
            nx, ny = x + dx * 2, y + dy * 2
            if 0 <= nx < cols and 0 <= ny < rows and not visited[ny][nx]:
                maze[y + dy][x + dx] = 0
                generate_maze(nx, ny)

    from collections import deque
    def is_reachable(sx, sy, tx, ty):
        visited_bfs = [[False for _ in range(cols)] for _ in range(rows)]
        queue = deque([(sx, sy)])
        visited_bfs[sy][sx] = True
        while queue:
            x, y = queue.popleft()
            if x == tx and y == ty:
                return True
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < cols and 0 <= ny < rows and maze[ny][nx] == 0 and not visited_bfs[ny][nx]:
                    visited_bfs[ny][nx] = True
                    queue.append((nx, ny))
        return False

    def pick_valid_target():
        while True:
            tx, ty = random.randint(0, cols - 1), random.randint(0, rows - 1)
            if maze[ty][tx] == 0 and (tx, ty) != (player_x, player_y) and is_reachable(player_x, player_y, tx, ty):
                return tx, ty

    generate_maze(0, 0)
    player_x, player_y = 0, 0
    target_x, target_y = pick_valid_target()

    segment_paths = []
    current_segment = []
    segment_start_time = time.time()
    goals_reached = 0

    running = True
    while running and not rospy.is_shutdown():
        now = time.time()
        if now - start_time > duration:
            break

        win.fill((0, 0, 0))
        for y in range(rows):
            for x in range(cols):
                rect = pygame.Rect(x * cell_size, y * cell_size, cell_size, cell_size)
                color = (0, 0, 0) if maze[y][x] == 0 else (100, 100, 100)
                pygame.draw.rect(win, color, rect)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        dx, dy = 0, 0
        if joystick:
            x_axis = joystick.get_axis(0)
            y_axis = joystick.get_axis(1)
            if abs(x_axis) > 0.5:
                dx = int(x_axis / abs(x_axis))
            if abs(y_axis) > 0.5:
                dy = int(y_axis / abs(y_axis))
        else:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]: dx = -1
            elif keys[pygame.K_RIGHT]: dx = 1
            elif keys[pygame.K_UP]: dy = -1
            elif keys[pygame.K_DOWN]: dy = 1

        if dx or dy:
            nx, ny = player_x + dx, player_y + dy
            if 0 <= nx < cols and 0 <= ny < rows and maze[ny][nx] == 0:
                player_x, player_y = nx, ny
                px = player_x * cell_size + cell_size // 2
                py = player_y * cell_size + cell_size // 2
                current_segment.append((time.time() - segment_start_time, px, py))

        if len(current_segment) > 1:
            for i in range(1, len(current_segment)):
                pygame.draw.line(win, (0, 100, 255), (current_segment[i-1][1], current_segment[i-1][2]), (current_segment[i][1], current_segment[i][2]), 2)

        pygame.draw.circle(win, (255, 0, 0), (target_x * cell_size + cell_size // 2, target_y * cell_size + cell_size // 2), 12)
        pygame.draw.circle(win, (0, 255, 0), (player_x * cell_size + cell_size // 2, player_y * cell_size + cell_size // 2), 8)

        if player_x == target_x and player_y == target_y:
            pub.publish(True)
            goals_reached += 1
            segment_paths.append((segment_start_time, time.time(), list(current_segment)))
            current_segment = []
            segment_start_time = time.time()
            if config["regen"]:
                cols = random.randint(20, 30)
                rows = random.randint(15, 20)
                cell_size = min(width // cols, height // rows)
                maze = [[1 for _ in range(cols)] for _ in range(rows)]
                visited = [[False for _ in range(cols)] for _ in range(rows)]
                generate_maze(0, 0)
                player_x, player_y = 0, 0
            target_x, target_y = pick_valid_target()

        pygame.display.update()
        clock.tick(15)

    entropy_pub.publish(False)

    # Save result
    with open(os.path.join(result_dir, f"level_{level}_summary.txt"), "w") as f:
        f.write(f"Session time: {duration} seconds\n")
        f.write(f"Targets reached: {goals_reached}\n")

    with open(os.path.join(result_dir, f"level_{level}_path_trace.csv"), "w", newline='') as f:
        writer = csv.writer(f)
        for idx, (start, end, segment) in enumerate(segment_paths, 1):
            writer.writerow([f"=== Segment {idx} Start: {time.strftime('%H:%M:%S', time.localtime(start))} | End: {time.strftime('%H:%M:%S', time.localtime(end))} ==="])
            writer.writerow(["Time (s)", "x", "y"])
            for t, x, y in segment:
                writer.writerow([round(t, 2), x, y])
            writer.writerow([])

    # Auto-generate trajectory plot with maze overlay
    fig, ax = plt.subplots(figsize=(10, 6))
    for y in range(rows):
        for x in range(cols):
            if maze[y][x] == 1:
                rect = plt.Rectangle((x * cell_size, y * cell_size), cell_size, cell_size, color='gray')
                ax.add_patch(rect)

    for idx, (_, _, segment) in enumerate(segment_paths, 1):
        xs = [x for _, x, _ in segment]
        ys = [y for _, _, y in segment]
        ax.plot(xs, ys, label=f'Segment {idx}')
        if xs and ys:
            mid = len(xs) // 2
            ax.text(xs[mid], ys[mid], f'{idx}', fontsize=9, color='black')

    ax.set_title(f"Trajectory Paths with Maze - Level {level}")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_xlim(0, cols * cell_size)
    ax.set_ylim(0, rows * cell_size)
    ax.invert_yaxis()
    ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()
    plt.savefig(os.path.join(result_dir, f"level_{level}_trajectories_plot.png"))
    plt.close()

    print(f"\n Level {level} complete. Results saved to: {result_dir}\n")

pygame.quit()
print("\n Target Game Session Complete!")
