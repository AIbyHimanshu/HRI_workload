#!/usr/bin/env python3

import rospy
import pygame
import random
import time
import os
import threading
import math
from std_msgs.msg import Bool, Float32
from utils import create_session_folder

# === Setup ROS Node ===
rospy.init_node("hrt_visual_game_node", anonymous=True)
entropy_pub = rospy.Publisher("/start_entropy", Bool, queue_size=1)
rospy.set_param("/current_game_type", "hrt")

# === Global entropy shared across threads ===
latest_entropy = 0.0

def entropy_cb(msg):
    global latest_entropy
    latest_entropy = msg.data

rospy.Subscriber("/calculated_entropy", Float32, entropy_cb)

# === Pygame Setup ===
pygame.init()
width, height = 800, 500
win = pygame.display.set_mode((width, height))
pygame.display.set_caption("HRT Shape Game")
font = pygame.font.Font(None, 36)
clock = pygame.time.Clock()

# === Game Params ===
duration = 60
cue_interval = 1
reaction_window = 1.5
cue = None
cue_start = None
response_logged = False
response = "none"
responses = []
correct_hits = 0
wrong_hits = 0
missed_hits = 0
total_cues = 0

# === Countdown ===
for i in reversed(range(1, 4)):
    win.fill((0, 0, 0))
    text = font.render(f"Starting in {i}...", True, (255, 255, 255))
    win.blit(text, (width // 2 - 100, height // 2 - 20))
    pygame.display.update()
    time.sleep(1)

win.fill((0, 0, 0))
text = font.render("Go!", True, (0, 255, 0))
win.blit(text, (width // 2 - 40, height // 2 - 20))
pygame.display.update()
time.sleep(1)

# === Session Folders ===
#timestamp = time.strftime("%Y%m%d_%H%M%S")
#result_dir = f"/home/ros/eyegaze_ws/results/hrt_game_result/session_{timestamp}"
#os.makedirs(result_dir, exist_ok=True)

# Shared results path from utility
result_dir = create_session_folder("hrt")
rospy.set_param("/current_session_dir", result_dir)

# === Start Game ===
entropy_pub.publish(True)
start_time = time.time()
next_cue_time = start_time

running = True
while running and not rospy.is_shutdown():
    now = time.time()
    win.fill((0, 0, 0))

    # === Draw cue ===
    if cue is None and now >= next_cue_time:
        cue = random.choice(["circle", "square", "triangle", "hexagon"])
        cue_start = now
        cue_x = random.randint(100, width - 100)
        cue_y = random.randint(100, height - 100)
        response = "none"
        response_logged = False
        total_cues += 1
        next_cue_time = now + cue_interval

    if cue:
        if cue == "circle":
            pygame.draw.circle(win, (0, 255, 255), (cue_x, cue_y), 40)
        elif cue == "square":
            pygame.draw.rect(win, (255, 255, 0), pygame.Rect(cue_x - 40, cue_y - 40, 80, 80))
        elif cue == "triangle":
            pygame.draw.polygon(win, (255, 0, 255), [(cue_x, cue_y - 40), (cue_x - 40, cue_y + 40), (cue_x + 40, cue_y + 40)])
        elif cue == "hexagon":
            points = [
                (cue_x + 40 * math.cos(a), cue_y + 40 * math.sin(a))
                for a in [i * math.pi / 3 for i in range(6)]
            ]
            pygame.draw.polygon(win, (0, 255, 0), points)

    # === Handle input ===
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if cue and not response_logged and event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                response = "left"
            elif event.key == pygame.K_RIGHT:
                response = "right"

            rt = round(now - cue_start, 3)
            is_edge = cue in ["square", "triangle", "hexagon"]
            correct = (response == "right" if is_edge else response == "left")
            responses.append((cue, response, correct, rt, round(now - start_time, 2)))
            if correct:
                correct_hits += 1
            else:
                wrong_hits += 1
            cue = None
            response_logged = True

    # === Timeout ===
    if cue and (now - cue_start > reaction_window) and not response_logged:
        responses.append((cue, "none", False, "N/A", round(now - start_time, 2)))
        missed_hits += 1
        cue = None
        response_logged = True

    pygame.display.update()
    clock.tick(30)

    # === End after duration ===
    if now - start_time > duration:
        running = False

# === Shutdown ===
entropy_pub.publish(False)
pygame.quit()

# === Save results ===
with open(os.path.join(result_dir, "reaction_log.csv"), "w") as f:
    f.write("Cue,Response,Correct,Reaction Time,Game Time\n")
    for row in responses:
        f.write(",".join(map(str, row)) + "\n")

with open(os.path.join(result_dir, "summary.txt"), "w") as f:
    f.write(f"Session time: {duration} seconds\n")
    f.write(f"Total cues: {total_cues}\n")
    f.write(f"Correct responses: {correct_hits}\n")
    f.write(f"Wrong responses: {wrong_hits}\n")
    f.write(f"Missed responses: {missed_hits}\n")

print("\n HRT Visual Game Ended")
print(f"Saved results to: {result_dir}")
