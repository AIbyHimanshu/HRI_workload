#!/usr/bin/env python3

import rospy
import pygame
import random
import time
import os
import threading
from std_msgs.msg import Bool, Float32

# === Setup ROS Node ===
rospy.init_node("hrt_visual_game_node", anonymous=True)
entropy_pub = rospy.Publisher("/start_entropy", Bool, queue_size=1)
rospy.set_param("/current_game_type", "hrt")

# === Global entropy shared across threads ===
latest_entropy = 0.0

def entropy_cb(msg):
    global latest_entropy
    latest_entropy = msg.data

# Use separate thread to not block rospy callbacks
rospy.Subscriber("/calculated_entropy", Float32, entropy_cb)

# === Pygame Setup ===
pygame.init()
width, height = 640, 360
win = pygame.display.set_mode((width, height))
pygame.display.set_caption("HRT Entropy Game")
font = pygame.font.Font(None, 36)
clock = pygame.time.Clock()

# === Game Params ===
duration = 30
cue_interval = 3
reaction_window = 2.0  # seconds
cue = None
cue_start = None
response_logged = False
response = "none"
responses = []
correct_hits = 0
total_cues = 0

# === Session Folders ===
timestamp = time.strftime("%Y%m%d_%H%M%S")
result_dir = f"/home/ros/eyegaze_ws/results/hrt_game_result/session_{timestamp}"
os.makedirs(result_dir, exist_ok=True)

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
        cue = random.choice(["left", "right"])
        cue_start = now
        response = "none"
        response_logged = False
        total_cues += 1
        next_cue_time = now + cue_interval

    if cue:
        center_y = height // 2
        if cue == "left":
            points = [
                (220, center_y),
                (270, center_y - 30), (270, center_y - 10),
                (330, center_y - 10), (330, center_y + 10),
                (270, center_y + 10), (270, center_y + 30)
            ]
        else:
            points = [
                (420, center_y),
                (370, center_y - 30), (370, center_y - 10),
                (310, center_y - 10), (310, center_y + 10),
                (370, center_y + 10), (370, center_y + 30)
            ]
        pygame.draw.polygon(win, (255, 255, 0), points)

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
            correct = (response == cue)
            responses.append((cue, response, correct, rt, round(now - start_time, 2)))
            if correct:
                correct_hits += 1
            cue = None
            response_logged = True

    # === Timeout ===
    if cue and (now - cue_start > reaction_window) and not response_logged:
        responses.append((cue, "none", False, "N/A", round(now - start_time, 2)))
        cue = None
        response_logged = True

    # === Overlay Entropy ===
    entropy_text = font.render(f"Entropy: {latest_entropy:.2f}", True, (0, 255, 255))
    win.blit(entropy_text, (10, 10))

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

print("\n✅ HRT Visual Game Ended")
print(f"📁 Saved results to: {result_dir}")
