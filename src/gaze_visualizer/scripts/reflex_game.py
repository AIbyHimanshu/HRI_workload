#!/usr/bin/env python3

import rospy
import pygame
import random
import time
import os
from std_msgs.msg import Bool

# Init ROS
rospy.init_node('reflex_game_node')
hit_pub = rospy.Publisher('/reflex_hit', Bool, queue_size=10)
entropy_pub = rospy.Publisher('/start_entropy', Bool, queue_size=1)

# Pygame Init
pygame.init()
win = pygame.display.set_mode((500, 300))
pygame.display.set_caption("Reflex Game")
font = pygame.font.Font(None, 50)
clock = pygame.time.Clock()

# Countdown before game start
for i in range(3, 0, -1):
    win.fill((0, 0, 0))
    countdown_text = font.render(str(i), True, (255, 255, 255))
    win.blit(countdown_text, (230, 130))
    pygame.display.update()
    time.sleep(1)

win.fill((0, 0, 0))
go_text = font.render("GO!", True, (0, 255, 0))
win.blit(go_text, (210, 130))
pygame.display.update()
time.sleep(1)

# Session control
start_time = time.time()
duration = 60  # Game duration
hits_correct = 0
hits_wrong = 0
misses = 0
cue_color = None
color_key_map = {'red': pygame.K_r, 'green': pygame.K_g}

# Folder structure
timestamp = time.strftime("%Y%m%d_%H%M%S")
result_dir = f"/home/ros/eyegaze_ws/results/reflex_game_result/session_{timestamp}"
os.makedirs(result_dir, exist_ok=True)

# Start entropy tracking
rospy.set_param("/current_game_type", "reflex")
entropy_pub.publish(True)

running = True
cue_timer = time.time()
cue_interval = 1  # seconds
cue_ready = False
response_received = False

while running and not rospy.is_shutdown():
    now = time.time()
    win.fill((0, 0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN and cue_ready:
            key = event.key
            if cue_color:
                expected_key = color_key_map[cue_color]
                if key == expected_key:
                    hits_correct += 1
                    hit_pub.publish(True)
                else:
                    hits_wrong += 1
                response_received = True
                cue_ready = False  # Ignore further keypresses for this cue

    # Show a new cue every second
    if now - cue_timer >= cue_interval:
        if cue_ready and not response_received:
            misses += 1  # no response in time
        cue_color = random.choice(['red', 'green'])
        cue_ready = True
        response_received = False
        cue_timer = now

    if cue_ready:
        if cue_color == 'red':
            win.fill((255, 0, 0))
            text = font.render("PRESS R", True, (255, 255, 255))
        else:
            win.fill((0, 255, 0))
            text = font.render("PRESS G", True, (0, 0, 0))
        win.blit(text, (160, 130))

    pygame.display.update()
    clock.tick(30)

    if now - start_time > duration:
        running = False

pygame.quit()
entropy_pub.publish(False)

# Save result
with open(os.path.join(result_dir, "summary.txt"), "w") as f:
    f.write(f"Session time: {duration} seconds\n")
    f.write(f"Total reflex hits correct: {hits_correct}\n")
    f.write(f"Total reflex hits wrong: {hits_wrong}\n")
    f.write(f"Total reflex misses: {misses}\n")

print(f"\n Reflex Game Ended")
print(f"Total Correct Hits: {hits_correct}")
print(f"Total Wrong Hits: {hits_wrong}")
print(f"Total Misses: {misses}")
print(f"Saved results to: {result_dir}")
