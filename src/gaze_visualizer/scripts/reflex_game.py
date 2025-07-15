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

# Session control
start_time = time.time()
duration = 60
hit_logged = False
triggered = False
hits = 0
round_start = time.time()
wait_time = random.randint(3, 7)

# Folder structure
timestamp = time.strftime("%Y%m%d_%H%M%S")
result_dir = f"/home/ros/eyegaze_ws/results/reflex_game_result/session_{timestamp}"
os.makedirs(result_dir, exist_ok=True)

# Start entropy tracking
rospy.set_param("/current_game_type", "reflex")
entropy_pub.publish(True)

running = True
while running and not rospy.is_shutdown():
    win.fill((0, 0, 0))
    now = time.time()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if triggered and event.type == pygame.KEYDOWN and not hit_logged:
            hit_pub.publish(True)
            hit_logged = True
            hits += 1

    # Trigger flash
    if not triggered and (now - round_start > wait_time):
        triggered = True

    if triggered and not hit_logged:
        win.fill((255, 0, 0))
        text = font.render("Tap NOW!", True, (255, 255, 255))
        win.blit(text, (150, 120))

    elif triggered and hit_logged:
        text = font.render("Good!", True, (0, 255, 0))
        win.blit(text, (180, 120))

        if now - round_start > wait_time + 2:
            # Reset next round
            triggered = False
            hit_logged = False
            round_start = time.time()
            wait_time = random.randint(3, 7)

    pygame.display.update()
    clock.tick(30)

    if now - start_time > duration:
        running = False

pygame.quit()
entropy_pub.publish(False)

# Save result
with open(os.path.join(result_dir, "summary.txt"), "w") as f:
    f.write(f"Session time: {duration} seconds\n")
    f.write(f"Total reflex hits: {hits}\n")

print(f"\n🔚 Reflex Game Ended\n✅ Total Hits: {hits}")
print(f"📁 Saved results to: {result_dir}")
