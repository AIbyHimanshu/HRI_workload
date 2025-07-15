#!/usr/bin/env python3

import rospy
import pygame
import random
import time
import os
from std_msgs.msg import Bool

# ROS node
rospy.init_node('target_hit_game_node')
pub = rospy.Publisher('/target_hit', Bool, queue_size=10)
entropy_pub = rospy.Publisher('/start_entropy', Bool, queue_size=1)

# Pygame setup
pygame.init()
width, height = 600, 400
win = pygame.display.set_mode((width, height))
pygame.display.set_caption("Target Tracking Game")

clock = pygame.time.Clock()

# Visuals
target_radius = 30
gaze_radius = 10
target_pos = [random.randint(50, width-50), random.randint(50, height-50)]
gaze_pos = [width // 2, height // 2]

# Game control
start_time = time.time()
duration = 30  # seconds
hits = 0
overlap_frames = 0

# Results path
timestamp = time.strftime("%Y%m%d_%H%M%S")
result_dir = f"/home/ros/eyegaze_ws/results/target_game_result/session_{timestamp}"
os.makedirs(result_dir, exist_ok=True)

# Entropy signal
rospy.set_param("/current_game_type", "target")   # Start entropy tracking
entropy_pub.publish(True)  # Signal to begin entropy calc

running = True
while running and not rospy.is_shutdown():
    win.fill((30, 30, 30))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:  gaze_pos[0] -= 5
    if keys[pygame.K_RIGHT]: gaze_pos[0] += 5
    if keys[pygame.K_UP]:    gaze_pos[1] -= 5
    if keys[pygame.K_DOWN]:  gaze_pos[1] += 5

    # Draw
    pygame.draw.circle(win, (255, 0, 0), target_pos, target_radius)
    pygame.draw.circle(win, (0, 255, 0), gaze_pos, gaze_radius)

    # Hit detection
    dx = gaze_pos[0] - target_pos[0]
    dy = gaze_pos[1] - target_pos[1]
    dist = (dx**2 + dy**2)**0.5

    if dist <= target_radius:
        overlap_frames += 1
        if overlap_frames > 10:
            pub.publish(True)
            hits += 1
            target_pos = [random.randint(50, width-50), random.randint(50, height-50)]
            overlap_frames = 0
    else:
        overlap_frames = 0

    pygame.display.update()
    clock.tick(30)

    # Check time limit
    if time.time() - start_time > duration:
        running = False

# End game
pygame.quit()
entropy_pub.publish(False)  # Signal to end entropy calc

# Save result
with open(os.path.join(result_dir, "summary.txt"), "w") as f:
    f.write(f"Session time: {duration} seconds\n")
    f.write(f"Total hits: {hits}\n")

print(f"\n🔚 Game Ended\n✅ Total Hits: {hits}")
print(f"📁 Saved results to: {result_dir}")
