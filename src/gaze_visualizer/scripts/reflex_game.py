#!/usr/bin/env python3

import rospy
import pygame
import random
import time
from std_msgs.msg import Bool

# ROS Setup
rospy.init_node('reflex_game_node')
pub = rospy.Publisher('/reflex_hit', Bool, queue_size=10)

# Pygame Setup
pygame.init()
win = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Reflex Tap Game")

font = pygame.font.SysFont("Arial", 32)
clock = pygame.time.Clock()

running = True
round_start = time.time()
wait_time = random.randint(3, 7)
triggered = False
hit_logged = False

while running and not rospy.is_shutdown():
    win.fill((0, 0, 0))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if triggered and event.type == pygame.KEYDOWN and not hit_logged:
            pub.publish(True)
            hit_logged = True

    # Trigger flash after wait time
    if not triggered and (time.time() - round_start > wait_time):
        triggered = True

    if triggered and not hit_logged:
        win.fill((200, 0, 0))  # Red flash
        text = font.render("Tap now!", True, (255, 255, 255))
        win.blit(text, (120, 130))

    elif triggered and hit_logged:
        text = font.render("Good!", True, (0, 255, 0))
        win.blit(text, (150, 130))

        # Restart after short pause
        if time.time() - round_start > wait_time + 2:
            triggered = False
            hit_logged = False
            round_start = time.time()
            wait_time = random.randint(3, 7)

    pygame.display.update()
    clock.tick(30)

