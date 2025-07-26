#!/usr/bin/env python3

import rospy
import pygame
import random
import time
import os
from std_msgs.msg import Bool
from utils import create_session_folder

# ROS init
rospy.init_node('hrt_game_node')
hit_pub = rospy.Publisher('/hrt_hit', Bool, queue_size=10)
entropy_pub = rospy.Publisher('/start_entropy', Bool, queue_size=1)

# Pygame init
pygame.init()
win = pygame.display.set_mode((800, 600))
pygame.display.set_caption("HRT Game")
font = pygame.font.Font(None, 50)
clock = pygame.time.Clock()

# Shapes and colors
shapes = ['circle', 'triangle', 'square', 'hexagoan']
colors = ['yellow', 'orange', 'pink', 'grey', 'blue', 'green', 'white', 'red']
color_map = {
    'yellow': (255, 255, 0),
    'orange': (255, 165, 0),
    'pink': (255, 105, 180),
    'grey': (169, 169, 169),
    'blue': (0, 0, 255),
    'green': (0, 255, 0),
    'white': (255, 255, 255),
    'red': (255, 0, 0)
}

# Setup participant dir
participant_dir = create_session_folder("hrt")
rospy.set_param("/current_session_dir", participant_dir)

levels = {
    1: "Color → Yellow=LEFT, Orange=RIGHT",
    2: "Shape → Circle=LEFT, Triangle=RIGHT",
    3: "Shape+Color → Triangle+Pink=LEFT, Square+Grey=RIGHT"
}

for level in sorted(levels.keys()):
    desc = levels[level]
    duration = 45

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

    # Entropy logging
    rospy.set_param("/current_game_type", f"hrt_lvl{level}")
    entropy_pub.publish(True)

    entropy_log_path = os.path.join(participant_dir, "entropy_log.csv")
    with open(entropy_log_path, 'a') as f:
        f.write(f"\nLevel: hrt_lvl{level}\n")
        f.write("Time,Entropy\n")

    hits_correct, hits_wrong, misses = 0, 0, 0
    cue_timer = time.time()
    cue_interval = 1.5
    cue_ready = False
    response_received = False
    start_time = time.time()
    running = True

    # Reaction log
    reaction_log_path = os.path.join(participant_dir, f"level_{level}_reaction_log.csv")
    with open(reaction_log_path, 'w') as log_file:
        log_file.write("Time,Shape,Color,PressedKey,Result\n")

    while running and not rospy.is_shutdown():
        now = time.time()
        win.fill((0, 0, 0))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and cue_ready:
                result = "miss"
                key_name = pygame.key.name(event.key).upper()
                if level == 1:
                    if shape_color == 'yellow' and event.key == pygame.K_LEFT:
                        hits_correct += 1
                        hit_pub.publish(True)
                        result = "correct"
                    elif shape_color == 'orange' and event.key == pygame.K_RIGHT:
                        hits_correct += 1
                        hit_pub.publish(True)
                        result = "correct"
                    else:
                        hits_wrong += 1
                        result = "wrong"
                elif level == 2:
                    if shape_name == 'circle' and event.key == pygame.K_LEFT:
                        hits_correct += 1
                        hit_pub.publish(True)
                        result = "correct"
                    elif shape_name == 'triangle' and event.key == pygame.K_RIGHT:
                        hits_correct += 1
                        hit_pub.publish(True)
                        result = "correct"
                    else:
                        hits_wrong += 1
                        result = "wrong"
                elif level == 3:
                    if shape_name == 'triangle' and shape_color == 'pink' and event.key == pygame.K_LEFT:
                        hits_correct += 1
                        hit_pub.publish(True)
                        result = "correct"
                    elif shape_name == 'square' and shape_color == 'grey' and event.key == pygame.K_RIGHT:
                        hits_correct += 1
                        hit_pub.publish(True)
                        result = "correct"
                    else:
                        hits_wrong += 1
                        result = "wrong"

                with open(reaction_log_path, 'a') as log_file:
                    log_file.write(f"{round(now - start_time, 2)},{shape_name},{shape_color},{key_name},{result}\n")

                cue_ready = False
                response_received = True

        if now - cue_timer >= cue_interval:
            if cue_ready and not response_received:
                misses += 1
            cue_ready = True
            response_received = False
            shape_name = random.choice(shapes)
            shape_color = random.choice(colors)
            shape_x = random.randint(100, 700)
            shape_y = random.randint(100, 500)
            cue_timer = now

        if cue_ready:
            pygame.draw.rect(win, (0, 0, 0), (0, 0, 800, 600))
            color = color_map[shape_color]
            if shape_name == 'circle':
                pygame.draw.circle(win, color, (shape_x, shape_y), 50)
            elif shape_name == 'triangle':
                pygame.draw.polygon(win, color, [(shape_x, shape_y - 50), (shape_x - 50, shape_y + 50), (shape_x + 50, shape_y + 50)])
            elif shape_name == 'square':
                pygame.draw.rect(win, color, pygame.Rect(shape_x - 50, shape_y - 50, 100, 100))

        pygame.display.update()
        clock.tick(30)

        if now - start_time > duration:
            running = False

    entropy_pub.publish(False)
    result_file = os.path.join(participant_dir, f"level_{level}_results.csv")
    with open(result_file, 'w') as f:
        f.write("Correct,Wrong,Misses\n")
        f.write(f"{hits_correct},{hits_wrong},{misses}\n")
    print(f"Level {level} done. Results saved to: {result_file}")

    if level < max(levels.keys()):
        for i in range(60, 0, -1):
            win.fill((0, 0, 0))
            break_text = font.render(f"Next level in {i}s", True, (255, 255, 0))
            text_rect = break_text.get_rect(center=(400, 300))
            win.blit(break_text, text_rect)
            pygame.display.update()
            time.sleep(1)

pygame.quit()
print("\n HRT Game Session Complete!")
