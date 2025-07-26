#!/usr/bin/env python3

import rospy
import pygame
import random
import time
import os
from std_msgs.msg import Bool
from utils import create_session_folder

# Init ROS
rospy.init_node('reflex_game_node')
hit_pub = rospy.Publisher('/reflex_hit', Bool, queue_size=10)
entropy_pub = rospy.Publisher('/start_entropy', Bool, queue_size=1)

# Pygame Init
pygame.init()
win = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Reflex Game")
font = pygame.font.Font(None, 50)
clock = pygame.time.Clock()

# --- Game Parameters ---
levels = {
    1: {"description": "Color → Key: Red=R, Green=G", "duration": 45},
    2: {"description": "Text Color → Press initial of the font color", "duration": 45},
    3: {"description": "Word Color Mismatch → Press initial of the word", "duration": 45},
}

color_key_map_lvl1 = {'red': pygame.K_r, 'green': pygame.K_g}
color_options = ['red', 'green', 'blue', 'yellow', 'white']
color_rgb_map = {
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255),
    'yellow': (255, 255, 0),
    'white': (255, 255, 255)
}

# ROS setup
participant_dir = create_session_folder("reflex")
rospy.set_param("/current_session_dir", participant_dir)

# --- Game Loop ---
for level in sorted(levels.keys()):
    desc = levels[level]['description']
    duration = levels[level]['duration']

    # Display countdown
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

    # Setup
    hits_correct, hits_wrong, misses = 0, 0, 0
    start_time = time.time()
    cue_color = None
    cue_ready = False
    cue_timer = time.time()
    cue_interval = 1.5
    response_received = False
    last_word = ""
    font_color_name = ""
    word_color_name = ""

    # Entropy session
    rospy.set_param("/current_game_type", f"reflex_lvl{level}")
    entropy_pub.publish(True)

    # Add entropy label header to csv
    entropy_log_path = os.path.join(participant_dir, "entropy_log.csv")
    with open(entropy_log_path, 'a') as f:
        f.write(f"Level: reflex_lvl{level}\n")
        f.write("Time,Entropy\n")

    running = True
    while running and not rospy.is_shutdown():
        now = time.time()
        win.fill((0, 0, 0))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN and cue_ready:
                key = event.unicode.upper()

                if level == 1:
                    expected_key = pygame.key.name(color_key_map_lvl1[cue_color]).upper()
                    if key == expected_key:
                        hits_correct += 1
                        hit_pub.publish(True)
                    else:
                        hits_wrong += 1

                elif level == 2:
                    if key == cue_color[0].upper():
                        hits_correct += 1
                        hit_pub.publish(True)
                    else:
                        hits_wrong += 1

                elif level == 3:
                    if key == word_color_name[0].upper():
                        hits_correct += 1
                        hit_pub.publish(True)
                    else:
                        hits_wrong += 1

                response_received = True
                cue_ready = False

        # Cue logic
        if now - cue_timer >= cue_interval:
            if cue_ready and not response_received:
                misses += 1
            cue_ready = True
            response_received = False

            if level == 1:
                cue_color = random.choice(list(color_key_map_lvl1.keys()))

            elif level == 2:
                cue_color = random.choice(color_options)
                last_word = random.choice(color_options)

            elif level == 3:
                cue_color = random.choice(color_options)
                word_color_name = random.choice([c for c in color_options if c != cue_color])
                font_color_name = random.choice([c for c in color_options if c != cue_color and c != word_color_name])

            cue_timer = now

        if cue_ready:
            if level == 1:
                color_rgb = color_rgb_map[cue_color]
                text = font.render(f"PRESS {cue_color[0].upper()}", True, (255, 255, 255))
                text_rect = text.get_rect(center=(400, 300))
                win.fill(color_rgb)
                win.blit(text, text_rect)

            elif level == 2:
                text = font.render(last_word.upper(), True, color_rgb_map[cue_color])
                text_rect = text.get_rect(center=(400, 300))
                win.blit(text, text_rect)

            elif level == 3:
                background_rgb = color_rgb_map[cue_color]
                font_rgb = color_rgb_map[font_color_name]
                win.fill(background_rgb)
                text = font.render(word_color_name.upper(), True, font_rgb)
                text_rect = text.get_rect(center=(400, 300))
                win.blit(text, text_rect)

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

    # Add break between levels
    if level < max(levels.keys()):
        for i in range(60, 0, -1):
            win.fill((0, 0, 0))
            break_text = font.render(f"Next level in {i}s", True, (255, 255, 0))
            text_rect = break_text.get_rect(center=(400, 300))
            win.blit(break_text, text_rect)
            pygame.display.update()
            time.sleep(1)

pygame.quit()
print("\n Reflex Game Session Complete!")
