#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import csv
import os
import time
import random

active = False
entropy_data = []
start_time = None
log_path = None
level_header_written = False

def start_cb(msg):
    global active, start_time, entropy_data, log_path, level_header_written

    if msg.data:
        # Begin new session
        active = True
        entropy_data = []
        start_time = time.time()
        level_header_written = False  # Reset for new session

        result_dir = rospy.get_param("/current_session_dir", None)
        game_type = rospy.get_param("/current_game_type", "default")

        if result_dir:
            log_path = os.path.join(result_dir, "entropy_log.csv")

            # Append level header and column names
            with open(log_path, 'a') as f:
                f.write(f"\nLevel: {game_type}\n")
                f.write("Time,Entropy\n")
            level_header_written = True

            rospy.loginfo(f"🟢 Entropy logging started for: {game_type} → {log_path}")
        else:
            rospy.logwarn("⚠️ No session directory found. Entropy won't be saved.")
            log_path = None

    else:
        # Stop and write to file
        active = False
        if log_path and entropy_data:
            with open(log_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(entropy_data)
            rospy.loginfo(f"📁 Entropy log saved to: {log_path}")
        else:
            rospy.logwarn("⚠️ No entropy data to save.")

def main():
    rospy.init_node("entropy_calc")
    rospy.Subscriber("/start_entropy", Bool, start_cb)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if active:
            now = time.time() - start_time
            entropy = round(random.uniform(0.0, 2.0), 4)
            rospy.loginfo(f"Publishing entropy: {entropy:.4f}")
            entropy_data.append([round(now, 2), entropy])
        rate.sleep()

if __name__ == "__main__":
    main()
