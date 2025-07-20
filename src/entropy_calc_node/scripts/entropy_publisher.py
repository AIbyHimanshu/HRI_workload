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

def start_cb(msg):
    global active, start_time, entropy_data, log_path

    if msg.data:
        # Begin new session
        active = True
        entropy_data = []
        start_time = time.time()

        result_dir = rospy.get_param("/current_session_dir", None)

        if result_dir:
            log_path = os.path.join(result_dir, "entropy_log.csv")
        else:
            rospy.logwarn("⚠️ No session directory found. Entropy won't be saved.")
            log_path = None

        game_type = rospy.get_param("/current_game_type", "default")
        rospy.loginfo(f"🟢 Entropy logging started for: {game_type} → {log_path}")

    else:
        # Stop and write to file
        active = False
        if log_path and entropy_data:
            with open(log_path, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "Entropy"])
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
