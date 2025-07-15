#!/usr/bin/env python3

import rospy
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import Float32
from datetime import datetime

# === Initialize session path ===
session_time = datetime.now().strftime("%Y%m%d_%H%M%S")
session_folder = os.path.expanduser(f"/home/ros/eyegaze_ws/results/maps_result/session_{session_time}")
os.makedirs(session_folder, exist_ok=True)

log_file = os.path.join(session_folder, "entropy_log.csv")
with open(log_file, "w") as f:
    f.write("timestamp,entropy\n")

entropy_history = []

def entropy_callback(msg):
    global entropy_history
    entropy = msg.data
    entropy_history.append((rospy.get_time(), entropy))
    
    # Append to CSV log
    with open(log_file, "a") as f:
        f.write(f"{rospy.get_time()},{entropy}\n")

def update_plot(frame):
    if not entropy_history:
        return

    timestamps, values = zip(*entropy_history)

    # Bar chart
    ax1.clear()
    ax1.bar(range(len(values)), values, color="skyblue")
    ax1.set_title("Entropy Over Time (Bar Chart)")
    ax1.set_ylabel("Entropy")
    ax1.set_xlabel("Step")
    ax1.set_ylim(0, 2.5)

    # Heatmap
    ax2.clear()
    heat = [[v] for v in values[-50:]]  # Show last 50
    ax2.imshow(heat, cmap='hot', aspect='auto', interpolation='nearest')
    ax2.set_title("Entropy Heatmap (Last 50)")
    ax2.set_xticks([])
    ax2.set_yticks([])

    # Save updated plot as image every 10 data points
    if len(values) % 10 == 0:
        fig.savefig(os.path.join(session_folder, f"entropy_plot_{len(values)}.png"))

if __name__ == '__main__':
    rospy.init_node('entropy_plotter', anonymous=True)
    rospy.Subscriber("/start_entropy", Float32, entropy_callback)

    # Set up plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
    ani = animation.FuncAnimation(fig, update_plot, interval=1000)

    plt.tight_layout()
    plt.show()
