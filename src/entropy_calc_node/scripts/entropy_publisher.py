#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import random
import math

entropy = 0.0
history = []

def calculate_entropy(data):
    global history
    history.append(data)
    if len(history) > 100:
        history = history[-100:]
    p_data = [history.count(x)/len(history) for x in set(history)]
    entropy = -sum([p * math.log(p) for p in p_data])
    return entropy

def entropy_loop():
    pub = rospy.Publisher('/start_entropy', Float32, queue_size=10)
    rospy.init_node('entropy_publisher', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        dummy_input = random.randint(0, 5)
        entropy = calculate_entropy(dummy_input)
        rospy.loginfo(f"Publishing entropy: {entropy:.4f}")
        pub.publish(entropy)
        rate.sleep()

if __name__ == '__main__':
    try:
        entropy_loop()
    except rospy.ROSInterruptException:
        pass
