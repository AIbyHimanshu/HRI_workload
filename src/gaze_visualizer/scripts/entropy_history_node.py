#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
import random

class EntropyHistoryVisualizer:
    def __init__(self):
        rospy.init_node('entropy_history_node')
        self.pub = rospy.Publisher('entropy_history_markers', MarkerArray, queue_size=10)
        rospy.Subscriber('/start_entropy', Float32, self.entropy_callback)

        self.history = []
        self.max_points = 30  # seconds of entropy

        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)

    def entropy_callback(self, msg):
        self.history.append(msg.data)
        if len(self.history) > self.max_points:
            self.history.pop(0)

    def get_color(self, value):
        # green to red gradient
        r = min(1.0, max(0.0, value / 2.0))
        g = 1.0 - r
        return (r, g, 0.0)

    def publish_markers(self, event):
        marker_array = MarkerArray()
        for i, entropy in enumerate(self.history):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = rospy.Time.now()
            m.ns = "entropy_history"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = entropy
            m.pose.position.x = i * 0.3
            m.pose.position.y = -1.0  # below origin
            m.pose.position.z = entropy / 2.0
            m.pose.orientation.w = 1.0
            r, g, b = self.get_color(entropy)
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            marker_array.markers.append(m)

        self.pub.publish(marker_array)

if __name__ == '__main__':
    EntropyHistoryVisualizer()
    rospy.spin()
