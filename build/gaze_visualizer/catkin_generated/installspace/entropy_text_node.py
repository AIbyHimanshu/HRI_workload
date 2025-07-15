#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

class EntropyTextMarker:
    def __init__(self):
        rospy.init_node('entropy_text_node', anonymous=True)
        self.entropy = 0.0
        self.pose = Pose()

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        rospy.Subscriber('/start_entropy', Float32, self.entropy_callback)
        rospy.Subscriber('/estimated_pose', Pose, self.pose_callback)

        rospy.Timer(rospy.Duration(0.1), self.publish_marker)

    def entropy_callback(self, msg):
        self.entropy = msg.data

    def pose_callback(self, msg):
        self.pose = msg

    def publish_marker(self, event):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "entropy_text"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = self.pose
        marker.pose.position.z += 1.0  # float above pose

        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.text = "Entropy: {:.3f}".format(self.entropy)

        self.marker_pub.publish(marker)

if __name__ == '__main__':
    EntropyTextMarker()
    rospy.spin()
