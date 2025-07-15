#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

def callback(data):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "entropy_text"
    marker.id = 0
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 1.0
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.3
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.text = "Entropy: {:.3f}".format(data.data)

    pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('entropy_text_node', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.Subscriber('/start_entropy', Float32, callback)
    rospy.spin()
