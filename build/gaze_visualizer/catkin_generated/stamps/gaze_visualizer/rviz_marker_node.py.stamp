#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf

true_pose = None
estimated_pose = None

def publish_marker(pose, marker_id, color):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "pose_marker"
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.4
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(1.0)
    pub_marker.publish(marker)

def cb_true(data):
    global true_pose
    true_pose = data
    publish_marker(true_pose, 1, (0.0, 1.0, 0.0))  # green

def cb_est(data):
    global estimated_pose
    estimated_pose = data
    publish_marker(estimated_pose, 2, (1.0, 0.0, 0.0))  # red

rospy.init_node('gaze_visualizer_node')
pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
rospy.Subscriber('/true_pose', Pose, cb_true)
rospy.Subscriber('/estimated_pose', Pose, cb_est)
rospy.spin()

