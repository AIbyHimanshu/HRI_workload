#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import math

true_pose = None
estimated_pose = None

def compute_errors():
    if true_pose and estimated_pose:
        dx = estimated_pose.position.x - true_pose.position.x
        dy = estimated_pose.position.y - true_pose.position.y
        dz = estimated_pose.position.z - true_pose.position.z
        linear_error = math.sqrt(dx**2 + dy**2 + dz**2)

        # Convert orientation quaternion to yaw for simplicity
        def get_yaw(orientation):
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
            return math.atan2(siny_cosp, cosy_cosp)

        yaw_true = get_yaw(true_pose.orientation)
        yaw_est = get_yaw(estimated_pose.orientation)
        angular_error = abs(yaw_true - yaw_est)

        pub_lin.publish(linear_error)
        pub_ang.publish(angular_error)

def cb_true(data):
    global true_pose
    true_pose = data
    compute_errors()

def cb_estimated(data):
    global estimated_pose
    estimated_pose = data
    compute_errors()

rospy.init_node('estimation_error_node')
pub_lin = rospy.Publisher('/estimation_error_lin', Float32, queue_size=10)
pub_ang = rospy.Publisher('/estimation_error_ang', Float32, queue_size=10)

rospy.Subscriber('/true_pose', Pose, cb_true)
rospy.Subscriber('/estimated_pose', Pose, cb_estimated)

rospy.spin()

