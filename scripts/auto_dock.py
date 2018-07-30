#!/usr/bin/env python

# Author: Jack Kilian
# Description: This script auto_docks the openrover basic platform if it is in a 3m circle in front of the dock.

import numpy as np
import os
import time

import rospy
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray

class ArucoDockingManager(object):
    
    CMD_VEL_ANGULAR_RATE = -0.25 #rad/s negative is clockwise
    dock_aruco_found = False
    docking = True
    _DOCK_ARUCO_NUM = 0
    dock_success_msg = Bool()
    dock_success_msg.data = False
    def __init__(self):
        # Initialize docking node
        self.pub_dock_success = rospy.Publisher('/rr_auto_dock/docked', Bool, queue_size=1)
        self.sub_aruco_detect = rospy.Subscriber("fiducial_transforms",FiducialTransformArray, self.aruco_detect_cb, queue_size=1)
        rospy.loginfo("Starting automatic docking.")
        #Publishers
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel/docking', Twist, queue_size=1)
        #Intialize Subscribers
        
    def aruco_detect_cb(self, fid_tf_array):
        try:
            fid_tf = fid_tf_array.transforms[self._DOCK_ARUCO_NUM]
            rospy.loginfo("The dock is z=%fm and x=%fm away.", fid_tf.transform.translation.z, fid_tf.transform.translation.x)
        except:
            rospy.loginfo("No marker found.")
            self.openrover_rotate(self.CMD_VEL_ANGULAR_RATE)
        self.pub_dock_success.publish(self.dock_success_msg)
        #msg = Twist()
        #msg.angular.z = 0
        #pub_cmd_vel.publish(msg)

    def openrover_rotate(self, turn_rate):
        msg = Twist()
        msg.angular.z = turn_rate
        self.pub_cmd_vel.publish(msg)

def auto_dock_main():
    docking_manager = ArucoDockingManager()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('rr_auto_dock', anonymous=True)
        auto_dock_main()
    except rospy.ROSInterruptException:
        pass

