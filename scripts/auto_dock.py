#!/usr/bin/env python

# Author: Jack Kilian
# Description: This script auto_docks the openrover basic platform if it is in a 3m circle in front of the dock.

import numpy as np
import os
import time

import rospy
from std_msgs.msg import Float32, String, Bool, Int32
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray
#from tf.msgs import tf
from tf.transformations import *

class ArucoDockingManager(object):
    
    CMD_VEL_ANGULAR_RATE = -0.4 #rad/s negative is clockwise
    APPROACH_ANGLE = 3.141593/8.0
    X_CENTERING_BOUND = 0.2
    K_P = -1
    dock_aruco_found = False
    is_docking = True
    is_docked = False
    DOCK_ARUCO_NUM = 0
    dock_success_msg = Bool()
    dock_success_msg.data = False
    docking_state_dict = {'detecting':0, 'centering':1, 'harsh_setup':2, 'easy_setup':3, 'approach':4, 'on_dock_aruco_lost':5, 'on_dock_charging':6, 'reversing':7}
    docking_state = 'detecting'

    def __init__(self):
        # Initialize docking node
        self.pub_dock_success = rospy.Publisher('/rr_auto_dock/docked', Bool, queue_size=1)
        self.sub_aruco_detect = rospy.Subscriber("fiducial_transforms",FiducialTransformArray, self.aruco_detect_cb, queue_size=1)
        self.sub_openrover_charging = rospy.Subscriber("rr_openrover_basic/charging",Bool, self.openrover_charging_cb, queue_size=1)
        rospy.loginfo("Starting automatic docking.")
        #Publishers
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel/docking', Twist, queue_size=1)
        #Intialize Subscribers
        
    def aruco_detect_cb(self, fid_tf_array):
        fid_tf = ''
        if not self.is_docked:
            try:
                fid_tf = fid_tf_array.transforms[self.DOCK_ARUCO_NUM]
                q_now = [fid_tf.transform.rotation.x, fid_tf.transform.rotation.y, fid_tf.transform.rotation.z, fid_tf.transform.rotation.w]
                euler_angles = euler_from_quaternion(q_now)
                x_trans = fid_tf.transform.translation.x
                z_trans = fid_tf.transform.translation.z
                #if abs(euler_angles[1])<APPROACH_ANGLE:
                rospy.loginfo("z=%fm and x=%fm", z_trans, x_trans)
                rospy.loginfo(euler_angles)
                if abs(x_trans)>self.X_CENTERING_BOUND:
                    rospy.loginfo("Centering")
                    self.openrover_rotate(self.K_P*x_trans)
                #decide what docking condition robot is in
            except:
                rospy.loginfo("No marker found.")
                self.openrover_rotate(self.CMD_VEL_ANGULAR_RATE)

            #msg = Twist()
            #msg.angular.z = 0
            #pub_cmd_vel.publish(msg)

    def openrover_charging_cb(self, charging_msg):
        self.is_docked = charging_msg.data
        self.dock_success_msg.data = charging_msg.data

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

