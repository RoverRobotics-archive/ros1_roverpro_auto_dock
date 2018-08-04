#!/usr/bin/env python

# Author: Jack Kilian
# Description: This script auto_docks the openrover basic platform if it is in a 3m circle in front of the dock.

import numpy as np
import os
import time

import rospy
from std_msgs.msg import Float32, String, Bool, Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray
#from tf.msgs import tf
from tf.transformations import *

class ArucoDockingManager(object):
    
    CMD_VEL_ANGULAR_RATE = 0.5 #rad/s negative is clockwise
    CMD_VEL_LINEAR_RATE = 0.25 #m/s
    TURN_RADIANS = -1.0472 #not exact
    TURN_DURATION = abs(TURN_RADIANS/CMD_VEL_ANGULAR_RATE)
    OPENROVER_TIMER_RATE = 10. #in hz
    SEARCHING_TIMEOUT = 120 #in seconds
    APPROACH_ANGLE = 0.1
    X_CENTERING_BOUND = 0.2
    #K_P = 1.5
    CHECK_FOR_ARUCO_COUNTER_MAX = 3

    JOG_DISTANCE = 0.5
    FINAL_DOCK_DISTANCE = 0.5

    dock_aruco_found = False
    dock_aruco_found_msg = Bool()
    check_for_aruco = False
    check_for_aruco_counter = 0

    cmd_vel_angular = 0

    is_docking = True
    is_searching = True
    is_docked = False
    is_turning = False
    is_looking = True
    is_jogging = False
    is_final_approach = False
    is_centered = False
    is_aproaching = False
    docking_failed = False
    DOCK_ARUCO_NUM = 0
    last_dock_aruco_tf = Transform()
    dock_aruco_tf = Transform()
    dock_success_msg = Bool()
    dock_success_msg.data = False
    docking_state_dict = {'detecting':0, 'centering':1, 'harsh_setup':2, 'easy_setup':3, 'approach':4, 'on_dock_aruco_lost':5, 'on_dock_charging':6, 'reversing':7}
    docking_state = 'detecting'

    def __init__(self):
        rospy.loginfo("Starting automatic docking.")
        #Publishers
        self.pub_dock_success = rospy.Publisher('/rr_auto_dock/docked', Bool, queue_size=1)
        self.pub_dock_found = rospy.Publisher('/rr_auto_dock/dock_found', Bool, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel/auto_dock', TwistStamped, queue_size=1)
        #Intialize Subscribers
        self.sub_aruco_detect = rospy.Subscriber("fiducial_transforms",FiducialTransformArray, self.aruco_detect_cb, queue_size=1)
        self.sub_openrover_charging = rospy.Subscriber("rr_openrover_basic/charging",Bool, self.openrover_charging_cb, queue_size=1)
        #Setup timers
        #self.turn_timer = rospy.Timer(rospy.Duration(self.TURN_DURATION), self.openrover_turn_timer_cb)
        self.docking_timer = rospy.Timer(rospy.Duration(self.SEARCHING_TIMEOUT), self.docking_failed_cb)
        #self.docking_timer.run()

        self.pub_dock_found.publish(self.dock_aruco_found_msg)
        self.pub_dock_success.publish(self.dock_success_msg)


    def aruco_detect_cb(self, fid_tf_array):
        #rospy.loginfo("aruco CB")
        fid_tf = ''
        if not self.is_docked and not self.docking_failed:
            try:
                fid_tf = fid_tf_array.transforms[self.DOCK_ARUCO_NUM]
                #rospy.loginfo("Marker found.")
                [euler_angles, x_trans, z_trans] = self.fid2pos(fid_tf)

                self.last_dock_aruco_tf = self.dock_aruco_tf
                self.dock_aruco_tf = fid_tf

                self.is_searching = False
                self.dock_aruco_found = True
                self.dock_aruco_found_msg.data = self.dock_aruco_found
                self.pub_dock_found.publish(self.dock_aruco_found_msg)
                self.check_for_aruco_counter = 0

                if not self.is_turning:
                    if abs(euler_angles[1])>self.APPROACH_ANGLE:
                        rospy.loginfo("Approach Bad %f", euler_angles[1])
                        is_centered = False
                    else:
                        rospy.loginfo("Approach Good %f", euler_angles[1])
                        is_centered = True
                
                if not self.is_final_approach:
                    if not is_centered:
                        if not self.is_turning:
                            self.openrover_turn(euler_angles[1])
                        if self.is_turning:
                            rospy.loginfo("is_turning")
                            msg = TwistStamped()
                            msg.twist.angular.z = self.cmd_vel_angular
                            self.pub_cmd_vel.publish(msg)
                    else:
                        if not self.is_turning:
                            if not self.is_jogging:
                                rospy.loginfo("Starting jog")
                                self.openrover_forward(self.JOG_DISTANCE)
                            if self.is_jogging:
                                msg = TwistStamped()
                                msg.twist.linear.x = self.CMD_VEL_LINEAR_RATE
                                self.pub_cmd_vel.publish(msg)

                    #decide what docking condition robot is in
                else:
                    if self.is_turning:
                        rospy.loginfo("Waiting to complete turn before jogging")
                    else:
                        if not self.is_jogging:
                            rospy.loginfo("Starting jog")
                            self.openrover_forward(self.JOG_DISTANCE)
                        if self.is_jogging:
                            msg = TwistStamped()
                            msg.twist.linear.x = self.CMD_VEL_LINEAR_RATE
                            self.pub_cmd_vel.publish(msg)

                rospy.loginfo("marker found chill")
            except:
                #rospy.loginfo("No marker found.")
                if self.is_searching: 
                    rospy.loginfo("searching")
                    if self.is_looking:
                        self.check_for_aruco_counter = self.check_for_aruco_counter + 1
                        rospy.loginfo("is_looking, aruco counter is %i", self.check_for_aruco_counter)
                        if (self.check_for_aruco_counter>self.CHECK_FOR_ARUCO_COUNTER_MAX):
                            rospy.loginfo("done looking")
                            self.is_looking = False
                            self.check_for_aruco_counter = 0
                    else:
                        if not self.is_turning:
                            self.openrover_turn(-self.TURN_RADIANS)
                        if self.is_turning:
                            rospy.loginfo("is_turning")
                            msg = TwistStamped()
                            msg.twist.angular.z = self.cmd_vel_angular
                            self.pub_cmd_vel.publish(msg)
                #------saw dock aruco, but then lost it
                if self.dock_aruco_found:
                    rospy.loginfo("Dock aruco lost. Finishing action")
                    if self.is_jogging:
                        msg = TwistStamped()
                        msg.twist.linear.x = self.CMD_VEL_LINEAR_RATE
                        self.pub_cmd_vel.publish(msg)
                    if self.is_turning:
                        rospy.loginfo("is_turning")
                        msg = TwistStamped()
                        msg.twist.angular.z = self.cmd_vel_angular
                        self.pub_cmd_vel.publish(msg)



                    # self.is_final_approach = True
                    # rospy.loginfo("Final docking")
                    # if not self.is_jogging:
                    #     self.openrover_forward(self.FINAL_DOCK_DISTANCE)
                    # if self.is_jogging:
                    #     msg = TwistStamped()
                    #     msg.twist.linear.x = self.CMD_VEL_LINEAR_RATE
                    #     self.pub_cmd_vel.publish(msg)
                    rospy.loginfo("Dock Aruco Found")
                    #2 scenarios. 1. Very close to dock and about to dock. 2. getting better alignment with dock

    def openrover_forward(self, distance):
        rospy.loginfo("Moving forward")
        self.is_jogging = True
        self.linear_timer = rospy.Timer(rospy.Duration(distance/self.CMD_VEL_LINEAR_RATE), self.openrover_linear_timer_cb, True)
        msg = TwistStamped()
        msg.twist.linear.x = self.CMD_VEL_LINEAR_RATE
        self.pub_cmd_vel.publish(msg)

    def openrover_linear_timer_cb(self, event):
        rospy.loginfo("Stop moving forward")
        self.is_jogging = False
        self.openrover_stop()

    def fid2pos(self, fid_tf):
        q_now = [fid_tf.transform.rotation.x, fid_tf.transform.rotation.y, fid_tf.transform.rotation.z, fid_tf.transform.rotation.w]
        euler_angles = euler_from_quaternion(q_now)
        x_trans = fid_tf.transform.translation.x
        z_trans = fid_tf.transform.translation.z
        #if abs(euler_angles[1])<APPROACH_ANGLE:
        #rospy.loginfo("z=%fm and x=%fm", z_trans, x_trans)
        #rospy.loginfo(euler_angles)
        return euler_angles, x_trans, z_trans

    def openrover_stop(self):
        msg = TwistStamped()
        msg.twist.angular.z = 0
        self.pub_cmd_vel.publish(msg)

    def openrover_charging_cb(self, charging_msg):
        self.is_docked = charging_msg.data
        if self.is_docked:
            self.openrover_stop()
        self.dock_success_msg.data = charging_msg.data
        self.pub_dock_success.publish(self.dock_success_msg)


    def openrover_rotate(self, turn_rate):
        msg = TwistStamped()
        msg.twist.angular.z = turn_rate
        self.pub_cmd_vel.publish(msg)

    def docking_failed_cb(self, event):
        self.docking_failed = True
        rospy.loginfo("Docking failed")

    def openrover_turn_timer_cb(self, event):
        rospy.loginfo("Turning ended")
        self.is_turning = False
        self.is_looking = True
        self.openrover_stop();

    def openrover_turn(self, radians):
        rospy.loginfo("Turning started")
        self.is_turning = True
        self.is_looking = False
        turn_period = abs(radians/self.CMD_VEL_ANGULAR_RATE)/2.
        self.turn_timer = rospy.Timer(rospy.Duration(turn_period), self.openrover_turn_timer_cb, True)
        msg = TwistStamped()
        if radians>0:
            rospy.loginfo("Turn right")
            self.cmd_vel_angular = -self.CMD_VEL_ANGULAR_RATE
        else:
            rospy.loginfo("Turn Left")
            self.cmd_vel_angular = self.CMD_VEL_ANGULAR_RATE
        self.pub_cmd_vel.publish(msg)


def auto_dock_main():
    docking_manager = ArucoDockingManager()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()

if __name__ == '__main__':
    try:
        # Initialize docking node
        rospy.init_node('rr_auto_dock', anonymous=True)
        auto_dock_main()
    except rospy.ROSInterruptException:
        pass

