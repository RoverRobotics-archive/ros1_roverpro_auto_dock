#!/usr/bin/env python

# Author: Jack Kilian
# Description: This script auto_docks the openrover basic platform if it is in a 3m circle in front of the dock.

import numpy as np
import os
import time
import math

import rospy
from std_msgs.msg import Float32, String, Bool, Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray
#from tf.msgs import tf
from tf.transformations import *

class ArucoDockingManager(object):
    MANAGER_PERIOD = 0.1
    CMD_VEL_ANGULAR_RATE = 0.75 #rad/s negative is clockwise
    CMD_VEL_LINEAR_RATE = 0.5 #m/s
    TURN_RADIANS = -1.0472/2 #not exact
    TURN_DURATION = abs(TURN_RADIANS/CMD_VEL_ANGULAR_RATE)
    MIN_TURN_PERIOD = 0.2
    OPENROVER_TIMER_RATE = 10. #in hz
    SEARCHING_TIMEOUT = 120 #in seconds
    APPROACH_ANGLE = 0.1
    X_CENTERING_BOUND = 0.2
    Z_TRANS_OFFSET = 0 #0.5
    #K_P = 1.5
    CHECK_FOR_ARUCO_COUNTER_MAX = 3

    JOG_DISTANCE = 0.5
    FINAL_APPROACH_DISTANCE = 1.5
    WIGGLE_RADIANS = -0.5
    DOCK_ARUCO_NUM = 0

    dock_aruco_found = False
    dock_aruco_found_msg = Bool()
    check_for_aruco = False
    check_for_aruco_counter = 0

    cmd_vel_angular = 0
    cmd_vel_linear = 0
    cmd_vel_msg = TwistStamped()

    is_final_wiggle = False
    is_in_action = False
    is_final_jog = False
    is_in_view = False
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
    last_dock_aruco_tf = Transform()
    dock_aruco_tf = Transform()
    dock_success_msg = Bool()
    dock_success_msg.data = False
    docking_state_dict = {'searching':0, 'turning':0.5, 'centering':1, 'harsh_setup':2, 'easy_setup':3, 'approach':4, 'on_dock_aruco_lost':5, 'on_dock_charging':6, 'reversing':7}
    docking_state = 'searching'

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
        self.state_manager_timer = rospy.Timer(rospy.Duration(self.MANAGER_PERIOD), self.state_manage_cb)

        #self.docking_timer.run()

        self.pub_dock_found.publish(self.dock_aruco_found_msg)
        self.pub_dock_success.publish(self.dock_success_msg)

    def state_manage_cb(self, event):
        rospy.loginfo("%s", self.docking_state)
        if self.docking_state=='searching':
            if self.check_for_aruco_counter>self.CHECK_FOR_ARUCO_COUNTER_MAX:
                self.check_for_aruco_counter = 0;
                self.is_looking = False
                self.openrover_turn(-self.TURN_RADIANS)
            if self.is_turning:
                self.openrover_turn(-self.TURN_RADIANS)
            else:
                self.is_looking = True

        if self.docking_state=='centering':
            if self.is_in_view:
                if not self.is_turning:
                    self.is_looking = True
                    if self.check_for_aruco_counter>self.CHECK_FOR_ARUCO_COUNTER_MAX:
                        self.is_looking = False
                        self.check_for_aruco_counter = 0;

                    if not self.is_looking:
                        if not self.is_turning:
                            [theta, distance] = self.fid2pos(self.dock_aruco_tf)
                            if abs(theta)>self.APPROACH_ANGLE:
                                self.is_centered = False
                                self.openrover_turn(theta)
                            else:
                                self.is_centered = True
                                self.docking_state='approach'
                                self.openrover_stop()
                            rospy.loginfo(theta)
                        else:
                            self.openrover_turn(self.cmd_vel_angular)
            else:
                self.docking_state='approach'

        if self.docking_state=='approach':
            [theta, distance] = self.fid2pos(self.dock_aruco_tf)
            if self.is_in_view:
                if abs(theta)<self.APPROACH_ANGLE:
                    self.openrover_forward(self.JOG_DISTANCE)
                else:
                    self.docking_state = 'centering'
                #jog forward
            else:
                if self.is_jogging:
                    self.openrover_forward(self.JOG_DISTANCE)
                else:
                    self.is_final_jog = False
                    self.docking_state='final_approach'
                #look, and if seen, keep jogging, else final ram

        if self.docking_state=='final_approach':
            if self.is_in_view:
                self.docking_state='approach'
            final_jog_finished = False
            if not self.is_final_jog:
                rospy.loginfo("Final Push")
                self.openrover_forward(self.FINAL_APPROACH_DISTANCE)
                self.is_final_jog=True
            if not self.is_jogging:
                final_jog_finished = True
            if self.is_final_jog and final_jog_finished:
                self.docking_state = 'final_wiggle'

        if self.docking_state=='final_wiggle':
            if not self.is_final_wiggle:
                self.openrover_turn(self.WIGGLE_RADIANS)
                self.is_final_wiggle = True
            if not self.is_turning and self.is_final_wiggle:
                self.docking_state = 'docking_failed'


            # if not self.is_jogging and not self.is_final_jog:
            #     rospy.loginfo("Final Push")
            #     self.openrover_forward(self.FINAL_APPROACH_DISTANCE)
            #     self.is_final_jog = True
            # elif self.is_jogging and self.is_final_jog:
            #     rospy.loginfo("start final push")
            #     final_jog_finished = True
            #     self.openrover_forward(self.FINAL_APPROACH_DISTANCE)
            # if not self.is_jogging and final_jog_finished:
            #     self.docking_state = 'docking_failed'

        if self.docking_state=='docking_failed':
            self.docking_failed = True            
            #[theta, x_trans, z_trans] = self.fid2pos(self.dock_aruco_tf)
            rospy.loginfo(self.fid2pos(self.dock_aruco_tf))

        if self.docking_state=='docked':
            rospy.loginfo("Charging")

        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    def aruco_detect_cb(self, fid_tf_array):
        #rospy.loginfo("aruco CB")
        try:
            fid_tf = fid_tf_array.transforms[self.DOCK_ARUCO_NUM]
            self.last_dock_aruco_tf = self.dock_aruco_tf
            self.dock_aruco_tf = fid_tf
            self.dock_aruco_found = True
            self.is_in_view = True
            self.dock_aruco_found_msg.data = self.dock_aruco_found
            self.pub_dock_found.publish(self.dock_aruco_found_msg)
            if self.is_looking:
                self.check_for_aruco_counter = self.check_for_aruco_counter + 1
            if self.docking_state=='searching':
                self.docking_state = 'centering'
            #rospy.loginfo("aruco found")
        except:
            self.is_in_view = False
            if self.is_looking:
                self.check_for_aruco_counter = self.check_for_aruco_counter + 1
            #rospy.loginfo("aruco not found")
        #rospy.loginfo("count %i", self.check_for_aruco_counter)

    def openrover_forward(self, distance):
        if self.is_jogging==False:
            self.is_jogging = True
            self.is_in_action = True
            jog_period = abs(distance/self.CMD_VEL_LINEAR_RATE)
            self.linear_timer = rospy.Timer(rospy.Duration(jog_period), self.openrover_linear_timer_cb, True)
            if distance>0:
                rospy.loginfo("Moving forward")
                self.cmd_vel_linear = self.CMD_VEL_LINEAR_RATE
            else:
                rospy.loginfo("Moving Backward")
                self.cmd_vel_linear = -self.CMD_VEL_LINEAR_RATE
        self.cmd_vel_msg.twist.linear.x = self.CMD_VEL_LINEAR_RATE

    def openrover_linear_timer_cb(self, event):
        rospy.loginfo("Stop moving forward")
        self.is_jogging = False
        self.is_in_action = False
        self.openrover_stop()

    def fid2pos(self, fid_tf):
        q_now = [fid_tf.transform.rotation.x, fid_tf.transform.rotation.y, fid_tf.transform.rotation.z, fid_tf.transform.rotation.w]
        theta = euler_from_quaternion(q_now)
        x_trans = fid_tf.transform.translation.x
        z_trans = fid_tf.transform.translation.z
        z_trans = z_trans - self.Z_TRANS_OFFSET
        theta = math.atan2(x_trans, z_trans)
        r = math.sqrt(x_trans ** 2 + z_trans ** 2)
        #if abs(theta)<APPROACH_ANGLE:
        #rospy.loginfo("z=%fm and x=%fm", z_trans, x_trans)
        #rospy.loginfo(theta)
        return theta, r

    def openrover_stop(self):
        self.cmd_vel_msg.twist.linear.x = 0
        self.cmd_vel_msg.twist.angular.z = 0

    def openrover_charging_cb(self, charging_msg):
        self.is_docked = charging_msg.data
        if self.is_docked:
            self.openrover_stop()
            self.docking_state='docked'
        self.dock_success_msg.data = charging_msg.data
        self.pub_dock_success.publish(self.dock_success_msg)

    def openrover_rotate(self, turn_rate):
        msg = TwistStamped()
        msg.twist.angular.z = turn_rate
        self.pub_cmd_vel.publish(msg)

    def docking_failed_cb(self, event):
        self.docking_failed = True
        self.docking_state = 'docking_failed'
        #rospy.loginfo("Docking failed")

    def openrover_turn_timer_cb(self, event):
        rospy.loginfo("Turning ended")
        self.openrover_stop();
        self.is_turning = False
        self.is_in_action = False

    def openrover_turn(self, radians):
        if self.is_turning==False:
            self.is_turning = True
            self.is_in_action = True
            turn_period = abs(radians/self.CMD_VEL_ANGULAR_RATE)
            if turn_period < self.MIN_TURN_PERIOD:
                turn_period = self.MIN_TURN_PERIOD
            self.turn_timer = rospy.Timer(rospy.Duration(turn_period), self.openrover_turn_timer_cb, True)
            if radians>0:
                rospy.loginfo("Turn right for %f", turn_period)
                self.cmd_vel_angular = -self.CMD_VEL_ANGULAR_RATE
            else:
                rospy.loginfo("Turn Left for %f", turn_period)
                self.cmd_vel_angular = self.CMD_VEL_ANGULAR_RATE
        self.cmd_vel_msg.twist.angular.z = self.cmd_vel_angular


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

