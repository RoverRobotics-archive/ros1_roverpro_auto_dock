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
    MAX_RUN_TIMEOUT = 240 #in seconds
    ARUCO_SLOW_WARN_TIMEOUT = rospy.Duration(1) #in seconds
    ARUCO_WAIT_TIMEOUT = 2 #in seconds

    CANCELLED_TIMEOUT = 10 #in seconds

    APPROACH_ANGLE = 0.1
    Z_TRANS_OFFSET = 0 #0.5
    #K_P = 1.5
    CHECK_FOR_ARUCO_COUNTER_MAX = 5

    JOG_DISTANCE = 0.5
    FINAL_APPROACH_DISTANCE = 1.0
    WIGGLE_RADIANS = -0.5
    DOCK_ARUCO_NUM = 0
    UNDOCK_DISTANCE = 1.0

    check_for_aruco = False
    check_for_aruco_counter = 0

    cmd_vel_angular = 0
    cmd_vel_linear = 0
    cmd_vel_msg = TwistStamped()

    is_final_wiggle = False
    is_in_action = False
    is_final_jog = False
    is_in_view = False
    is_docked = False
    is_turning = False
    is_looking = False
    is_jogging = False
    is_waiting = True
    is_undocking = False
    docking_failed = False
    aruco_last_time = rospy.Time()
    last_dock_aruco_tf = Transform()
    dock_aruco_tf = Transform()
    docking_state_list = {'waiting', 'searching', 'centering', 'approach', 'final_approach', 'final_wiggle', 'docking_failed', 'docked', 'undock'}
    docking_state = 'waiting'
    docking_state_msg = String()
    docking_state_msg.data = docking_state
    last_docking_state = ''

    def __init__(self):
        rospy.loginfo("Starting automatic docking.")
        #Publishers
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel/auto_dock', TwistStamped, queue_size=1, latch=True)
        self.pub_docking_state = rospy.Publisher('/auto_dock/docking_state', String, queue_size=1, latch=True)
        self.pub_docking_state.publish(self.docking_state_msg)

        #Intialize Subscribers
        self.sub_aruco_detect = rospy.Subscriber("fiducial_transforms",FiducialTransformArray, self.aruco_detect_cb, queue_size=1)
        self.sub_openrover_charging = rospy.Subscriber("rr_openrover_basic/charging",Bool, self.openrover_charging_cb, queue_size=1)
        self.sub_undock = rospy.Subscriber("/auto_dock/undock", Bool, self.undock_cb, queue_size=1)
        self.sub_cancel_auto_dock = rospy.Subscriber("/auto_dock/cancel", Bool, self.cancel_cb, queue_size=1)
        self.sub_start = rospy.Subscriber("/auto_dock/start", Bool, self.start_cb, queue_size=1)
        #Setup timers
        self.state_manager_timer = rospy.Timer(rospy.Duration(self.MANAGER_PERIOD), self.state_manage_cb, oneshot=False)
        self.docking_timer = rospy.Timer(rospy.Duration(self.MAX_RUN_TIMEOUT), self.docking_failed_cb, oneshot=True)

    def state_manage_cb(self, event):
        rospy.loginfo(self.docking_state)
        if self.docking_state=='waiting':
            self.is_looking = False

        if self.docking_state=='searching':
            if not self.is_turning:
                self.is_looking = True
                rospy.loginfo('searching aruco count: %i', self.check_for_aruco_counter)
                if self.check_for_aruco_counter>self.CHECK_FOR_ARUCO_COUNTER_MAX:
                    self.check_for_aruco_counter = 0
                    self.is_looking = False
                    self.openrover_turn(-self.TURN_RADIANS)

        if self.docking_state=='centering':
            #wait for another detection then center

            self.is_looking = True
            rospy.loginfo('center aruco count: %i', self.check_for_aruco_counter)
            if self.check_for_aruco_counter>1:
                 self.check_for_aruco_counter = 0
                 self.is_looking = False
            if self.is_in_view and not self.is_turning:
                [theta, distance] = self.fid2pos(self.dock_aruco_tf)
                if abs(theta)>self.APPROACH_ANGLE:
                    self.openrover_turn(theta)
                else:
                    rospy.loginfo('centered switching to approach state')
                    self.docking_state='approach'
                    self.openrover_stop()
                #rospy.loginfo(theta)
            # if self.is_in_view and not self.is_turning:
            #     self.is_looking = True
            #     if self.check_for_aruco_counter>self.CHECK_FOR_ARUCO_COUNTER_MAX:
            #         self.is_looking = False
            #         self.check_for_aruco_counter = 0
            #     if not self.is_looking:
            #         if not self.is_turning and self.is_in_view:
            #             [theta, distance] = self.fid2pos(self.dock_aruco_tf)
            #             if abs(theta)>self.APPROACH_ANGLE:
            #                 self.openrover_turn(theta)
            #             else:
            #                 self.docking_state='approach'
            #                 self.openrover_stop()
            #             #rospy.loginfo(theta)
            #         else:
            #             self.openrover_turn(self.cmd_vel_angular)
            # else: #if not in view, then too close or no marker, either way push to approach
            #     self.docking_state='approach'

        if self.docking_state=='approach':
            if self.is_in_view:
                [theta, distance] = self.fid2pos(self.dock_aruco_tf)
                if abs(theta)>self.APPROACH_ANGLE:
                    rospy.loginfo("approach angle exceeded: %f", abs(theta))
                    self.openrover_stop()
                    self.docking_state = 'centering'
                else:
                    self.openrover_forward(self.JOG_DISTANCE)
                #jog forward
            else:
                if not self.is_jogging:
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

        if self.docking_state=='docking_failed':
            self.docking_failed = True
            #[theta, distance] = self.fid2pos(self.dock_aruco_tf)
            #[theta, x_trans, z_trans] = self.fid2pos(self.dock_aruco_tf)
            #rospy.loginfo(self.fid2pos(self.dock_aruco_tf))

        if self.docking_state=='docked':
            pass

        if self.docking_state=='undock':
            if not self.is_undocking:
                self.openrover_forward(-self.UNDOCK_DISTANCE)
                self.is_undocking = True
            if not self.is_jogging:
                self.docking_state='waiting'
                self.full_reset()

        if self.docking_state=='cancelled':
            pass

        self.publish_docking_state()
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    def publish_docking_state(self): #Publish docking state if it has changed
        if not (self.docking_state_msg.data == self.docking_state):
            self.docking_state_msg.data = self.docking_state
            self.pub_docking_state.publish(self.docking_state_msg)

    def full_reset(self): #reset all variables to startup conditions
        self.cmd_vel_angular = 0
        self.cmd_vel_linear = 0
        self.is_looking = False
        self.is_waiting = True
        self.is_final_wiggle = False
        self.is_in_action = False
        self.is_final_jog = False
        self.is_in_view = False
        self.is_docked = False
        self.is_turning = False
        self.is_jogging = False
        self.is_undocking = False
        self.docking_failed = False
        self.aruco_last_time = rospy.Time()
        try:
            self.docking_timer.shutdown()
            rospy.loginfo('full_reset shutdown docking_timer: ')
        except:
            pass
        try:
            self.waiting_timer.shutdown()
            rospy.loginfo('full_reset shutdown waiting_timer: ')
        except:
            pass

    def openrover_forward(self, distance):
        if self.is_jogging==False:
            self.is_jogging = True
            self.is_in_action = True
            jog_period = abs(distance/self.CMD_VEL_LINEAR_RATE)
            rospy.loginfo('jog_period: %f', jog_period)
            self.linear_timer = rospy.Timer(rospy.Duration(1), self.openrover_linear_timer_cb, oneshot=True)
            if distance>0:
                rospy.loginfo("Moving forward")
                self.cmd_vel_linear = self.CMD_VEL_LINEAR_RATE
            else:
                rospy.loginfo("Moving Backward")
                self.cmd_vel_linear = -self.CMD_VEL_LINEAR_RATE*1.5
        self.cmd_vel_msg.twist.linear.x = self.cmd_vel_linear

    def fid2pos(self, fid_tf):
        q_now = [fid_tf.transform.rotation.x, fid_tf.transform.rotation.y, fid_tf.transform.rotation.z, fid_tf.transform.rotation.w]
        euler_angles = euler_from_quaternion(q_now)
        x_trans = fid_tf.transform.translation.x
        z_trans = fid_tf.transform.translation.z
        z_trans = z_trans - self.Z_TRANS_OFFSET
        theta = math.atan2(x_trans, z_trans)
        r = math.sqrt(x_trans ** 2 + z_trans ** 2)
        #if abs(theta)<APPROACH_ANGLE:
        #rospy.loginfo("z=%fm and x=%fm", z_trans, x_trans)
        #rospy.loginfo("Theta: %3.3f, r: %3.3f, x_trans: %3.3f, z_trans: %3.3f, x: %3.3f, y: %3.3f, z: %3.3f", theta, r, x_trans, z_trans, euler_angles[0], euler_angles[1], euler_angles[2])
        return theta, r

    def openrover_stop(self):
        self.cmd_vel_msg.twist.linear.x = 0
        self.cmd_vel_msg.twist.angular.z = 0
        self.is_jogging = False
        self.is_turning = False
        try:
            self.linear_timer.shutdown()
        except:
            pass
        try:
            self.turn_timer.shutdown()
        except:
            pass

    def openrover_turn(self, radians):
        if self.is_turning==False:
            self.is_turning = True
            self.is_in_action = True
            turn_period = abs(radians/self.CMD_VEL_ANGULAR_RATE)
            if turn_period < self.MIN_TURN_PERIOD:
                turn_period = self.MIN_TURN_PERIOD
            self.turn_timer = rospy.Timer(rospy.Duration(turn_period), self.openrover_turn_timer_cb, oneshot=True)
            if radians>0:
                rospy.loginfo("Turn right for %f", turn_period)
                self.cmd_vel_angular = -self.CMD_VEL_ANGULAR_RATE
            else:
                rospy.loginfo("Turn Left for %f", turn_period)
                self.cmd_vel_angular = self.CMD_VEL_ANGULAR_RATE
        self.cmd_vel_msg.twist.angular.z = self.cmd_vel_angular

##---Callbacks
    def undock_cb(self, event):
        rospy.loginfo('undock_cb')
        if event.data == True and not self.docking_state=='cancelled':
            self.openrover_stop()
            self.full_reset()
            self.docking_state='undock'

    def cancel_cb(self, event):
        rospy.loginfo('cancel_cb')
        if event.data:
            self.docking_state='cancelled'
            self.openrover_stop()
            self.full_reset()
            self.cancelled_timer = rospy.Timer(rospy.Duration(self.CANCELLED_TIMEOUT), self.cancelled_timer_cb, oneshot=True)

    def cancelled_timer_cb(self, event):
        rospy.loginfo('cancelled_timer_cb')
        self.docking_state='waiting'

    def start_cb(self, event):
        rospy.loginfo("start_cb")
        if event.data and not (self.docking_state=='docked'):
            self.docking_state='searching'
            self.docking_timer = rospy.Timer(rospy.Duration(self.MAX_RUN_TIMEOUT), self.docking_failed_cb, oneshot=True)

    def wait_now_cb(self, event):
        rospy.loginfo("wait_now_cb")
        if not (self.docking_state=='docked') and not (self.docking_state=='cancelled'):
            self.docking_state = 'waiting'
            self.is_waiting = True
            self.openrover_stop()
            self.docking_timer.shutdown()

    def aruco_detect_cb(self, fid_tf_array):
        if not (self.docking_state=='docked') and not (self.docking_state=='cancelled'):
            #handle timers related to timing out and warning if aruco is running slowly

            #---this chunk is broken
            # aruco_now_time = rospy.Time.now()
            # aruco_cb_period = aruco_now_time-self.aruco_last_time
            # if aruco_now_time > (self.aruco_last_time+self.ARUCO_SLOW_WARN_TIMEOUT):
            #     rospy.logwarn("Aruco running at %2.3fHz.", aruco_cb_period.to_sec()/1000000000)
            # self.aruco_last_time = aruco_now_time
            #---above chunk is broken

            #If no aruco cb's happen within ARUCO_WAIT_TIMEOUT seconds, then assume the image pipe has been disconnected and go into waiting state
            try:
                self.waiting_timer.shutdown()
                self.waiting_timer = rospy.Timer(rospy.Duration(self.ARUCO_WAIT_TIMEOUT), self.wait_now_cb, oneshot=True)
            except:
                self.waiting_timer = rospy.Timer(rospy.Duration(self.ARUCO_WAIT_TIMEOUT), self.wait_now_cb, oneshot=True)



            if self.is_looking: #pause while looking for a certain number of images
                self.check_for_aruco_counter = self.check_for_aruco_counter + 1
            else:
                self.check_for_aruco_counter = 0
            try:
                #If there is no 0 index of transform, then aruco was not found
                fid_tf = fid_tf_array.transforms[0]
                self.last_dock_aruco_tf = self.dock_aruco_tf
                self.dock_aruco_tf = fid_tf
                self.is_in_view = True
                rospy.loginfo('marker detected')
                if self.docking_state=='searching':
                    self.openrover_stop()
                    self.check_for_aruco_counter = 0
                    self.is_looking = False
                    self.docking_state = 'centering'
            except:
                self.is_in_view = False

    def openrover_linear_timer_cb(self, event):
        rospy.loginfo("openrover_linear_timer_cb: Stop moving forward")
        self.is_jogging = False
        self.is_in_action = False
        self.openrover_stop()

    def openrover_charging_cb(self, charging_msg):
        self.is_docked = charging_msg.data
        if self.is_docked and not self.is_undocking:
            self.openrover_stop()
            self.docking_state='docked'
        if not self.is_docked and self.docking_state=='docked': #unsure if this is functioning
            self.docking_state='waiting'
            self.is_waiting=True

    def docking_failed_cb(self, event):
        rospy.loginfo("Docking failed cb")
        self.openrover_stop()
        self.full_reset()
        self.docking_state = 'docking_failed'
        #rospy.loginfo("Docking failed")

    def openrover_turn_timer_cb(self, event):
        rospy.loginfo("openrover_turn_timer_cb: Turning ended")
        self.openrover_stop()
        self.is_turning = False
        self.is_in_action = False


def auto_dock_main():
    docking_manager = ArucoDockingManager()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()

if __name__ == '__main__':
    try:
        # Initialize docking node
        rospy.init_node('auto_dock', anonymous=True)
        auto_dock_main()
    except rospy.ROSInterruptException:
        pass

