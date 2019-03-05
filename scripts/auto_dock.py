#!/usr/bin/env python

# Author: Jack Kilian
# Description: This script auto_docks the openrover basic platform if it is in a 3m circle in front of the dock.

import math

import rospy
from std_msgs.msg import Float32, String, Bool, Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import *
from std_srvs.srv import SetBool

class ArucoDockingManager(object):

    def __init__(self):
        rospy.loginfo("Starting automatic docking.")

        #ROS Params
        self.DOCK_ARUCO_NUM = rospy.get_param('~dock_aruco_number', 0)
        self.TURN_RADIANS = rospy.get_param('~search_turn_amount', -0.8) #a little less than the FOV of the cameras
        self.UNDOCK_DISTANCE = rospy.get_param('~undock_distance', 1.0)
        self.UNDOCK_TURN_AMOUNT = rospy.get_param('~undock_turn_amount', 3.1415)
        self.START_DELAY = rospy.get_param('~start_delay', 2.0) #in seconds
        self.CMD_VEL_LINEAR_RATE = rospy.get_param('~cmd_vel_linear_rate', 0.3)  #m/s
        self.CMD_VEL_ANGULAR_RATE = rospy.get_param('~cmd_vel_angular_rate', 0.8) #rad/s negative is clockwise
        self.MOTOR_RESPONSE_DELAY = rospy.get_param('~motor_response_delay', 0.05) #in secs
        self.ACTION_DELAY = rospy.get_param('~action_delay', 0.3) #amount of time that must pass
                                                                #  before accepting Aruco detections


        #Constants
        self.MANAGER_PERIOD = 0.1
        self.MAX_RUN_TIMEOUT = 240 #in seconds
        self.ARUCO_SLOW_WARN_TIMEOUT = rospy.Duration(1) #in seconds
        self.ARUCO_WAIT_TIMEOUT = 2 #in seconds

        self.CANCELLED_TIMEOUT = 10 #in seconds

        self.APPROACH_ANGLE = 0.1
        self.Z_TRANS_OFFSET = 0 #0.5
        self.ARUCO_CALLBACK_COUNTER_MAX = 5

        self.JOG_DISTANCE = 0.3
        self.FINAL_APPROACH_DISTANCE = 1.5
        self.MAX_CENTERING_COUNT = 50

        #Instance Vars
        self.check_for_aruco = False
        self.aruco_callback_counter = 0
        self.centering_counter = 0

        self.cmd_vel_angular = 0
        self.cmd_vel_linear = 0
        self.cmd_vel_msg = TwistStamped()

        self.is_in_view = False
        self.is_docked = False
        self.enable_detections = True

        self.aruco_last_time = rospy.Time()
        self.finished_action_time = rospy.Time.now()
        self.last_dock_aruco_tf = Transform()
        self.dock_aruco_tf = Transform()
        self.docking_state_list = {'undocked', 'searching', 'centering', 'approach', 'final_approach', 'final_wiggle', 'docking_failed', 'docked', 'undock'}
        self.action_state_list = {'turning', 'count_aruco_callbacks', 'jogging', 'stopping'}
        self.action_state = ''
        self.action_state_data = ''
        self.action_state_msg = String()
        self.undocking_state_list = {'reversing', 'turning'}
        self.undocking_state = ''
        self.docking_state = 'undocked'
        self.docking_state_msg = String()
        self.docking_state_msg.data = self.docking_state
        self.last_docking_state = ''
        self.last_action_state = ''

        #Publishers
        self.pub_aruco_detections_enable = rospy.Publisher('/aruco_detect/enable', Bool, queue_size=1, latch=True)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel/auto_dock', TwistStamped, queue_size=1, latch=True)
        self.pub_docking_state = rospy.Publisher('/auto_dock/state', String, queue_size=1, latch=True)
        self.pub_action_state = rospy.Publisher('/auto_dock/action_state', String, queue_size=1, latch=True)

        #Intialize Subscribers
        self.sub_aruco_detect = rospy.Subscriber("/fiducial_transforms",FiducialTransformArray, self.aruco_detect_cb, queue_size=1)
        self.sub_openrover_charging = rospy.Subscriber("rr_openrover_basic/charging",Bool, self.openrover_charging_cb, queue_size=1)

        self.sub_undock = rospy.Subscriber("/auto_dock/undock", Bool, self.undock_cb, queue_size=1)
        self.sub_cancel_auto_dock = rospy.Subscriber("/auto_dock/cancel", Bool, self.cancel_cb, queue_size=1)
        self.sub_start = rospy.Subscriber("/auto_dock/dock", Bool, self.start_cb, queue_size=1)

        #Services
        rospy.wait_for_service('/aruco_detect/enable_detections')
        self.set_enable_detections = rospy.ServiceProxy('/aruco_detect/enable_detections', SetBool)

        #Setup timers
        self.state_manager_timer = rospy.Timer(rospy.Duration(self.MANAGER_PERIOD), self.state_manage_cb, oneshot=False)

    def state_manage_cb(self, event):
        if self.docking_state=='undocked':
            self.disable_aruco_detections()
            self.undocked_state_fun()

        if self.docking_state=='searching':
            self.enable_aruco_detections()
            self.searching_state_fun()

        if self.docking_state=='centering':
            self.enable_aruco_detections()
            self.centering_state_fun()

        if self.docking_state=='approach':
            self.enable_aruco_detections()
            self.approach_state_fun()

        if self.docking_state=='final_approach':
            self.enable_aruco_detections()
            self.final_approach_state_fun()

        if self.docking_state == 'docking_failed':
            self.disable_aruco_detections()

        if self.docking_state == 'docked':
            self.disable_aruco_detections()

        if self.docking_state == 'undock':
            self.disable_aruco_detections()
            self.undock_state_fun()

        if self.docking_state == 'cancelled':
            self.disable_aruco_detections()

        action_state_data = ('%s | %s' % (self.docking_state, self.action_state))
        self.publish_action_state(action_state_data)
        self.publish_docking_state()
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    ##---State Functions
    def undocked_state_fun(self):
        self.set_action_state('')

    def searching_state_fun(self):
        #rospy.logdebug('searching aruco count: %i', self.aruco_callback_counter)
        self.centering_counter = 0
        if self.action_state=='turning':
            return
        if self.aruco_callback_counter<self.ARUCO_CALLBACK_COUNTER_MAX:
            self.set_action_state('count_aruco_callbacks')
        else:
            self.aruco_callback_counter = 0
            self.set_action_state('')
            self.openrover_turn(-self.TURN_RADIANS)

    def centering_state_fun(self):
        #wait for another detection then center
        #rospy.logdebug('centering aruco count: %i', self.aruco_callback_counter)
        if self.action_state=='turning':
            return
        if self.aruco_callback_counter < 2:
            self.centering_counter = self.centering_counter + 1
            self.set_action_state('count_aruco_callbacks')
            return
        self.aruco_callback_counter = 0
        self.set_action_state('')
        if self.centering_counter >= self.MAX_CENTERING_COUNT:
            rospy.logwarn('centering failed. reverting to last state: %s', self.last_docking_state)
            self.aruco_callback_counter = 0
            self.set_action_state('')
            self.set_docking_state(self.last_docking_state)
            return
        if self.is_in_view:
            [theta, distance, theta_bounds] = self.fid2pos(self.dock_aruco_tf)
            if abs(theta)>theta_bounds:
                self.openrover_turn(theta)
            else:
                rospy.logdebug('centered switching to approach state')
                self.set_docking_state('approach')
                self.openrover_stop()

    def approach_state_fun(self):
        self.centering_counter = 0
        if self.is_in_view:
            [theta, distance, theta_bounds] = self.fid2pos(self.dock_aruco_tf)
            if abs(theta)>theta_bounds:
                rospy.logdebug("approach angle exceeded: %f", abs(theta))
                self.openrover_stop()
                self.set_docking_state('centering')
            else:
                if self.action_state == 'jogging':
                    return
                if abs(distance) < self.FINAL_APPROACH_DISTANCE:
                    self.openrover_stop()
                    self.openrover_forward(2*self.FINAL_APPROACH_DISTANCE)
                    self.set_docking_state('final_approach')
                else:
                    self.openrover_forward(self.JOG_DISTANCE)
        else:
            self.openrover_stop()
            self.openrover_forward(2*self.FINAL_APPROACH_DISTANCE)
            self.set_docking_state('final_approach')

    def final_approach_state_fun(self):
        [theta, distance, theta_bounds] = self.fid2pos(self.dock_aruco_tf)
        if self.is_in_view and abs(distance) >self.FINAL_APPROACH_DISTANCE:
            self.openrover_stop()
            self.set_docking_state('approach')
            return
        if self.action_state == 'jogging':
            return
        if self.action_state == '':
            self.full_reset()
            self.set_docking_state('docking_failed')

    def undock_state_fun(self):
        if self.action_state == 'jogging':
            return
        if self.action_state == 'turning':
            return
        if self.undocking_state == '':
            rospy.logwarn("Backup")
            self.openrover_forward(-self.UNDOCK_DISTANCE)
            self.undocking_state = 'reversing'
            return
        if self.undocking_state == 'reversing':
            rospy.logwarn("Undock turning")
            self.openrover_turn(self.UNDOCK_TURN_AMOUNT)
            self.undocking_state = 'turning'
            return
        self.set_docking_state('undocked')

    ##---General Functions
    def set_docking_state(self, new_docking_state):
        if not self.docking_state == new_docking_state:
            self.last_docking_state = self.docking_state
            self.docking_state = new_docking_state
            rospy.logdebug("new state: %s, last state: %s", self.docking_state, self.last_docking_state)

    def set_action_state(self, new_action_state):
        if not self.action_state == new_action_state:
            self.last_action_state = self.action_state
            self.action_state = new_action_state
            #rospy.logdebug("new astate: %s, last astate: %s", self.action_state, self.last_action_state)

    def publish_docking_state(self): #Publish docking state if it has changed
        if not (self.docking_state_msg.data == self.docking_state):
            self.docking_state_msg.data = self.docking_state
            self.pub_docking_state.publish(self.docking_state_msg)

    def publish_action_state(self, string_in): #Publish docking state if it has changed
        self.action_state_msg.data = string_in
        self.pub_action_state.publish(self.action_state_msg)

    def full_reset(self): #reset all variables to startup conditions
        self.cmd_vel_angular = 0
        self.cmd_vel_linear = 0
        self.is_in_view = False
        self.is_docked = False
        self.aruco_last_time = rospy.Time()
        self.set_action_state('')
        self.undocking_state = ''
        self.centering_counter = 0
        try:
            self.docking_timer.shutdown()
            rospy.logdebug('full_reset shutdown docking_timer: ')
        except:
            pass
        try:
            self.undocked_timer.shutdown()
            rospy.logdebug('full_reset shutdown undocked_timer: ')
        except:
            pass

    def openrover_forward(self, distance):
        if self.action_state=='':
            self.set_action_state('jogging')
            jog_period = abs(distance/self.CMD_VEL_LINEAR_RATE)
            rospy.logdebug('jog_period: %f', jog_period)
            self.linear_timer = rospy.Timer(rospy.Duration(jog_period), self.openrover_linear_timer_cb, oneshot=True)
            if distance>0:
                rospy.logdebug("Moving forward")
                self.cmd_vel_linear = self.CMD_VEL_LINEAR_RATE
            else:
                rospy.logdebug("Moving Backward")
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
        if r > 3.0:
            theta_bounds = self.APPROACH_ANGLE
        else:
            theta_bounds = r/30.0
        #if abs(theta)<APPROACH_ANGLE:
        #rospy.logdebug("z=%fm and x=%fm", z_trans, x_trans)
        #rospy.logdebug("Theta: %3.3f, r: %3.3f, x_trans: %3.3f, z_trans: %3.3f, x: %3.3f, y: %3.3f, z: %3.3f", theta, r, x_trans, z_trans, euler_angles[0], euler_angles[1], euler_angles[2])
        #rospy.logdebug("Theta: %3.3f, r: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds)
        return theta, r, theta_bounds

    def openrover_stop(self):
        self.set_action_state('')
        self.cmd_vel_msg.twist.linear.x = 0
        self.cmd_vel_msg.twist.angular.z = 0
        self.finished_action_time = rospy.Time.now()
        try:
            self.linear_timer.shutdown()
        except:
            pass
        try:
            self.turn_timer.shutdown()
        except:
            pass

    def openrover_turn(self, radians):
        if self.action_state=='':
            self.set_action_state('turning')
            turn_period = abs(radians/self.CMD_VEL_ANGULAR_RATE) + self.MOTOR_RESPONSE_DELAY
            self.turn_timer = rospy.Timer(rospy.Duration(turn_period), self.openrover_turn_timer_cb, oneshot=True)
            if radians>0:
                rospy.logdebug("Turn right for %f", turn_period)
                self.cmd_vel_angular = -self.CMD_VEL_ANGULAR_RATE
            else:
                rospy.logdebug("Turn Left for %f", turn_period)
                self.cmd_vel_angular = self.CMD_VEL_ANGULAR_RATE
        self.cmd_vel_msg.twist.angular.z = self.cmd_vel_angular

    def disable_aruco_detections(self):
        if self.enable_detections == True:
            try:
                self.enable_detections = False
                resp = self.set_enable_detections(False)
                return
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def enable_aruco_detections(self):
        if self.enable_detections == False:
            try:
                self.enable_detections = True
                resp = self.set_enable_detections(True)
                return
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    ##---Callbacks
    def undock_cb(self, event):
        rospy.logdebug('undock_cb')
        if event.data == True and not self.docking_state=='cancelled':
            self.openrover_stop()
            self.full_reset()
            self.set_docking_state('undock')
            self.set_action_state('')

    def cancel_cb(self, event):
        rospy.logdebug('cancel_cb')
        if event.data:
            self.set_docking_state('cancelled')
            self.openrover_stop()
            self.full_reset()
            self.cancelled_timer = rospy.Timer(rospy.Duration(self.CANCELLED_TIMEOUT), self.cancelled_timer_cb, oneshot=True)

    def cancelled_timer_cb(self, event):
        rospy.logdebug('cancelled_timer_cb')
        self.set_docking_state('undocked')

    def start_cb(self, event):
        rospy.logdebug("start_cb")
        self.openrover_stop()
        rospy.sleep(self.START_DELAY)
        if event.data and not (self.docking_state=='docked'):
            self.set_docking_state('searching')
            self.docking_timer = rospy.Timer(rospy.Duration(self.MAX_RUN_TIMEOUT), self.docking_failed_cb, oneshot=True)

    def wait_now_cb(self, event):
        rospy.logdebug("wait_now_cb")
        if not (self.docking_state=='docked') and not (self.docking_state=='cancelled'):
            self.set_docking_state('undocked')
            self.openrover_stop()
            try:
                self.docking_timer.shutdown()
            except:
                pass

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

            #If no aruco cb's happen within ARUCO_WAIT_TIMEOUT seconds, then assume the image pipe has been disconnected and go into undocked state
            try:
                self.undocked_timer.shutdown()
                self.undocked_timer = rospy.Timer(rospy.Duration(self.ARUCO_WAIT_TIMEOUT), self.wait_now_cb, oneshot=True)
            except:
                self.undocked_timer = rospy.Timer(rospy.Duration(self.ARUCO_WAIT_TIMEOUT), self.wait_now_cb, oneshot=True)

            if self.action_state == 'count_aruco_callbacks': #pause while looking for a certain number of images
                self.aruco_callback_counter = self.aruco_callback_counter + 1
                rospy.logdebug("Aruco count %i", self.aruco_callback_counter)
            else:
                self.aruco_callback_counter = 0

            if len(fid_tf_array.transforms)>0:
                time_delta = (fid_tf_array.header.stamp - self.finished_action_time)
                if (time_delta) < rospy.Duration(self.ACTION_DELAY):
                    rospy.logwarn("auto_dock Old aruco image. Discarding detections. %f ", (time_delta.secs + (time_delta.nsecs/1000000000.0)))
                    return
                for transform in fid_tf_array.transforms:
                    if transform.fiducial_id == self.DOCK_ARUCO_NUM:
                        #If there is no 0 index of transform, then dock Aruco was not found
                        self.last_dock_aruco_tf = self.dock_aruco_tf
                        self.dock_aruco_tf = transform
                        self.is_in_view = True
                        rospy.logdebug('marker detected')
                        [theta, r, theta_bounds] = self.fid2pos(self.dock_aruco_tf) #for debugging
                        if self.docking_state=='searching':
                            rospy.logdebug('marker detected')
                            self.openrover_stop()
                            self.aruco_callback_counter = 0
                            self.set_docking_state('centering')
                        break
            else:
                self.is_in_view = False

    def openrover_turn_timer_cb(self, event):
        rospy.logdebug("openrover_turn_timer_cb: Turning ended")
        self.openrover_stop()

    def openrover_linear_timer_cb(self, event):
        rospy.logdebug("openrover_linear_timer_cb: Stop moving forward")
        self.openrover_stop()

    def openrover_charging_cb(self, charging_msg):
        self.is_docked = charging_msg.data
        if self.docking_state == 'undock':
            return
        if self.is_docked:
            if not self.docking_state=='docked':
                self.full_reset()
                self.openrover_stop()
                self.set_docking_state ('docked')
        else:
            if self.docking_state == 'docked':
                self.set_docking_state('undocked')

    def docking_failed_cb(self, event):
        rospy.logdebug("Docking failed cb")
        self.full_reset()
        self.openrover_stop()
        self.set_docking_state('docking_failed')
        #rospy.logdebug("Docking failed")

def auto_dock_main():
    docking_manager = ArucoDockingManager()
    rospy.spin()

if __name__ == '__main__':
    try:
        # Initialize docking node
        rospy.init_node('auto_dock', log_level=rospy.DEBUG)
        auto_dock_main()
    except rospy.ROSInterruptException:
        pass
