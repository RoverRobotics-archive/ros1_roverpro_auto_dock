#!/usr/bin/env python

# Author: Jack Kilian
# Description: This script auto_docks the openrover basic platform if it is in a 3m circle in front of the dock.

import rospy
from std_msgs.msg import Float32, String, Bool, Int
import os
import time
    
# Main Function
def joystick_main():

    # Initialize driver node
    rospy.init_node('rr_auto_dock', anonymous=True)
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()
    
if __name__ == '__main__':
    try:
        joystick_main()
    except rospy.ROSInterruptException:
pass
