#!/usr/bin/env python3
# license removed for brevity
import rospy
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
import os
import sys
import numpy as np

global horizontalerror, verticalerror, sizeerror
global time_lastbox

global setpoint_size = .2 #fraction of frame that should be filled by target. Largest axis (height or width) used.
global deadzone_size = 0.0 #deadzone on controlling size
global deadzone_position = 0.0 #deadzone on controlling position in frame

global size_gain = 2
global yaw_gain = 1
global pitch_gain = 200
global yaw_mode = True
global traverse_gain = 2

global limit_speed = 2
global limit_yawrate = .75
global limit_pitchchange = 200
global pitchcommand = 1500 # 1000 = 0, 2000 = 44 down

def boundingbox_callback(box):
    global horizontalerror, verticalerror, sizeerror, setpoint_size
    global time_lastbox
    #positive errors right, down, forward
    horizontalerror = .5-box.center.x
    verticalerror = .5-box.center.y
    sizeerror = setpoint_size - max(box.size_x, box.size_y)
    time_lastbox = rospy.Time.now()
    return

def dofeedbackcontrol():
    global horizontalerror, verticalerror, sizeerror
    global time_lastbox
    global setpoint_size, deadzone_size, deadzone_position
    global size_gain, yaw_gain, pitch_gain,yaw_mode,traverse_gain
    global limit_speed, limit_yawrate, limit_pitchchange

    #Initialize publishers/subscribers/node
    rospy.Subscriber('/gaia/bounding_box', BoundingBox2D, boundingbox_callback)
    twistpub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
    rcpub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', OverrideRCIn, queue_size=1)
    rospy.init_node('feedbacknode', anonymous=False)

    # control loop
    twistmsg = Twist()
    rcmsg = OverrideRCIn()
    rcmessage.channels = np.zeros(18,dtype=int).tolist()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #feedback control algorithm
        #don't publish if message is old
        if (rospy.Time.now() - time_lastbox < rospy.Duration(.5)):
            #calculate raw commands
            if pitchcommand < 1800:
                fspeed = sizeerror * size_gain
            else:
                fspeed = 0
            yawrate = horizontalerror * yaw_gain
            hspeed = horizontalerror * traverse_gain
            pitchdelta = verticalerror * pitch_gain
            pitchdelta = min(max(pitchdelta,-limit_pitchchange),limit_pitchchange)
            pitchcommand += pitchdelta
            
            #bound controls to ranges
            fspeed = min(max(fspeed,-limit_speed),limit_speed) #lower bound first, upper bound second
            hspeed = min(max(hspeed,-limit_speed),limit_speed)
            yawrate = min(max(yawrate,-limit_yawrate),limit_yawrate)
            pitchcommand = min(max(pitchcommand,1000),2000)
            #assign to messages, publish
            twistmsg.linear.x = fspeed
            if yaw_mode:
                twismsg.linear.y = 0
                twistmsg.angular.z = yawrate
            else:
                twistmsg.linear.y = hspeed
                twistmsg.angular.z = 0
            rcmessage.channels[7] = pitchcommand
        
            twistpub.publish(twistmsg)
            rcpub.publish(rcmsg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        dofeedbackcontrol()
    except rospy.ROSInterruptException:
        pass