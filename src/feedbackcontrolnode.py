#!/usr/bin/env python3
# license removed for brevity
import rospy
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn
import os
import sys
import numpy as np

global horizontalerror, verticalerror, sizeerror
time_lastbox = None

setpoint_size = .2 #fraction of frame that should be filled by target. Largest axis (height or width) used.
deadzone_size = 0.0 #deadzone on controlling size
deadzone_position = 0.0 #deadzone on controlling position in frame

size_gain = 2
yaw_gain = 1
pitch_gain = -100
yaw_mode = True
traverse_gain = 2

limit_speed = 2
limit_yawrate = .75
limit_pitchchange = 50

pitchcommand = 1500

yaw = 0

def yaw_callback(pose):
    #TODO: Compute and store yaw
    global yaw
    q = pose.orientation
    yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    print(yaw)
    pass

def boundingbox_callback(box):
    global horizontalerror, verticalerror, sizeerror
    global time_lastbox, pitchcommand
    #positive errors right, down, forward
    horizontalerror = .5-box.center.x
    verticalerror = .5-box.center.y
    sizeerror = setpoint_size - max(box.size_x, box.size_y)
    time_lastbox = rospy.Time.now()
    pitchdelta = verticalerror * pitch_gain
    pitchdelta = min(max(pitchdelta,-limit_pitchchange),limit_pitchchange)
    pitchcommand += pitchdelta
    return

def dofeedbackcontrol():
    global pitchcommand
    #Initialize publishers/subscribers/node
    print("Initializing feedback node...")
    rospy.Subscriber('/gaia/bounding_box', BoundingBox2D, boundingbox_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, yaw_callback)
    twistpub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
    rcpub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    rospy.init_node('feedbacknode', anonymous=False)

    # control loop
    twistmsg = Twist()
    rcmsg = OverrideRCIn()
    rcmsg.channels = np.zeros(8,dtype=np.uint16).tolist()
    rate = rospy.Rate(10) # 10hz

    print("Feedback node initialized, starting control")
    while not rospy.is_shutdown():
        #feedback control algorithm
        #don't publish if message is old
        if time_lastbox != None and rospy.Time.now() - time_lastbox < rospy.Duration(.5):
            print("Time check passed\n")
            #calculate raw commands
            if pitchcommand < 1800:
                fspeed = sizeerror * size_gain
            else:
                fspeed = 0
            yawrate = horizontalerror * yaw_gain
            hspeed = horizontalerror * traverse_gain
            pitchdelta = verticalerror * pitch_gain
            pitchdelta = min(max(pitchdelta,-limit_pitchchange),limit_pitchchange)
            #pitchcommand += pitchdelta
            
            #bound controls to ranges
            fspeed = min(max(fspeed,-limit_speed),limit_speed) #lower bound first, upper bound second
            hspeed = min(max(hspeed,-limit_speed),limit_speed)
            yawrate = min(max(yawrate,-limit_yawrate),limit_yawrate)
            pitchcommand = min(max(pitchcommand,1000),2000)
            rcmsg.channels[7] = int(pitchcommand)
            #assign to messages, publish
            twistmsg.linear.y = fspeed
            if yaw_mode:
                twistmsg.linear.x = 0
                twistmsg.angular.z = yawrate
            else:
                twistmsg.linear.x = hspeed
                twistmsg.angular.z = 0
            print("Publishing messages")
            twistpub.publish(twistmsg)
            rcpub.publish(rcmsg)
        elif time_lastbox != None and (rospy.Time.now() - time_lastbox > rospy.Duration(5)):
            pitchcommand = 1500

        
        rate.sleep()

if __name__ == '__main__':
    try:
        dofeedbackcontrol()
    except rospy.ROSInterruptException:
        pass
