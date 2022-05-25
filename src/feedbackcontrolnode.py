#!/usr/bin/env python3
# license removed for brevity
import rospy
from vision_msgs.msg import BoundingBox2D,Detection2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn
import math
from math import atan2
import os
import sys
import numpy as np

global horizontalerror, verticalerror, sizeerror
time_lastbox = None

top_down_mode = True  # different operating mode for motion if the drone is far above the feature

setpoint_size = .5 #fraction of frame that should be filled by target. Largest axis (height or width) used.
deadzone_size = 0.0 #deadzone on controlling size
deadzone_position = 0.0 #deadzone on controlling position in frame

vertical_gain = 2 # half of the size_gain value
size_gain = 6
yaw_gain = 1
gimbal_pitch_gain = -100
gimbal_yaw_gain = 40
yaw_mode = False
traverse_gain = 3
flow_gain = 10

limit_speed = 2
limit_speed_v = 0.5 # different speed limit for changing altitude
limit_yawrate = .4
limit_pitchchange = 50
limit_yawchange = 50

# pitchcommand = 1500
pitchcommand = 1850 # looking down
yawcommand = 1500

yaw = 0
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def yaw_callback(pose):
    global yaw
    q = pose.pose.orientation
    # yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    r,p,y = euler_from_quaternion(q.x,q.y,q.z,q.w)
    yaw = y
    # print(pose.pose.position.z,end='\r') # printing altitude
    # print(yaw)

def boundingbox_callback(box):
    global horizontalerror, verticalerror, sizeerror
    global time_lastbox, pitchcommand, yawcommand
    #positive errors left, up, forward
    if box.bbox.center.x != -1:
        horizontalerror = .5-box.bbox.center.x
        verticalerror = .5-box.bbox.center.y
        sizeerror = setpoint_size - max(box.bbox.size_x, box.bbox.size_y)
        time_lastbox = rospy.Time.now()
        pitchdelta = verticalerror * gimbal_pitch_gain
        pitchdelta = min(max(pitchdelta,-limit_pitchchange),limit_pitchchange)
        pitchcommand += pitchdelta
        pitchcommand = min(max(pitchcommand,1000),2000)
        yawdelta = horizontalerror * gimbal_yaw_gain
        yawdelta = min(max(yawdelta,-limit_yawchange),limit_yawchange)
        # yawcommand += yawdelta
        # yawcommand = min(max(yawcommand,1000),2000)
        yawcommand = 1500
    return

def flow_callback(flow):
    global horizontalerror, verticalerror
    # adjust the feedback error using the optical flow
    print('doing optical flow feedback')
    horizontalerror += flow.size_x * flow_gain
    verticalerror += flow.size_y * flow_gain

    return

def dofeedbackcontrol():
    global pitchcommand, yawcommand
    #Initialize publishers/subscribers/node
    print("Initializing feedback node...")
    rospy.Subscriber('/gaia/bounding_box', Detection2D, boundingbox_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, yaw_callback)
    rospy.Subscriber('/gaia/flow',BoundingBox2D,flow_callback)
    twistpub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
    rcpub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    rospy.init_node('feedbacknode', anonymous=False)

    # control loop
    twistmsg = Twist()
    rcmsg = OverrideRCIn()
    rcmsg.channels = np.zeros(18,dtype=np.uint16).tolist()
    rate = rospy.Rate(10) # 10hz

    print("Feedback node initialized, starting control")
    while not rospy.is_shutdown():
        #feedback control algorithm
        #don't publish if message is old
        if time_lastbox != None and rospy.Time.now() - time_lastbox < rospy.Duration(.5):
            # print("Time check passed\n")

            # if pitchcommand > 1675:
            #     top_down_mode= True
            # else:
            #     top_down_mode = False
            top_down_mode = True
            
            if top_down_mode:

                #calculate raw commands
                # if pitchcommand < 1800: 
                #     fspeed = sizeerror * size_gain
                # else: # if the gimbal is pitching down to see (i.e., pitch servo > 1800), stop going forward
                #     fspeed = 0
                # yawrate = horizontalerror * yaw_gain #commented out when gimbal yaw is active
                yawrate = 0
                # yawrate = ((yawcommand - 1500)/1000)*yaw_gain*.75 #.75 multiplier included here for now, should be pulled out to gain later
                hspeed = -horizontalerror * traverse_gain
                fspeed = verticalerror * traverse_gain
                # hspeed = 0
                # fspeed = 0
                
                vspeed = -sizeerror * vertical_gain # size error is negative because you want to move down (negative velocity) to get closer
                # vspeed = 1
                # pitchdelta = verticalerror * gimbal_pitch_gain
                # pitchdelta = min(max(pitchdelta,-limit_pitchchange),limit_pitchchange)
                #pitchcommand += pitchdelta
                
                #bound controls to ranges
                fspeed = min(max(fspeed,-limit_speed),limit_speed) #lower bound first, upper bound second
                hspeed = min(max(hspeed,-limit_speed),limit_speed)
                vspeed = min(max(vspeed,-limit_speed_v),limit_speed_v) # vertical speed

                yawrate = min(max(yawrate,-limit_yawrate),limit_yawrate)
                # pitchcommand = min(max(pitchcommand,1000),2000)
                pitchcommand = 1850 # setting to just look down
                # yawcommand = min(max(yawcommand,1000),2000)
                yawcommand = 1500 # fixing in middle
                rcmsg.channels[7] = int(pitchcommand) #send pitch command on channel 8
                rcmsg.channels[6] = int(yawcommand) #send yaw command on channel 7
                #assign to messages, publish
                if yaw_mode:
                    twistmsg.linear.x = math.cos(yaw)*fspeed
                    twistmsg.linear.y = math.sin(yaw)*fspeed
                    twistmsg.linear.z = vspeed  # adding vertical motion
                    twistmsg.angular.z = yawrate
                else:
                    twistmsg.linear.x = math.cos(yaw)*fspeed + math.sin(yaw)*hspeed
                    twistmsg.linear.y = math.sin(yaw)*fspeed - math.cos(yaw)*hspeed
                    twistmsg.linear.z = vspeed  # adding vertical motion
                    twistmsg.angular.z = 0

            else:
                # set vertical motion to zero
                vspeed = 0
                twistmsg.linear.z = vspeed
                #calculate raw commands
                if pitchcommand < 1800: 
                    fspeed = sizeerror * size_gain
                else: # if the gimbal is pitching down to see (i.e., pitch servo > 1800), stop going forward
                    fspeed = 0
                # yawrate = horizontalerror * yaw_gain #commented out when gimbal yaw is active
                yawrate = ((yawcommand - 1500)/1000)*yaw_gain*.75 #.75 multiplier included here for now, should be pulled out to gain later
                hspeed = horizontalerror * traverse_gain
                # pitchdelta = verticalerror * gimbal_pitch_gain
                # pitchdelta = min(max(pitchdelta,-limit_pitchchange),limit_pitchchange)
                #pitchcommand += pitchdelta
                
                #bound controls to ranges
                fspeed = min(max(fspeed,-limit_speed),limit_speed) #lower bound first, upper bound second
                hspeed = min(max(hspeed,-limit_speed),limit_speed)
                yawrate = min(max(yawrate,-limit_yawrate),limit_yawrate)
                pitchcommand = min(max(pitchcommand,1000),2000)
                # yawcommand = min(max(yawcommand,1000),2000)
                rcmsg.channels[7] = int(pitchcommand) #send pitch command on channel 8
                rcmsg.channels[6] = int(yawcommand) #send yaw command on channel 7
                #assign to messages, publish
                if yaw_mode:
                    twistmsg.linear.x = math.cos(yaw)*fspeed
                    twistmsg.linear.y = math.sin(yaw)*fspeed
                    twistmsg.angular.z = yawrate
                else:
                    twistmsg.linear.x = math.cos(yaw)*fspeed + math.sin(yaw)*hspeed
                    twistmsg.linear.y = math.sin(yaw)*fspeed - math.cos(yaw)*hspeed
                    twistmsg.angular.z = 0

            # print("Publishing messages")
            twistpub.publish(twistmsg)
            rcpub.publish(rcmsg)
        elif time_lastbox != None and (rospy.Time.now() - time_lastbox > rospy.Duration(5)):
            # pitchcommand = 1500
            pitchcommand = 1850 # looking down
            yawcommand = 1500

        # twistmsg.linear.z = 1
        # print("Publishing messages")
        # twistpub.publish(twistmsg)
        # rcpub.publish(rcmsg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        dofeedbackcontrol()
    except rospy.ROSInterruptException:
        pass
