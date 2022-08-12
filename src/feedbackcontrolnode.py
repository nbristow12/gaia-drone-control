#!/home/ffil/gaia-feedback-control/gaia-fc-env/bin/python3
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

#------------OPTIONS FOR MODES-------#
OPT_FLOW = False # true means optical flow is used
top_down_mode = False  # different operating mode for motion if the drone is far above the feature
hybrid_mode = False # starts horizontal, moves toward object, then once above it, moves into the top-down-mode
yaw_mode = True # whether or not to yaw the entire drone during motion
#---------------------------------------------------#

# bounding box options
setpoint_size = .6 #fraction of frame that should be filled by target. Largest axis (height or width) used.
setpoint_size_approach = 1 # only relevant for hybrid mode, for getting close to target
# deadzone_size = 0.0 #deadzone on controlling size
# deadzone_position = 0.0 #deadzone on controlling position in frame

# optical flow parameters
alt_flow = 6 # altitude at which to begin optical flow

# gain values
size_gain = 5
yaw_gain = 0.75
gimbal_pitch_gain = -100
gimbal_yaw_gain = 40

traverse_gain = 2.5
flow_gain = 20
vertical_gain = 3 # half of the size_gain value

# safety parameters
alt_min = 4 # minimum allowable altitude
alt_delta=2 # how high to move after crossing munumum alt
limit_speed = 2
limit_speed_v = 0.5 # different speed limit for changing altitude
limit_yawrate = .4
limit_pitchchange = 50
limit_yawchange = 50

# initialize
horizontalerror = 0 
verticalerror=0 
sizeerror=0 
vspeed = 0 # positive is upwards
hspeed = 0 # positive is to the right
fspeed = 0 # positive is forwards
yaw = 0
move_up = False # initialized value
alt = 10 # initialized value outside the safegaurd
above_object = False
pitch_down = 1850
pitch_thresh = 1800
yaw_center = 1500
if top_down_mode:
    pitchcommand = pitch_down # looking down
else:
    pitchcommand = 1650
yawcommand = yaw_center


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

def pose_callback(pose):
    global yaw,alt
    q = pose.pose.orientation
    # yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    r,p,y = euler_from_quaternion(q.x,q.y,q.z,q.w)
    yaw = y
    alt = pose.pose.position.z
    # print(yaw)


def boundingbox_callback(box):
    global horizontalerror, verticalerror, sizeerror
    global time_lastbox, pitchcommand, yawcommand
    # positive errors give right, up
    if box.bbox.center.x != -1:
        time_lastbox = rospy.Time.now()
        if above_object: # different bbox size desired for approach and above stages for hybrid mode
            sizeerror = setpoint_size_approach - max(box.bbox.size_x, box.bbox.size_y) # if box is smaller than setpoit, error is positive
        else:
            sizeerror = setpoint_size - max(box.bbox.size_x, box.bbox.size_y) # if box is smaller than setpoit, error is positive
        if not OPT_FLOW:
            horizontalerror = .5-box.bbox.center.x # if box center is on LHS of image, error is positive
            verticalerror = .5-box.bbox.center.y # if box center is on upper half of image, error is positive 
            print('Horzontal error: %f' % horizontalerror)
            print('Vertical error: %f' % verticalerror)

        pitchdelta = verticalerror * gimbal_pitch_gain
        pitchdelta = min(max(pitchdelta,-limit_pitchchange),limit_pitchchange)
        pitchcommand += pitchdelta
        pitchcommand = min(max(pitchcommand,1000),2000)
        yawdelta = horizontalerror * gimbal_yaw_gain
        yawdelta = min(max(yawdelta,-limit_yawchange),limit_yawchange)
        yawcommand += yawdelta
        yawcommand = min(max(yawcommand,1000),2000)
        
    return

def flow_callback(flow):
    global horizontalerror, verticalerror,time_lastbox
    # adjust the feedback error using the optical flow
    if OPT_FLOW:
        print('doing optical flow feedback')
        horizontalerror = flow.size_x * flow_gain
        verticalerror = flow.size_y * flow_gain
        time_lastbox = rospy.Time.now()
    return

def dofeedbackcontrol():
    global pitchcommand, yawcommand
    global above_object
    #Initialize publishers/subscribers/node
    print("Initializing feedback node...")
    rospy.Subscriber('/gaia/bounding_box', Detection2D, boundingbox_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, alt_callback)
    rospy.Subscriber('/gaia/flow',BoundingBox2D,flow_callback)
    twistpub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
    rcpub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    rospy.init_node('feedbacknode', anonymous=False)

    # print(rospy.get_published_topics())

    # control loop
    twistmsg = Twist()
    rcmsg = OverrideRCIn()
    rcmsg.channels = np.zeros(18,dtype=np.uint16).tolist()
    rate = rospy.Rate(10) # 10hz


    print("Feedback node initialized, starting control")
    while not rospy.is_shutdown():
        # print(alt)
        #feedback control algorithm
        #don't publish if message is old
        # if time_lastbox != None and rospy.Time.now() - time_lastbox < rospy.Duration(.5):
        if True:
            # safeguard for vertical motion
            if alt < alt_min:
                move_up = True # will move up until desired altitude, to reset
            elif alt > alt_min+alt_delta:
                move_up = False     # desired altitude reached
            
            if alt < alt_flow and above_object:
                OPT_FLOW = True # switch to optical flow feedback control mode

            if top_down_mode:
                
                # lateral movement (hspeed > 0 moves right)
                hspeed = -horizontalerror * traverse_gain
                # forward movement   (fspeed > 0 move backward)
                fspeed = verticalerror * traverse_gain

                # vertical movement depending on the minimum altitude safeguard
                # vspeed > 0 moves upward
                if move_up:
                    vspeed = 1
                else:
                    # vspeed=-1
                    vspeed = -sizeerror * vertical_gain # size error is negative because you want to move down (negative velocity) to get closer

                #bound controls to ranges
                fspeed = min(max(fspeed,-limit_speed),limit_speed) #lower bound first, upper bound second
                hspeed = min(max(hspeed,-limit_speed),limit_speed)
                vspeed = min(max(vspeed,-limit_speed_v),limit_speed_v) # vertical speed
                
                
                yawrate = 0 # don't rotate/yaw teh drone if in top-down mode 
                yawcommand = yaw_center   # stay centered
                pitchcommand = pitch_down # stare down

                yawrate = min(max(yawrate,-limit_yawrate),limit_yawrate)
                yawcommand = min(max(yawcommand,1000),2000)
                pitchcommand = min(max(pitchcommand,1000),2000)

                # vspeed=0
                # hspeed=0
                # fspeed=0
                
                #assign to messages, publish
                rcmsg.channels[7] = int(pitchcommand) #send pitch command on channel 8
                rcmsg.channels[6] = int(yawcommand) #send yaw command on channel 7
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
            elif hybrid_mode:
                
                # determine if above object based on pitch
                if pitchcommand > pitch_thresh:
                    above_object=True
                else:
                    above_object=False
                
                # lateral movement (hspeed > 0 moves right)
                hspeed = -horizontalerror * traverse_gain
                # forward movement   (fspeed > 0 move backward)
                if above_object:
                    fspeed = verticalerror * traverse_gain
                else:
                    fspeed = sizeerror * size_gain
                # vertical movement depending on the minimum altitude safeguard
                # vspeed > 0 moves upward
                if move_up:
                    vspeed = 1
                else:
                    if above_object:
                        vspeed = -sizeerror * vertical_gain # size error is negative because you want to move down (negative velocity) to get closer
                    else:
                        vspeed=0

                #bound controls to ranges
                fspeed = min(max(fspeed,-limit_speed),limit_speed) #lower bound first, upper bound second
                hspeed = min(max(hspeed,-limit_speed),limit_speed)
                vspeed = min(max(vspeed,-limit_speed_v),limit_speed_v) # vertical speed
                
                if above_object:
                    yawrate = 0 # don't rotate/yaw teh drone if in top-down mode 
                else:
                    yawrate = ((yawcommand - 1500)/1000)*yaw_gain #.75 multiplier included here for now, should be pulled out to gain later


                yawrate = min(max(yawrate,-limit_yawrate),limit_yawrate)
                yawcommand = min(max(yawcommand,1000),2000)
                pitchcommand = min(max(pitchcommand,1000),2000)

                # vspeed=0
                # hspeed=0
                # fspeed=0
                
                #assign to messages, publish
                rcmsg.channels[7] = int(pitchcommand) #send pitch command on channel 8
                rcmsg.channels[6] = int(yawcommand) #send yaw command on channel 7
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

            else: # nik's old version
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
                hspeed = -horizontalerror * traverse_gain
                # pitchdelta = verticalerror * gimbal_pitch_gain
                # pitchdelta = min(max(pitchdelta,-limit_pitchchange),limit_pitchchange)
                #pitchcommand += pitchdelta
                
                #bound controls to ranges
                fspeed = min(max(fspeed,-limit_speed),limit_speed) #lower bound first, upper bound second
                hspeed = min(max(hspeed,-limit_speed),limit_speed)
                yawrate = min(max(yawrate,-limit_yawrate),limit_yawrate)
                pitchcommand = min(max(pitchcommand,1000),2000)
                # pitchcommand=1850
                # print('Pitch: %f' % pitchcommand)
                # print('Yawcommand: %f' % yawcommand)
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
        # elif time_lastbox != None and (rospy.Time.now() - time_lastbox > rospy.Duration(5)):
        #     # pitchcommand = 1500
        #     pitchcommand = 1850 # looking down
        #     yawcommand = 1500

        rate.sleep()

if __name__ == '__main__':
    try:
        dofeedbackcontrol()
    except rospy.ROSInterruptException:
        pass

