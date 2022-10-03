#!/usr/bin/python3 
# ######!/usr/bin/env python3 

# license removed for brevity
from tkinter import image_types
import rospy
from sensor_msgs.msg import Image
import os, datetime
import PySpin
import simple_pyspin as spy
import sys, subprocess, time
import cv2
from goprocam import GoProCamera
from goprocam import constants
import queue, threading
from pathlib import Path
sys.path.append("goproapi")
import re
import numpy as np
import time
#--------OPTION TO VIEW ACQUISITION IN REAL_TIME-------------#
VIEW_IMG=True
save_image = True
save_format = '.avi'
dec = 2
#-----------------------------------------------------#


# create saving directory
username = os.getlogin( )
tmp = datetime.datetime.now()
stamp = ("%02d-%02d-%02d" % 
    (tmp.year, tmp.month, tmp.day))
maindir = Path('/home/%s/1FeedbackControl' % username)
runs_today = list(maindir.glob('*%s*_camera' % stamp))
if runs_today:
    runs_today = [str(name) for name in runs_today]
    regex = 'run\d\d'
    runs_today=re.findall(regex,''.join(runs_today))
    runs_today = np.array([int(name[-2:]) for name in runs_today])
    new_run_num = max(runs_today)+1
else:
    new_run_num = 1
savedir = maindir.joinpath('%s_run%02d_camera' % (stamp,new_run_num))
os.makedirs(savedir)  

#seems to find device automatically if connected? Why don't we do this?
serialstring = 'DeviceSerialNumber'
#serialstring = '18285036'


camlist = spy.list_cameras()
if len(camlist)>0:

    cam = spy.Camera(0)
    
else:
    raise Exception('NO CAMERAS FOUND')

cam.init()

cam.cam.DecimationHorizontal.SetValue(1)
cam.cam.DecimationVertical.SetValue(1)
cam.OffsetX = 0
cam.OffsetY = 0
cam.Width = cam.WidthMax # better than using cam.SensorWidth
cam.Height = cam.HeightMax

# image sensor binning and decimation options
# cam.cam.BinningHorizontal.SetValue(args.binning_x)
# cam.cam.BinningVertical.SetValue(args.binning_y)
cam.cam.DecimationHorizontal.SetValue(dec)
cam.cam.DecimationVertical.SetValue(dec)

cam.start()

def publishimages():
    pub = rospy.Publisher('/camera/image', Image, queue_size=1)
    rospy.init_node('cameranode', anonymous=False)

    """
    This function acquires images from a device.

    :param cam: Camera to acquire images from.
    :param nodemap: Device nodemap.
    :param nodemap_tldevice: Transport layer device nodemap.
    :type cam: CameraPtr
    :type nodemap: INodeMap
    :type nodemap_tldevice: INodeMap
    :return: True if successful, False otherwise.
    :rtype: bool
    """

    # capture_init()

    # gpCam = GoProCamera.GoPro()
    device_serial_number = False

    # initializing timelog
    timelog = open(savedir.joinpath('Timestamps.csv'),'w')
    timelog.write('FrameID,Timestamp\n')

    try:
        result = True
        
        # cap = VideoCapture("udp://127.0.0.1:10000") # stream from gopro wifi
        


        # Retrieve, convert, and save images
        i = 0
        first_image=True
        img = Image()
        while not rospy.is_shutdown():
            i += 1
            try:

                #  Retrieve next received image


                img_raw = cam.get_array()
                # img_raw = cv2.cvtColor(img_raw,cv2.COLOR_BayerRG2BGR)
                # img_raw = cv2.resize(img_raw,(640,480))

                ret = True

                #-----------#
                # img_raw = np.zeros(shape=(480,640,3),dtype=np.uint8)+100
                # ret = True
                # time.sleep(0.1)
                #------------#

                img.header.seq = i
                img.header.stamp = rospy.Time.now()


                # adding to time stamp log
                timelog.write('%d,%f\n' % (img.header.seq,time.time()))
  
                if not ret:
                    print('Image capture with opencv unsuccessful')
                    
                else:


                    img.height,img.width = img_raw.shape[0],img_raw.shape[1]
                    # print('Time at acquisition')
                    # print('Image %d' % img.header.seq)
                    # print(img.header.stamp)


                    # #get numpy image
                    # image_numpy = image_converted.GetNDArray()
                    if VIEW_IMG:
                        cv2.imshow('gopro',img_raw)
                        cv2.waitKey(1)
                        # imgtmp = PILImage.fromarray(img_raw,'RGB')
                        # imgtmp.show()

                    #assign image to ros structure
                    # img.data = image_numpy.flatten().tolist()
                    img.data = img_raw.flatten().tolist()
                    #send image on topic
                    pub.publish(img)

                    if save_image:
                        # Create a unique filename
                        if device_serial_number:
                            filename = savedir.joinpath('Acquisition-%s-%06.0f' % (device_serial_number, i))
                        else:  # if serial number is empty
                            filename = savedir.joinpath('Acquisition-%06.0f' % i)


                        if save_format=='.raw':
                            fid = open(str(filename)+save_format,'wb')
                            fid.write(img_raw.flatten())
                            fid.close()
                        elif save_format == '.avi':
                            if i==1:
                                codec = cv2.VideoWriter_fourcc('M','J','P','G')
                                video = cv2.VideoWriter(str(savedir.joinpath('Acquisition'+save_format)),
                                    fourcc=codec,
                                    fps=10,
                                    frameSize = (img_raw.shape[1],img_raw.shape[0]))
                            video.write(img_raw)
                        else:
                            cv2.imwrite(str(filename)+save_format,img_raw)

            except Exception as e:       
                print('Error: %s' % e)

        #  End acquisition

        # cap.release()
        cam.close()

    

    except Exception as e:       
        print('Error: %s' % e)

    #original rosnode loop code
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     current_image = new Image()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()


def capture_init():
    # starts gopro streaming in background
    
    FILE = Path(__file__).resolve()
    goproapi = Path(FILE.parents[1] / 'src/goproapi')  # gopro functions directory
    cmd = str(goproapi.joinpath('gopro_keepalive.py'))
    # subprocess.call(['python3',cmd])
    with open(os.devnull, 'w') as fp:
        output = subprocess.Popen('python3 ' + cmd, stdout=fp,shell=True)

    return

if __name__ == '__main__':
    try:
        publishimages()
    except rospy.ROSInterruptException:
        pass

