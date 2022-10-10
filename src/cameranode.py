#!/home/ffil/gaia-feedback-control/gaia-fc-env/bin/python3
# ######!/usr/bin/env python3 

# license removed for brevity
from tkinter import image_types
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import TimeReference
import os, datetime
# import PySpin
import sys, subprocess, time
import cv2
from goprocam import GoProCamera
from goprocam import constants
import queue, threading
from pathlib import Path
sys.path.append("goproapi")
import re
import numpy as np
#--------OPTION TO VIEW ACQUISITION IN REAL_TIME-------------#
VIEW_IMG = False
save_image = True
save_format = '.avi'
#-----------------------------------------------------#


# create saving directory
gps_t = 0
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
# /home/ffil/gaia-feedback-control/src/GAIA-drone-control/src/goproapi/gopro_keepalive.py
# bufferless VideoCapture
class VideoCapture:

    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.q = queue.Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

  # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                # break
                continue
            if not self.q.empty():
                try:
                    self.q.get_nowait()   # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.q.get()
    def release(self):
        self.cap.release()
        return

def time_callback(gpstime):
    global gps_t
    gps_t = float(gpstime.time_ref.to_sec())
    # gps_t = gps_t
    
    # print(gps_t)
    # print(rospy.get_time())
    # print(time.time())

def publishimages():
    pub = rospy.Publisher('/camera/image', Image, queue_size=1)
    rospy.init_node('cameranode', anonymous=False)
    rospy.Subscriber('mavros/time_reference',TimeReference,time_callback)

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

    capture_init()

    gpCam = GoProCamera.GoPro()
    #gpCam.gpControlSet(constants.Stream.BIT_RATE, constants.Stream.BitRate.B2_4Mbps)
    #gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.W480)
    device_serial_number = False

    # initializing timelog
    timelog = open(savedir.joinpath('Timestamps.csv'),'w')
    timelog.write('FrameID,Timestamp\n')

    try:
        result = True
        
        # cap = cv2.VideoCapture("udp://127.0.0.1:10000") # stream from gopro wifi
        # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # setting buffer size to 1, only latest frame kept

        cap = VideoCapture("udp://127.0.0.1:10000") # stream from gopro wifi
        

        capt1 = 0
        # Retrieve, convert, and save images
        i = 0
        first_image=True
        img = Image()
        while not rospy.is_shutdown():
            i += 1
            try:

                #  Retrieve next received image
                
                t1 = time.time()
                img_raw = cap.read()

                capt2 = time.time()
                # print('Capture fps ~',1/(capt2-capt1))
                capt1 = capt2

                if img_raw is not None:
                    ret = True
                    if first_image:
                        print('#-------------------------------------------#')
                        print('Image capture successfully started')
                        print('#-------------------------------------------#')
                    first_image = False
                else:
                    ret = False
                
                img.header.seq = i
                img.header.stamp = rospy.Time.now()


                # adding to time stamp log
                # timelog.write('%d,%f\n' % (img.header.seq,time.time()))
                timelog.write('%d,%f\n' % (img.header.seq,gps_t))
  
                if not ret:
                    print('Image capture with opencv unsuccessful')
                    
                else:


                    img.height,img.width = img_raw.shape[:-1]
                    # print('Time at acquisition')
                    # print('Image %d' % img.header.seq)
                    # print(img.header.stamp)


                    # #get numpy image
                    # image_numpy = image_converted.GetNDArray()

                    # print('Delay before showing',1e3*(time.time() - capt2))

                    if VIEW_IMG:
                        cv2.imshow('gopro',img_raw)
                        cv2.waitKey(1)
                        # imgtmp = PILImage.fromarray(img_raw,'RGB')
                        # imgtmp.show()
                    # print('Delay including showing',1e3*(time.time() - capt2))
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
                                    fps=30,
                                    frameSize = (img_raw.shape[1],img_raw.shape[0]))
                            video.write(img_raw)
                        else:
                            cv2.imwrite(str(filename)+save_format,img_raw)

            except Exception as e:       
                print('Error: %s' % e)

        #  End acquisition

        cap.release()

    

    except Exception as e:       
        print('Error: %s' % e)

    #original rosnode loop code
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     current_image = new Image()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

# def gstreamer_pipeline(
#     sensor_id=0,
#     capture_width=640,
#     capture_height=480,
#     display_width=640,
#     display_height=480,
#     framerate=30,
#     flip_method=0,
#     exposure_time=100,
#     gain=0.0,
# ):
#     cmd_init = "nvarguscamerasrc sensor-mode=0 sensor-id=%d" % sensor_id
#     if gain!=0.0:
#          cmd_init = cmd_init + " gainrange='%d %d'" % (gain,gain)
#     if exposure_time!=0:
#          cmd_init = cmd_init + " exposuretimerange='%d %d'" % (exposure_time*1e3,exposure_time*1e3)
#     cmd_init = cmd_init + " !"
#     cmd_main = (
#         "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
#         "nvvidconv flip-method=%d ! "
#         "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
#         "videoconvert ! "
#         "video/x-raw, format=(string)BGR ! appsink"
#         % (
#             capture_width,
#             capture_height,
#             framerate,
#             flip_method,
#             display_width,
#             display_height,
#         )
#     )
#     return cmd_init+cmd_main

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

