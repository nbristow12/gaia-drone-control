#!/usr/bin/env python3
# license removed for brevity
from tkinter import image_types
import rospy
from sensor_msgs.msg import Image
import os, datetime
import PySpin
import sys, subprocess, time
import cv2
from goprocam import GoProCamera
from goprocam import constants
import queue, threading

# sys.path.append("goproapi")

#--------OPTION TO VIEW ACQUISITION IN REAL_TIME-------------#
VIEW_IMG=False
#-----------------------------------------------------#

save_image = True
username = os.getlogin( )
tmp = datetime.datetime.now()
stamp = ("%02d-%02d-%02d__%02d-%02d-%02d" % 
    (tmp.year, tmp.month, tmp.day, 
    tmp.hour, tmp.minute, tmp.second))
savedir = '/home/%s/OutputImages_%s/' % (username,stamp) #script cannot create folder, must already exist when run
os.makedirs(savedir)

#seems to find device automatically if connected? Why don't we do this?
serialstring = 'DeviceSerialNumber'
#serialstring = '18285036'

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
                break
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


    
    gpCam = GoProCamera.GoPro()
    #gpCam.gpControlSet(constants.Stream.BIT_RATE, constants.Stream.BitRate.B2_4Mbps)
    #gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.W480)
    device_serial_number = False

    try:
        result = True
        
        # cap = cv2.VideoCapture("udp://127.0.0.1:10000") # stream from gopro wifi
        # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # setting buffer size to 1, only latest frame kept
        cap = VideoCapture("udp://127.0.0.1:10000") # stream from gopro wifi
        


        # Retrieve, convert, and save images
        i = 0
        first_image=True
        img = Image()
        while not rospy.is_shutdown():
            i += 1
            try:

                #  Retrieve next received image

                
                img_raw = cap.read()
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

  
                if not ret:
                    print('Image capture with opencv unsuccessful')
                    
                else:


                    img.height,img.width = img_raw.shape[:-1]
                    # print('Time at acquisition')
                    # print('Image %d' % img.header.seq)
                    # print(img.header.stamp)


                    # #get numpy image
                    # image_numpy = image_converted.GetNDArray()
                    if VIEW_IMG:
                        cv2.imshow('gopro',img_raw)
                        cv2.waitKey(1)

                    #assign image to ros structure
                    # img.data = image_numpy.flatten().tolist()
                    img.data = img_raw.flatten().tolist()
                    #send image on topic
                    pub.publish(img)

                    if save_image:
                        # Create a unique filename
                        if device_serial_number:
                            filename = savedir + ('Acquisition-%s-%06.0f.jpg' % (device_serial_number, i))
                        else:  # if serial number is empty
                            filename = savedir + ('Acquisition-%06.0f.jpg' % i)


                        
                        cv2.imwrite(filename,img_raw)

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

if __name__ == '__main__':
    try:
        publishimages()
    except rospy.ROSInterruptException:
        pass
