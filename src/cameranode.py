#!/home/ffil/gaia-feedback-control/gaia-fc-env/bin/python3
# license removed for brevity
from tkinter import image_types
import rospy
from sensor_msgs.msg import Image
import os, datetime
# import PySpin
import sys, subprocess, time
import cv2
from goprocam import GoProCamera
from goprocam import constants
import queue, threading

# sys.path.append("goproapi")

#--------OPTION TO VIEW ACQUISITION IN REAL_TIME-------------#
VIEW_IMG=False
#-----------------------------------------------------#

PI_CAM = False # just for testing when I didn't have gopro with me...

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


    if not PI_CAM:
        gpCam = GoProCamera.GoPro()
        #gpCam.gpControlSet(constants.Stream.BIT_RATE, constants.Stream.BitRate.B2_4Mbps)
        #gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.W480)
    device_serial_number = False

    try:
        result = True
        
        # cap = cv2.VideoCapture("udp://127.0.0.1:10000") # stream from gopro wifi
        # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # setting buffer size to 1, only latest frame kept
        if PI_CAM:
            cmd = gstreamer_pipeline()
            print(cmd)
            cap = cv2.VideoCapture(cmd, cv2.CAP_GSTREAMER)
        else:
            cap = VideoCapture("udp://127.0.0.1:10000") # stream from gopro wifi
        


        # Retrieve, convert, and save images
        i = 0
        first_image=True
        img = Image()
        while not rospy.is_shutdown():
            i += 1
            try:

                #  Retrieve next received image

                if PI_CAM:
                    _,img_raw = cap.read()
                else:
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

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
    exposure_time=100,
    gain=0.0,
):
    cmd_init = "nvarguscamerasrc sensor-mode=0 sensor-id=%d" % sensor_id
    if gain!=0.0:
         cmd_init = cmd_init + " gainrange='%d %d'" % (gain,gain)
    if exposure_time!=0:
         cmd_init = cmd_init + " exposuretimerange='%d %d'" % (exposure_time*1e3,exposure_time*1e3)
    cmd_init = cmd_init + " !"
    cmd_main = (
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
    return cmd_init+cmd_main

if __name__ == '__main__':
    try:
        publishimages()
    except rospy.ROSInterruptException:
        pass
