#!/usr/bin/env python3
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
import os
import PySpin
import sys, subprocess, time
import cv2
from goprocam import GoProCamera
from goprocam import constants

# sys.path.append("goproapi")

save_image = True
username = os.getlogin( )
savedir = '/home/%s/OutputImages/' % username #script cannot create folder, must already exist when run

#seems to find device automatically if connected? Why don't we do this?
serialstring = 'DeviceSerialNumber'
#serialstring = '18285036'

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

    # initalize gopro
    p = subprocess.Popen([sys.executable,
                    "/home/%s/gaia-ws/src/GAIA-drone-control/src/goproapi/gopro_keepalive.py" % username],
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.STDOUT)


    print('waiting 3 seconds to initialize camera')
    time.sleep(3)
    
    gpCam = GoProCamera.GoPro()
    #gpCam.gpControlSet(constants.Stream.BIT_RATE, constants.Stream.BitRate.B2_4Mbps)
    #gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.W480)
    device_serial_number = False

    try:
        result = True
        
        cap = cv2.VideoCapture("udp://127.0.0.1:10000") # stream from gopro wifi
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # setting buffer size to 1, only latest frame kept
        
        # Retrieve singleton reference to system object
        # system = PySpin.System.GetInstance()

        # # Get current library version
        # version = system.GetLibraryVersion()
        # print('Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

        # # Retrieve list of cameras from the system
        # cam_list = system.GetCameras()

        # num_cameras = cam_list.GetSize()

        # print('Number of cameras detected: %d' % num_cameras)

        # # Finish if there are no cameras
        # if num_cameras == 0:

        #     # Clear camera list before releasing system
        #     cam_list.Clear()

        #     # Release system instance
        #     system.ReleaseInstance()

        #     print('Not enough cameras!')
        #     input('Done! Press Enter to exit...')
        #     return False

        # # Run example on each camera
        # cam = None
        # for i, x in enumerate(cam_list):
        #     cam = x

        # # Retrieve TL device nodemap and print device information
        # nodemap_tldevice = cam.GetTLDeviceNodeMap()

        # # Initialize camera
        # cam.Init()

        # # Retrieve GenICam nodemap
        # nodemap = cam.GetNodeMap()

        # # Set acquisition mode to continuous
        # #
        # #  *** NOTES ***
        # #  Because the example acquires and saves 10 images, setting acquisition
        # #  mode to continuous lets the example finish. If set to single frame
        # #  or multiframe (at a lower number of images), the example would just
        # #  hang. This would happen because the example has been written to
        # #  acquire 10 images while the camera would have been programmed to
        # #  retrieve less than that.
        # #
        # #  Setting the value of an enumeration node is slightly more complicated
        # #  than other node types. Two nodes must be retrieved: first, the
        # #  enumeration node is retrieved from the nodemap; and second, the entry
        # #  node is retrieved from the enumeration node. The integer value of the
        # #  entry node is then set as the new value of the enumeration node.
        # #
        # #  Notice that both the enumeration and the entry nodes are checked for
        # #  availability and readability/writability. Enumeration nodes are
        # #  generally readable and writable whereas their entry nodes are only
        # #  ever readable.
        # #
        # #  Retrieve enumeration node from nodemap

        # # In order to access the node entries, they have to be casted to a pointer type (CEnumerationPtr here)
        # node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
        # if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
        #     print('Unable to set acquisition mode to continuous (enum retrieval). Aborting...')
        #     return False

        # # Retrieve entry node from enumeration node
        # node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        # if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(node_acquisition_mode_continuous):
        #     print('Unable to set acquisition mode to continuous (entry retrieval). Aborting...')
        #     return False

        # # Retrieve integer value from entry node
        # acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

        # # Set integer value from entry node as new value of enumeration node
        # node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        # print('Acquisition mode set to continuous...')

        # #  Begin acquiring images
        # #
        # #  *** NOTES ***
        # #  What happens when the camera begins acquiring images depends on the
        # #  acquisition mode. Single frame captures only a single image, multi
        # #  frame catures a set number of images, and continuous captures a
        # #  continuous stream of images. Because the example calls for the
        # #  retrieval of 10 images, continuous mode has been set.
        # #
        # #  *** LATER ***
        # #  Image acquisition must be ended when no more images are needed.
        # cam.BeginAcquisition()

        # print('Acquiring images...')

        # #  Retrieve device serial number for filename
        # #
        # #  *** NOTES ***
        # #  The device serial number is retrieved in order to keep cameras from
        # #  overwriting one another. Grabbing image IDs could also accomplish
        # #  this.
        # device_serial_number = ''
        # node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        # if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
        #     device_serial_number = node_device_serial_number.GetValue()
        #     print('Device serial number retrieved as %s...' % device_serial_number)

        # Retrieve, convert, and save images
        i = 0
        img = Image()
        while not rospy.is_shutdown():
            i += 1
            try:

                #  Retrieve next received image
                #
                #  *** NOTES ***
                #  Capturing an image houses images on the camera buffer. Trying
                #  to capture an image that does not exist will hang the camera.
                #
                #  *** LATER ***
                #  Once an image from the buffer is saved and/or no longer
                #  needed, the image must be released in order to keep the
                #  buffer from filling up.
                # img_raw = cam.GetNextImage(1000)
                
                ret, img_raw = cap.read()
                
                img.header.seq = i
                img.header.stamp = rospy.Time.now()

                #  Ensure image completion
                #
                #  *** NOTES ***
                #  Images can easily be checked for completion. This should be
                #  done whenever a complete image is expected or required.
                #  Further, check image status for a little more insight into
                #  why an image is incomplete.
                # if img_raw.IsIncomplete():
                    # print('Image incomplete with image status %d ...' % img_raw.GetImageStatus())
                if not ret:
                    print('Image capture with opencv unsuccessful')
                else:

                    #  Print image information; height and width recorded in pixels
                    #
                    #  *** NOTES ***
                    #  Images have quite a bit of available metadata including
                    #  things such as CRC, image status, and offset values, to
                    #  name a few.
                    # img.width = img_raw.GetWidth()
                    # img.height = img_raw.GetHeight()
                    # img.step = img.height #valid for 8 bit greyscale
                    img.height,img.width = img_raw.shape[:-1]
                    print('Grabbed Image %d, width = %d, height = %d' % (i, img.width, img.height))

                    #  Convert image to mono 8
                    #
                    #  *** NOTES ***
                    #  Images can be converted between pixel formats by using
                    #  the appropriate enumeration value. Unlike the original
                    #  image, the converted one does not need to be released as
                    #  it does not affect the camera buffer.
                    #
                    #  When converting images, color processing algorithm is an
                    #  optional parameter.
                    # image_converted = img_raw.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)

                    # #get numpy image
                    # image_numpy = image_converted.GetNDArray()

                    # cv2.imshow('image window',image_numpy)
                    # cv2.waitKey(25)

                    #assign image to ros structure
                    # img.data = image_numpy.flatten().tolist()
                    img.data = img_raw.flatten().tolist()
                    #send image on topic
                    pub.publish(img)

                    if save_image:
                        # Create a unique filename
                        if device_serial_number:
                            filename = savedir + ('Acquisition-%s-%d.jpg' % (device_serial_number, i))
                        else:  # if serial number is empty
                            filename = savedir + ('Acquisition-%d.jpg' % i)

                        #  Save image
                        #
                        #  *** NOTES ***
                        #  The standard practice of the examples is to use device
                        #  serial numbers to keep images of one device from
                        #  overwriting those of another.
                        # image_converted.Save(filename)
                        cv2.imwrite(filename,img_raw)

                    #  Release image
                    #
                    #  *** NOTES ***
                    #  Images retrieved directly from the camera (i.e. non-converted
                    #  images) need to be released in order to keep from filling the
                    #  buffer.
                    # img_raw.Release()
                    print('')

            # except PySpin.SpinnakerException as ex:
            #     print('Error: %s' % ex)
            #     return False
            except Exception as e:       
                print('Error: %s' % e)

        #  End acquisition
        #
        #  *** NOTES ***
        #  Ending acquisition appropriately helps ensure that devices clean up
        #  properly and do not need to be power-cycled to maintain integrity.
        # cam.EndAcquisition()
        cap.release()

        # Deinitialize camera
        # cam.DeInit()

        # Release reference to camera
        # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
        # cleaned up when going out of scope.
        # The usage of del is preferred to assigning the variable to None.
        # del cam

        # # Clear camera list before releasing system
        # cam_list.Clear()

        # # Release system instance
        # system.ReleaseInstance()
    
    # except PySpin.SpinnakerException as ex:
    #     print('Error: %s' % ex)
    #     return False
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