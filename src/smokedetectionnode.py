#!/usr/bin/env python3
# license removed for brevity
from re import sub
import rospy
from rospy.client import init_node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
import numpy as np
import cv2
import os
import PySpin
import sys

global pub,box

def imagecallback(img):
    global box
    box = BoundingBox2D()
    print("Reading from buffer\n")
    img_numpy = np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.width,-1)
    print("Image read\n\n")
    #image test code
    # cv2.imshow('image window',img_numpy)
    # cv2.waitKey(0)
    
    #TODO: Run network, set bounding box parameters

    pub.publish(box)

def init_detection_node():
    global pub,box
    pub = rospy.Publisher('/gaia/bounding_box', BoundingBox2D, queue_size=3)
    box = BoundingBox2D()

    rospy.Subscriber('/camera/image', Image, imagecallback)
    rospy.init_node('detectionnode', anonymous=False)

    rospy.spin()


if __name__ == '__main__':
    try:
        init_detection_node()
    except rospy.ROSInterruptException:
        pass