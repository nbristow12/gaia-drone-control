#!/usr/bin/env python3
# license removed for brevity
import cv2 as cv
import numpy as np
from pathlib import Path
import rospy
from rospy.client import init_node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D,Detection2D


global pub,flow

skip=5

def imageboxcallback(datalist):
    """callback functio to run once the two images and bounding boxes have been receieved

    Args:
        img1 (_type_): _description_
        img2 (_type_): _description_
        boundingbox (_type_): _description_
    """
    global pub,flow
    flow = BoundingBox2D()

    if datalist[0].bbox.size_x: # i.e., if bounding box exists for this image
        # get image 1
        img1 = datalist[0].img_source
        # get image 2 after some skip (end of list)
        img2 = datalist[-1].img_source
        # get bounding box for image 1
        bb = [datalist[0].bbox.center.x, datalist[0].bbox.center.y, datalist[0].bbox.size_x, datalist[0].bbox.size_y]
        # bounding box indices
        xx,w = (bb[[1,3]]*img1.shape[1]).astype(np.uint32)
        yy,h = (bb[[2,4]]*img1.shape[0]).astype(np.uint32)
        boundingbox_pixels = [yy-h//2, yy+h//2, xx-w//2, xx+w//2]
        # run optical flow analysis to get single vector
        flow_x,flow_y = opticalflow(img1,img2,boundingbox_pixels)

        # normalized displacements by the size of frame
        flow.size_x = flow_x/img1.shape[1]
        flow.size_y = flow_y/img1.shape[0]
        pub.publish(flow)

def loopcallback(data):
    """somewhat hacky approach. create a list of subscribed images and their bounding boxes
       keep appending till specified "skip" is reached
       once reached, send to optical flow callback
       if too many in list, throw out oldest

    Args:
        data (_type_): _description_
    """
    global datalist
    datalist.append(data)
    if len(data) == skip:
        print('Obtained enough images for optical flow test')
        imageboxcallback(datalist)
    elif len(data) > skip:
        datalist = datalist[-skip:] # throw out the oldest
        imageboxcallback(datalist) # run again
    else:
        print('Not enough images yet for optical flow')

def init_flownode():
    """initializing optical flow node
    """
    global pub,flow
    global datalist
    
    rospy.init_node('flownode', anonymous=False)
    pub = rospy.Publisher('/gaia/flow',BoundingBox2D,queue_size=1)
    flow = BoundingBox2D() # using this becaue not sure other ros message formats to use
    
    # initialize list of images with bounding boxes
    datalist = []
    # each time box is computed, append to list, and if there are enough images, run optical flow
    rospy.Subscriber('/gaia/boundingbox', Detection2D, loopcallback)

    rospy.spin()
    
    return


def opticalflow(img1,img2,boundingbox):
    """running actual optical flow analysis on two images with a delay between them, using
    bounding box information

    Args:
        img1 (_type_): _description_
        img2 (_type_): _description_
        boundingbox (_type_): _description_

    Returns:
        _type_: _description_
    """

    color_filter_thresh = 180
    median_skipping = 1

    # extraacting bounding box indices
    y1,y2,x1,x2 = boundingbox
    # computing flow outside the bounding box
    flow_outside = cv.optflow.calcOpticalFlowDenseRLOF(img1,img2,None) # using defaults for now
    flow_inside = flow_outside[y1:y2,x1:x2,:].copy()
    flow_outside[y1:y2,x1:x2,:] = np.nan
    flow_outside_x = np.nanmedian(flow_outside[:,:,0].flatten()[::median_skipping])
    flow_outside_y = np.nanmedian(flow_outside[:,:,1].flatten()[::median_skipping])

    # compensating for external motion using flow outside bounding box
    flow_inside[:,:,0] -= flow_outside_x
    flow_inside[:,:,1] -= flow_outside_y

    # filter out low displacements
    flow_inside[np.abs(flow_inside) < 2] = np.nan

    # filter out if not very white (questionable approach, not likely to be generalizable but ok for now)
    color_filter = np.mean(img1[y1:y2,x1:x2,:],axis=2) < color_filter_thresh
    flow_inside[color_filter] = np.nan

    # take the median values of what is left as the bulk smoke flow direction
    result = np.array([np.nanmedian(flow_inside[:,:,0].flatten()),np.nanmedian(flow_inside[:,:,1].flatten())])

    return result.astype(np.float64)



if __name__ == '__main__':
    try:
        init_flownode()
    except rospy.ROSInterruptException:
        pass
