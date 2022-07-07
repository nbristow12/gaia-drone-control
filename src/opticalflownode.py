#!/home/ffil/gaia-feedback-control/gaia-fc-env/bin/python3
# # license removed for brevity
import cv2 as cv
import numpy as np
from pathlib import Path
import rospy
from rospy.client import init_node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D,Detection2D
import os, datetime, time

#--------OPTION TO VIEW FLOW RESULTS IN REAL_TIME-------------#
VIEW_IMG=False # also option to save image of output
SAVE_FLOW = True
#-----------------------------------------------------#

#--------OPTION FOR RAFT OPTICAL FLOW------------#
RAFT=False # also option to save image of output
#-----------------------------------------------------#

if RAFT:
    DEVICE = 'cuda'
    import sys
    sys.path.append('/home/ffil/gaia-ws/src/GAIA-drone-control/src/RAFTcore')
    from raft import RAFT
    from utils import flow_viz
    from utils.utils import InputPadder
    import torch
    import argparse


username = os.getlogin( )
tmp = datetime.datetime.now()
stamp = ("%02d-%02d-%02d__%02d-%02d-%02d" % 
    (tmp.year, tmp.month, tmp.day, 
    tmp.hour, tmp.minute, tmp.second))
savedir = '/home/%s/1FeedbackControl/FeedbackControl_%s/flow/' % (username,stamp) 
os.makedirs(savedir)

global flowpub,flow

skip=5

# def imageboxcallback(datalist):
#     """callback functio to run once the two images and bounding boxes have been receieved

#     Args:
#         img1 (_type_): _description_
#         img2 (_type_): _description_
#         boundingbox (_type_): _description_
#     """
#     global flowpub,flow,good_first_image
#     flow = BoundingBox2D()


    
#     if datalist[0].bbox.size_x != -1: # i.e., if bounding box exists for this image
#         print('got good bounding box, starting flow analysis')
#         # get image 1
#         tmp_img = datalist[0].source_img
#         img1 = np.frombuffer(tmp_img.data,dtype=np.uint8).reshape(tmp_img.height,tmp_img.width,-1)
#         # get image 2 after some skip (end of list)
#         tmp_img = datalist[-1].source_img
#         img2 = np.frombuffer(tmp_img.data,dtype=np.uint8).reshape(tmp_img.height,tmp_img.width,-1)
        
        
#         # img2 = datalist[-1].source_img
#         # get bounding box for image 1
#         bb = [datalist[0].bbox.center.x, datalist[0].bbox.center.y, datalist[0].bbox.size_x, datalist[0].bbox.size_y]
#         # bounding box indices
#         xx= int(bb[0]*img1.shape[1])
#         w = int(bb[2]*img1.shape[1])
#         yy = int(bb[1]*img1.shape[0])
#         h = int(bb[3]*img1.shape[0])
#         # yy,h = (bb[[2,4]]*img1.shape[0]).astype(np.uint32)
#         boundingbox_pixels = [yy-h//2, yy+h//2, xx-w//2, xx+w//2]
#         # run optical flow analysis to get single vector
#         savenum = datalist[0].source_img.header.seq
#         flow_x,flow_y,tmp = opticalflow(img1,img2,boundingbox_pixels,savenum)

#         # normalized displacements by the size of frame
#         flow.size_x = flow_x/img1.shape[1]
#         flow.size_y = flow_y/img1.shape[0]
        
        
        
#         flowpub.publish(flow)
#         datalist = [] # clear list after done
#         good_first_image = False
#     else:
#         print('bounding box not found, waiting')

def opticalflowprep(datalist):
    """callback functio to run once the two images and bounding boxes have been receieved

    Args:
        img1 (_type_): _description_
        img2 (_type_): _description_
        boundingbox (_type_): _description_
    """
    global flowpub,flow
    flow = BoundingBox2D()

    # get image 1
    tmp_img = datalist[0].source_img
    img1 = np.frombuffer(tmp_img.data,dtype=np.uint8).reshape(tmp_img.height,tmp_img.width,-1)
    # get image 2 after some skip (end of list)
    tmp_img = datalist[-1].source_img
    img2 = np.frombuffer(tmp_img.data,dtype=np.uint8).reshape(tmp_img.height,tmp_img.width,-1)
    
    # get bounding box for image 1
    bb = [datalist[0].bbox.center.x, datalist[0].bbox.center.y, datalist[0].bbox.size_x, datalist[0].bbox.size_y]
    # bounding box indices
    xx= int(bb[0]*img1.shape[1])
    w = int(bb[2]*img1.shape[1])
    yy = int(bb[1]*img1.shape[0])
    h = int(bb[3]*img1.shape[0])
    
    boundingbox_pixels = [yy-h//2, yy+h//2, xx-w//2, xx+w//2]
    
    # savename = savedir+'OpticalFlow-%06.0f.jpg' % 
    # run optical flow analysis to get single vector
    flow_x,flow_y = opticalflow(img1,img2,boundingbox_pixels,savenum=datalist[0].source_img.header.seq)

    # normalized displacements by the size of frame
    flow.size_x = flow_x/img1.shape[1]
    flow.size_y = flow_y/img1.shape[0]
    
    
    
    flowpub.publish(flow)
    datalist = [] # clear list after done



def newloopcallback(data):
    global datalist
    if rospy.Time.now() - data.source_img.header.stamp > rospy.Duration(1):
        print("OpticalFlowNode: one of images is too old, dropping\n")
        return
    if len(datalist)==0:
        if data.bbox.center.x == -1:
            # going to wait until a bounding box is found
            return
        else:
            # add image and bounding box as img1
            datalist.append(data)
    else:
        dt = data.source_img.header.stamp - datalist[0].source_img.header.stamp
        if dt > rospy.Duration(0.2):
            if dt < rospy.Duration(1):
            
                # second image with proper delay found, passing to optical flow
                datalist.append(data)
                opticalflowprep(datalist)
            else:
                print('Second image too late, emptying list')
                # print('original: ')
                # print(datalist[0].source_img.header.stamp)
                # print('current')
                # print(data.source_img.header.stamp)
                # print('dt')
                # print(dt)
                # print('rospy threshold')
                # print(rospy.Duration(1))
                
                # emptying the list of images to start fresh
                datalist = []
        else:
            print('Second image too early')
            
            
# def loopcallback(data):
#     """somewhat hacky approach. create a list of subscribed images and their bounding boxes
#        keep appending till specified "skip" is reached
#        once reached, send to optical flow callback
#        if too many in list, throw out oldest

#     Args:
#         data (_type_): _description_
#     """
#     global datalist,good_first_image
#     # print(rospy.Time.now())
#     # print('Time after receiving from detection')
#     # print('Image %d' % data.header.seq)
#     # print(data.header.stamp)
#     if rospy.Time.now() - data.source_img.header.stamp > rospy.Duration(1):
#         print("dropping old image from optical flow\n")
#         datalist = []
#         return
    
#     # print('Image %d' % data.header.seq)
#     # print(data.header.stamp)
    
#     if data.bbox.center.x == -1 and not good_first_image:
#         print('waiting for good first frame to start from')
#         # good_first_image = False
#         return
#     good_first_image = True
#     print('adding bb to list')
#     print('length of list: %d' % len(datalist))
#     datalist.append(data)
#     if len(datalist) == skip:
#         # print('Obtained enough images for optical flow test')
#         imageboxcallback(datalist)
#     elif len(datalist) > skip:
#         # print('Too many images')
#         datalist = datalist[-skip:] # throw out the oldest
#         imageboxcallback(datalist) # run again
#     else:
#         print('Not enough images yet for optical flow')

def init_flownode():
    """initializing optical flow node
    """
    global flowpub,flow
    global datalist,good_first_image
    print('initializing optical flow node')
    rospy.init_node('opticalflownode', anonymous=False)
    flowpub = rospy.Publisher('/gaia/flow',BoundingBox2D,queue_size=1)
    flow = BoundingBox2D() # using this becaue not sure other ros message formats to use
    

    if RAFT:        
        parser = argparse.ArgumentParser()
        parser.add_argument('--model', help="restore checkpoint")
        parser.add_argument('--path', help="dataset for evaluation")
        parser.add_argument('--small', action='store_true', help='use small model')
        parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
        parser.add_argument('--alternate_corr', action='store_true', help='use efficent correlation implementation')
        args = parser.parse_args(['--model','/home/ffil/gaia-ws/src/GAIA-drone-control/src/RAFTcore/raft-small.pth',
                                    '--path','',
                                    '--small'])
        # args.model = '/home/ffil/gaia-ws/src/GAIA-drone-control/src/RAFTcore/raft-small.pth',
        # args.path = ''
        # args.small = True
        # args.mixed_precision = False
        # args.alternate_corr = False
        
        print('loading model')
        global model
        model = torch.nn.DataParallel(RAFT(args))
        model.load_state_dict(torch.load(args.model))

        model = model.module
        model.to(DEVICE)
        model.eval()
    
    # initialize list of images with bounding boxes
    datalist = []
    # each time box is computed, append to list, and if there are enough images, run optical flow
    
    #----------------uncomment to turn ON optical flow------------#
    rospy.Subscriber('/gaia/bounding_box', Detection2D, newloopcallback)
    rospy.spin()
    #-----------------------------------------------#
    return


def opticalflow(img1,img2,boundingbox,savenum):
    """running actual optical flow analysis on two images with a delay between them, using
    bounding box information

    Args:
        img1 (_type_): _description_
        img2 (_type_): _description_
        boundingbox (_type_): _description_

    Returns:
        _type_: _description_
    """

    print('computing optical flow')
    t1 = time.time()
    color_filter_thresh = 180
    median_skipping = 1

    # extraacting bounding box indices
    y1,y2,x1,x2 = boundingbox
    # computing flow outside the bounding box
    if RAFT:
        with torch.no_grad():
            img1 = torch.from_numpy(img1).permute(2, 0, 1).float()
            img2 = torch.from_numpy(img2).permute(2, 0, 1).float()
            img1 = img1[None].to(DEVICE)
            img2 = img2[None].to(DEVICE)
            
            padder = InputPadder(img1.shape)
            img1, img2 = padder.pad(img1, img2)
            
            _,flow_outside = model(img1,img2,iters=20,test_mode=True)
            flow_outside = flow_outside[0].permute(1,2,0).cpu().numpy()
    else:
        time_init = time.time()
        flow_outside = cv.optflow.calcOpticalFlowDenseRLOF(img1,img2,None) # using defaults for now
        print('Took %f seconds' % (time.time() - time_init))
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

    boxflow_x,boxflow_y = np.nanmedian(flow_inside[:,:,0].flatten()),np.nanmedian(flow_inside[:,:,1].flatten())
    t2 = time.time()
    print('Optical flow computation took %f seconds' % (t2-t1))
    
    # drawing plot
    tmp = img1.copy()
        # drawing flow arrows
    step = 5
    for ii in range(0,flow_inside.shape[1],step):
        for jj in range(0,flow_inside.shape[0],step):
            if not any(np.isnan(flow_inside[jj,ii,:])):
                tmp = cv.arrowedLine(
                    tmp,
                    pt1 = np.array([ii,jj]) + np.array([x1,y1]),
                    pt2 = np.array([ii,jj]) + np.array([x1,y1]) + np.array([flow_inside[jj,ii,0],flow_inside[jj,ii,1]]).astype(int),
                    color=[0,255,0],
                    thickness=1,
                    tipLength=0.5
                )
    
    
    if np.isnan(boxflow_x):
        boxflow_x = 0
    if np.isnan(boxflow_y):
        boxflow_y = 0
    # adding bounding box
    tmp = cv.rectangle(tmp,(x1,y1),(x2,y2),color=[0,0,255],thickness=4)



    if boxflow_x != 0 and boxflow_y !=0:# drawing bulk motion arrow in bounding box
        tmp = cv.arrowedLine(
            tmp,
            pt1 = np.array([(x1+x2)//2,(y1+y2)//2],dtype=np.uint32),
            pt2 = np.array([(x1+x2)/2,(y1+y2)/2],dtype=np.uint32) + 10*np.array([boxflow_x,boxflow_y],dtype=np.uint32),
            color=[255,0,0],
            thickness=5,
            tipLength=0.5
        )

    # drawing bulk motion arrow from entire frame
    tmp = cv.arrowedLine(
        tmp,
        pt1 = np.array([img1.shape[1]//2,img1.shape[0]//2],dtype=np.uint32),
        pt2 = np.array([img1.shape[1]//2,img1.shape[0]//2],dtype=np.uint32) + 10*np.array([flow_outside_x,flow_outside_y],dtype=np.uint32),
        color=[255,255,0],
        thickness=5,
        tipLength=0.5
    )

    # plt.imshow(cv.cvtColor(tmp,cv.COLOR_BGR2RGB))
    # plt.show()
    result = tmp

    if VIEW_IMG:
        t1 = time.time()
        cv.imshow('gopro',result)
        cv.waitKey(1)
        t2 = time.time()
        print('Optical flow plotting took %f seconds' % (t2-t1))


        
    if SAVE_FLOW:
        t1 = time.time()
        savename = savedir+'OpticalFlow-%06.0f.jpg' % savenum
        cv.imwrite(savename,tmp)
        np.savez(savedir+'OpticalFlow-%06.0f' % savenum,img1,flow_outside_x,flow_outside_y,flow_inside,boxflow_x,boxflow_y)
        t2 = time.time()
        print('Optical flow saving took %f seconds' % (t2-t1))
        # if VIEW_IMG:

    
    
    return np.float64(boxflow_x),np.float64(boxflow_y)


    # take the median values of what is left as the bulk smoke flow direction
    # result = np.array([np.nanmedian(flow_inside[:,:,0].flatten()),np.nanmedian(flow_inside[:,:,1].flatten())])

    



if __name__ == '__main__':
    try:
        init_flownode()
    except rospy.ROSInterruptException:
        pass
