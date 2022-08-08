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

#------------OPTION TO TURN OFF OPTICAL FLOW-------#
OPT_FLOW_OFF = False
#---------------------------------------------------#

#--------OPTION TO VIEW FLOW RESULTS IN REAL_TIME-------------#
VIEW_IMG=True # also option to save image of output
SAVE_FLOW = True
#-----------------------------------------------------#

#--------OPTION FOR RAFT OPTICAL FLOW------------#
USE_RAFT=True # also option to save image of output
USE_OUTSIDE_MEDIAN_COMP = False
USE_INPAINTING = True
USE_FILTER_FLOW = False
USE_FILTER_COLOR = True
USE_HOMOGRAPHY = False
USE_UNCERTAINTY = True
USE_MIN_VECTORS_FILTER = False
if USE_HOMOGRAPHY: USE_OUTSIDE_MEDIAN_COMP=False
#-----------------------------------------------------#

if USE_RAFT:
    DEVICE = 'cuda'
    import sys
    sys.path.append('/home/ffil/gaia-feedback-control/src/GAIA-drone-control/src/RAFTcore')
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

global flowpub,flow,model_raft

skip=5

def loopcallback(data):
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
                flow = opticalflowmain()
                flowpub.publish(flow)
                datalist=[]
            else:
                print('Second image too late, emptying list')
                
                # emptying the list of images to start fresh
                datalist = []
        else:
            print('Second image too early')


def init_flownode():
    """initializing optical flow node
    """
    global flowpub,flow,model_raft
    global datalist,good_first_image
    print('initializing optical flow node')
    rospy.init_node('opticalflownode', anonymous=False)
    flowpub = rospy.Publisher('/gaia/flow',BoundingBox2D,queue_size=1)
    flow = BoundingBox2D() # using this becaue not sure other ros message formats to use
    

    if USE_RAFT:        
        parser = argparse.ArgumentParser()
        parser.add_argument('--model', help="restore checkpoint")
        parser.add_argument('--path', help="dataset for evaluation")
        parser.add_argument('--small', action='store_true', help='use small model')
        parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
        parser.add_argument('--alternate_corr', action='store_true', help='use efficent correlation implementation')
        args = parser.parse_args(['--model','/home/ffil/gaia-feedback-control/src/GAIA-drone-control/src/RAFTcore/raft-small.pth',
                                    '--path','',
                                    '--small'])
        # args.model = '/home/ffil/gaia-ws/src/GAIA-drone-control/src/RAFTcore/raft-small.pth',
        # args.path = ''
        # args.small = True
        # args.mixed_precision = False
        # args.alternate_corr = False
        
        print('loading RAFT model')
        model_raft = flow_init(args)

    
    # initialize list of images with bounding boxes
    datalist = []
    # each time box is computed, append to list, and if there are enough images, run optical flow
    
    #----------------uncomment to turn ON optical flow------------#
    if not OPT_FLOW_OFF:
        rospy.Subscriber('/gaia/bounding_box', Detection2D, loopcallback)
        rospy.spin()
    #-----------------------------------------------#




def opticalflowmain():
    """callback functio to run once the two images and bounding boxes have been receieved

    Args:
        img1 (_type_): _description_
        img2 (_type_): _description_
        boundingbox (_type_): _description_
    """
    global datalist,flowpub,flow
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
    flow_x,flow_y = opticalflowfunction(img1,img2,boundingbox_pixels,savenum=datalist[0].source_img.header.seq)

    # normalized displacements by the size of frame
    flow.size_x = flow_x/img1.shape[1]
    flow.size_y = flow_y/img1.shape[0]
    
    
    
    
    
    return flow

def opticalflowfunction(img1,img2,boundingbox,savenum):
    """running actual optical flow analysis on two images with a delay between them, using
    bounding box information

    Args:
        img1 (_type_): _description_
        img2 (_type_): _description_
        boundingbox (_type_): _description_

    Returns:
        _type_: _description_
    """
    if USE_RAFT:
        global model_raft
    print('computing optical flow')
    t1 = time.time()
    color_filter_thresh = 180
    median_skipping = 1

    # extraacting bounding box indices
    y1,y2,x1,x2 = boundingbox
    # computing flow outside the bounding box
    time_init = time.time()
    # if USE_RAFT:
    #     flow_outside,_,_ = RAFTflow(prev,curr)
    #     with torch.no_grad():
    #         img1 = torch.from_numpy(img1).permute(2, 0, 1).float()
    #         img2 = torch.from_numpy(img2).permute(2, 0, 1).float()
    #         img1 = img1[None].to(DEVICE)
    #         img2 = img2[None].to(DEVICE)
            
    #         padder = InputPadder(img1.shape)
    #         img1, img2 = padder.pad(img1, img2)
            
    #         _,flow_outside = model(img1,img2,iters=20,test_mode=True)
    #         flow_outside = flow_outside[0].permute(1,2,0).cpu().numpy()
    if USE_RAFT:
        flow_outside,_,_ = RAFTflow(img1.copy(),img2.copy())
    else:
        
        flow_outside = cv.optflow.calcOpticalFlowDenseRLOF(img1,img2,None) # using defaults for now
    print('Took %f seconds' % (time.time() - time_init))
    flow_inside = flow_outside[y1:y2,x1:x2,:].copy()
    flow_outside[y1:y2,x1:x2,:] = np.nan
    # flow_outside_x = np.nanmedian(flow_outside[:,:,0].flatten()[::median_skipping])
    # flow_outside_y = np.nanmedian(flow_outside[:,:,1].flatten()[::median_skipping])

    # # compensating for external motion using flow outside bounding box
    # flow_inside[:,:,0] -= flow_outside_x
    # flow_inside[:,:,1] -= flow_outside_y

    # # filter out low displacements
    # flow_inside[np.abs(flow_inside) < 2] = np.nan

    # # filter out if not very white (questionable approach, not likely to be generalizable but ok for now)
    # color_filter = np.mean(img1[y1:y2,x1:x2,:],axis=2) < color_filter_thresh
    # flow_inside[color_filter] = np.nan

    if USE_INPAINTING:

        tmp = time.time()

        u1 = flow_outside[:,:,0].copy()
        u2 = flow_outside[:,:,1].copy()
        rescale = 4
        u1 = cv.resize(u1,[u1.shape[1]//rescale,u1.shape[0]//rescale])
        u2 = cv.resize(u2,[u2.shape[1]//rescale,u2.shape[0]//rescale])


        u1 = cv.inpaint(u1,(1*np.isnan(u1)).astype(np.uint8),inpaintRadius=10,flags=cv.INPAINT_TELEA)
        u2 = cv.inpaint(u2,(1*np.isnan(u2)).astype(np.uint8),inpaintRadius=10,flags=cv.INPAINT_TELEA)
        
        u1 = cv.resize(u1,[img1.shape[1],img1.shape[0]])
        u2 = cv.resize(u2,[img1.shape[1],img1.shape[0]])


        print('Inpainting time: %f' % (time.time()-tmp))


        flow_inside[:,:,0] -=u1[y1:y2,x1:x2]
        flow_inside[:,:,1] -=u2[y1:y2,x1:x2]
        flow_outside_x = np.nanmean(flow_inside[:,:,0].flatten())
        flow_outside_y = np.nanmean(flow_inside[:,:,1].flatten())

        
    else:
        flow_outside_x = np.nanmedian(flow_outside[:,:,0].flatten()[::1])
        flow_outside_y = np.nanmedian(flow_outside[:,:,1].flatten()[::1])
    
    if USE_OUTSIDE_MEDIAN_COMP:
        # compensating for external motion using flow_inside outside bounding box
        flow_inside[:,:,0] -= flow_outside_x
        flow_inside[:,:,1] -= flow_outside_y

    

    if USE_FILTER_FLOW:
        # filter out low displacements
        flow_inside[np.abs(flow_inside) < 5] = np.nan

    if USE_FILTER_COLOR:
        tmp = time.time()
        # filter out if not very white
        # if USE_RAFT:
        #     color_filter = np.mean(prev_small,axis=2) < 180
        # else:
        color_filter = np.mean(img1[y1:y2,x1:x2,:],axis=2) < 180
        flow_inside[color_filter] = np.nan
        print('COlor filter time: %f' % (time.time()-tmp))
    
    if USE_UNCERTAINTY:
        tmp = time.time()
        sigma_inside = np.array([np.nanstd(flow_inside[:,:,0].flatten()),np.nanstd(flow_inside[:,:,1].flatten())])
        mean_inside = np.abs(np.array([np.nanmean(flow_inside[:,:,0].flatten()),np.nanmean(flow_inside[:,:,1].flatten())]))
        # print('Sigma: (%0.3f,%0.3f); Mean: (%0.3f,%0.3f)' % (sigma_inside[0],sigma_inside[1],mean_inside[0],mean_inside[1]))
        if all(sigma_inside > 2*mean_inside):
            flow_inside = np.full_like(flow_inside,np.nan)
            # print('not good')
        print('Uncertainty time: %f' % (time.time()-tmp))
    
        # plot_flow(img1.copy(),flow_inside,flow_outside,[x1,x2,y1,y2],plot_outside_arrow=False)
    if USE_MIN_VECTORS_FILTER:
        tmp = time.time()
        count = np.sum(~np.isnan(flow_inside.flatten()))/2
        # print('Count: %d' % int(count))
        BLACK = (265,265,265)
        font = cv.FONT_HERSHEY_SIMPLEX
        font_size = 1
        font_color = BLACK
        font_thickness = 2
        img1 = cv.putText(img1,'Vectors: %d' % count,(10,img1.shape[0]-30),font, font_size, font_color, font_thickness, cv.LINE_AA)
        if count < 1e4:
            flow_inside = np.full_like(flow_inside,np.nan)
        print('Vector count time: %f' % (time.time()-tmp))

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
        print('Optical flow plotting time %f' % (t2-t1))


        
    if SAVE_FLOW:
        t1 = time.time()
        savename = savedir+'OpticalFlow-%06.0f.jpg' % savenum
        cv.imwrite(savename,tmp)
        np.savez((savedir+'OpticalFlow-%06.0f' % savenum),flow_outside_x,flow_outside_y,flow_inside,boxflow_x,boxflow_y)
        t2 = time.time()
        print('Optical flow saving time %f' % (t2-t1))
        # if VIEW_IMG:

    
    
    return np.float64(boxflow_x),np.float64(boxflow_y)

def RAFTflow(img1,img2):
    global model_raft
    with torch.no_grad():

        img1 = torch.from_numpy(img1).permute(2, 0, 1).float()
        img2 = torch.from_numpy(img2).permute(2, 0, 1).float()
        img1 = img1[None].to(DEVICE)
        img2 = img2[None].to(DEVICE)

        padder = InputPadder(img1.shape)
        img1, img2 = padder.pad(img1, img2)

        _,flow = model_raft(img1,img2,iters=20,test_mode=True)
        flow = flow[0].permute(1,2,0).cpu().numpy()
        img1 = img1[0].permute(1,2,0).cpu().numpy()
        img2 = img2[0].permute(1,2,0).cpu().numpy()
    return flow,img1,img2
   
def flow_init(args):
    model_raft = torch.nn.DataParallel(RAFT(args))
    model_raft.load_state_dict(torch.load(args.model))

    model_raft = model_raft.module
    model_raft.to(DEVICE)
    model_raft.eval()
    return model_raft




    # take the median values of what is left as the bulk smoke flow direction
    # result = np.array([np.nanmedian(flow_inside[:,:,0].flatten()),np.nanmedian(flow_inside[:,:,1].flatten())])

    



if __name__ == '__main__':
    try:
        init_flownode()
    except rospy.ROSInterruptException:
        pass
