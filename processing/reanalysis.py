"""this script re-runs both object detection and optical flow on a raw camera video, 
    in teh same way as it was run, except without any skipping,
    and outputs into a single video file
"""

# %% import packages

from ast import increment_lineno
import cv2 as cv
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import sys, os, time
# from cv2.xfeatures2d import matchGMS,SURF_create
# from rasterio.fill import fillnodata
# from scipy import interpolate
import torch
print(f"Torch setup complete. Using torch {torch.__version__} ({torch.cuda.get_device_properties(0).name if torch.cuda.is_available() else 'CPU'})")
if torch.cuda.is_available():
    DEVICE='cuda'
else:
    DEVICE='cpu'
    
FILE = Path(__file__).resolve()
# YOLO paths and importing
YOLOv5_ROOT = FILE.parents[1] / 'src/modules/yolov5'  # YOLOv5 root directory
if str(YOLOv5_ROOT) not in sys.path:
    sys.path.append(str(YOLOv5_ROOT))  # add YOLOv5_ROOT to PATH
YOLOv5_ROOT = Path(os.path.relpath(YOLOv5_ROOT, Path.cwd()))  # relative
from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device
from utils.plots import Annotator, colors
from utils.augmentations import letterbox

#--------OPTION FOR RAFT OPTICAL FLOW------------#
USE_RAFT=True # also option to save image of output
USE_COMP = False
USE_OUTSIDE_MEDIAN_COMP = False
USE_INPAINTING = True
USE_FILTER_FLOW = False
USE_FILTER_COLOR = True
USE_HOMOGRAPHY = False
USE_UNCERTAINTY = False
USE_MIN_VECTORS_FILTER = False
#-----------------------------------------------------#
if USE_HOMOGRAPHY: USE_OUTSIDE_MEDIAN_COMP=False
SAVE_VIDEO = True



if USE_RAFT:
    RAFT_ROOT = Path(FILE.parents[1] / 'src/modules/RAFT')  # RAFT directory
    print(RAFT_ROOT)
    if str(RAFT_ROOT) not in sys.path:
        sys.path.append(str(RAFT_ROOT))  # add RAFT to PATH
    from raft import RAFT
    from utils import flow_viz
    from utils.utils import InputPadder
    import torch
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help="restore checkpoint")
    parser.add_argument('--path', help="dataset for evaluation")
    parser.add_argument('--small', action='store_true', help='use small model')
    parser.add_argument('--mixed_precision', action='store_true', help='use mixed precision')
    parser.add_argument('--alternate_corr', action='store_true', help='use efficent correlation implementation')
    args = parser.parse_args(['--model',str(RAFT_ROOT) + '/raft-small.pth',
                                '--path','',
                                '--small'])
    
    print('loading model')
    # global model
    model_raft = torch.nn.DataParallel(RAFT(args))
    if DEVICE=='cuda':
        model_raft.load_state_dict(torch.load(args.model))
    else:
        model_raft.load_state_dict(torch.load(args.model,map_location=torch.device('cpu')))

    model_raft = model_raft.module
    model_raft.to(DEVICE)
    model_raft.eval()

folder = Path(r'/Users/nate/UMN/FeedbackControl_2022-08-26_goodRuns/FeedbackControl_2022-08-25__11-07-16/camera')
filename = 'Acquisition.avi'
#%%
def main():

    # read video file
    # folder = Path(r'D:\GAIA\SmokePlumeDetection\1Experiments\2022-05-06_UMore\OutputImages_2022-05-06__11-28-55')

    # folder = Path(r'D:')
    file = folder.joinpath(filename)
    cap = cv.VideoCapture(str(file))

    videoFps = cap.get(cv.CAP_PROP_FPS)

    first_frame = 520
    # first_frame = 2690
    # first_frame = 5600
    # last_possible_frame = int(cap.get(cv.CAP_PROP_FRAME_COUNT)) # this doesn't work if the video wasn't saved properly, as is typical for rospya
    last_possible_frame = 2000
    skip = 5
    cap.set(cv.CAP_PROP_POS_FRAMES , first_frame)


    frame_id = first_frame
    # loop and read video frames




    print('Initializing model')
    # weights=YOLOv5_ROOT / 'smoke01k_015empty_H-M-L_withUMore.pt'
    weights=YOLOv5_ROOT / 'smoke.pt'
    model_yolo, device, names = detect_init(weights)
    imgsz = [448,448] # scaled image size to run inference on
    model_yolo(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model_yolo.parameters())))  # run once

    # %%

    while True:
        # %%
        # reading current frame
        # frame_id=2000
        cap.set(cv.CAP_PROP_POS_FRAMES , frame_id)
        ret,prev = cap.read()
        t1 = time.time()
        smoke,img_with_boxes = detect_smoke(prev,imgsz,model_yolo,device,names,view_img=True)
        print('Inference time: %f' % (time.time()-t1))
        if smoke:



            # # bounding box indices
            xx,w = (smoke[0].bounding_box[[0,2]]*prev.shape[1]).astype(np.uint32)
            yy,h = (smoke[0].bounding_box[[1,3]]*prev.shape[0]).astype(np.uint32)

            y1,y2,x1,x2 = yy-h//2, yy+h//2, xx-w//2, xx+w//2
            # y1,y2,x1,x2 = (0,prev.shape[0],0,prev.shape[1])

            # Read next frame after skip
            cap.set(cv.CAP_PROP_POS_FRAMES , frame_id+skip)
            success, curr = cap.read()
            # if not success:
            #     break

            if USE_HOMOGRAPHY:
                mask = np.ones((prev.shape[0],prev.shape[1]),dtype=np.uint8)
                mask[y1:y2,x1:x2] = 0
                prev = motionHomography(prev,curr,mask)


            # using dense optical flow with RLOF
            # flow = cv.optflow.calcOpticalFlowDenseRLOF(prev[y1:y2,x1:x2,:],curr[y1:y2,x1:x2,:],None) # using defaults for now

            # computing flow outside the bounding box
            t1 = time.time()
            if USE_RAFT:
                flow_outside,_,_ = RAFTflow(prev,curr,model_raft)
            else:
                flow_outside = cv.optflow.calcOpticalFlowDenseRLOF(prev,curr,None) # using defaults for now
            print('Flow time: %f sec' % (time.time()-t1))


            flow = flow_outside[y1:y2,x1:x2,:].copy()
            flow_outside[y1:y2,x1:x2,:] = np.nan
            

            if USE_COMP:
                if USE_INPAINTING:
                    tmp = time.time()

                    u1 = flow_outside[:,:,0].copy()
                    u2 = flow_outside[:,:,1].copy()
                    rescale = 4
                    u1 = cv.resize(u1,[u1.shape[1]//rescale,u1.shape[0]//rescale])
                    u2 = cv.resize(u2,[u2.shape[1]//rescale,u2.shape[0]//rescale])



                    # u1 = fillnodata(u1,~np.isnan(u1),max_search_distance=500)
                    # u2 = fillnodata(u2,~np.isnan(u2),max_search_distance=500)
                    u1 = cv.inpaint(u1,(1*np.isnan(u1)).astype(np.uint8),inpaintRadius=10,flags=cv.INPAINT_TELEA)
                    u2 = cv.inpaint(u2,(1*np.isnan(u2)).astype(np.uint8),inpaintRadius=10,flags=cv.INPAINT_TELEA)

                    
                    u1 = cv.resize(u1,[prev.shape[1],prev.shape[0]])
                    u2 = cv.resize(u2,[prev.shape[1],prev.shape[0]])

                    # img = flow_outside[:,:,0]
                    # valid_mask = ~np.isnan(img)
                    # coords = np.array(np.nonzero(valid_mask)).T
                    # values = img[valid_mask]

                    # it = interpolate.LinearNDInterpolator(coords, values, fill_value=0)
                    print('Inpainting time: %f' % (time.time()-tmp))
                    # filled = it(list(np.ndindex(img.shape))).reshape(img.shape)
                    # u1 = filled.copy()
                    # u2 = u1.copy()
                    
                    flow[:,:,0] -=u1[y1:y2,x1:x2]
                    flow[:,:,1] -=u2[y1:y2,x1:x2]
                    flow_outside_x = np.nanmean(flow[:,:,0].flatten())
                    flow_outside_y = np.nanmean(flow[:,:,1].flatten())
                    # %matplotlib inline
                    # u1 = cv.rectangle(u1,(x1,y1),(x2,y2),color=[0,0,255],thickness=4)
                    # u2 = cv.rectangle(u2,(x1,y1),(x2,y2),color=[0,0,255],thickness=4)
                    # plt.imshow(np.concatenate((u1,u2),axis=1))
                    # plt.show()

                else:
                    flow_outside_x = np.nanmedian(flow_outside[:,:,0].flatten()[::1])
                    flow_outside_y = np.nanmedian(flow_outside[:,:,1].flatten()[::1])
                
                if USE_OUTSIDE_MEDIAN_COMP:
                    # compensating for external motion using flow outside bounding box
                    flow[:,:,0] -= flow_outside_x
                    flow[:,:,1] -= flow_outside_y
            else:
                flow_outside_x=flow_outside_y=0
            

            if USE_FILTER_FLOW:
                # filter out low displacements
                flow[np.abs(flow) < 5] = np.nan

            if USE_FILTER_COLOR:
                tmp = time.time()
                # filter out if not very white
                # if USE_RAFT:
                #     color_filter = np.mean(prev_small,axis=2) < 180
                # else:
                color_filter = np.mean(prev[y1:y2,x1:x2,:],axis=2) < 180
                flow[color_filter] = np.nan
                print('COlor filter time: %f' % (time.time()-tmp))
            
            if USE_UNCERTAINTY:
                tmp = time.time()
                sigma_inside = np.array([np.nanstd(flow[:,:,0].flatten()),np.nanstd(flow[:,:,1].flatten())])
                mean_inside = np.abs(np.array([np.nanmean(flow[:,:,0].flatten()),np.nanmean(flow[:,:,1].flatten())]))
                # print('Sigma: (%0.3f,%0.3f); Mean: (%0.3f,%0.3f)' % (sigma_inside[0],sigma_inside[1],mean_inside[0],mean_inside[1]))
                # if all(sigma_inside > 2*mean_inside):
                    # flow = np.full_like(flow,np.nan)
                    # print('not good')
                print('Uncertainty time: %f' % (time.time()-tmp))
            
                # plot_flow(prev.copy(),flow,flow_outside,[x1,x2,y1,y2],plot_outside_arrow=False)
            if USE_MIN_VECTORS_FILTER:
                tmp = time.time()
                count = np.sum(~np.isnan(flow.flatten()))/2
                # print('Count: %d' % int(count))
                BLACK = (265,265,265)
                font = cv.FONT_HERSHEY_SIMPLEX
                font_size = 1
                font_color = BLACK
                font_thickness = 2
                prev = cv.putText(prev,'Vectors: %d' % count,(10,prev.shape[0]-30),font, font_size, font_color, font_thickness, cv.LINE_AA)
                if count < 1e4:
                    flow = np.full_like(flow,np.nan)
                print('Vector count time: %f' % (time.time()-tmp))
            
            result = plot_flow(prev.copy(),flow,flow_outside,[x1,x2,y1,y2],plot_outside_arrow=USE_COMP,plot_outside_detail = False,show_figure=True)

            # %%
            # fig,ax = plt.subplots(1,2)
            # ax[0].hist(flow[:,:,0].flatten())
            # ax[1].hist(flow[:,:,1].flatten())
            # plt.show()
        # except:
        else:
            result = prev           
        BLACK = (265,265,265)
        font = cv.FONT_HERSHEY_SIMPLEX
        font_size = 1
        font_color = BLACK
        font_thickness = 2
        # result = cv.putText(result,'frame %d' % frame_id,(10,30),font, font_size, font_color, font_thickness, cv.LINE_AA)


 

        if SAVE_VIDEO:
            if frame_id == first_frame:
                if USE_RAFT:
                    savename = folder.joinpath('opticalflow_RAFT.avi')
                else:
                    savename = folder.joinpath('opticalflow.avi')
                if savename.suffix.lower()=='.mp4':
                    # codec = cv.VideoWriter_fourcc(*'XVID')
                    codec = 0x7634706d
                elif savename.suffix.lower()=='.avi':
                    codec = cv.VideoWriter_fourcc('M','J','P','G')
                video = cv.VideoWriter(filename=str(savename),
                    fourcc=codec, 
                    fps=videoFps, 
                    frameSize=(result.shape[1],result.shape[0]))
            video.write(result)

        cv.imshow('frame',result)
        key = cv.waitKey(1)
        if key == 27:   # if esc key pressed
            print('\nESC was pressed, ending acquisition')
            cv.destroyAllWindows()
            if SAVE_VIDEO:
                video.release()
            # connection.close()
            # server_socket.close()
            break

        frame_id+=1
        if frame_id >= last_possible_frame:
            cv.destroyAllWindows()
            if SAVE_VIDEO:
                video.release()
            break


        print('full time %f' % (time.time()-t1))




# functions

## methods from yolov5_smoke/detect_fun.py
def detect_init(weights=YOLOv5_ROOT / 'yolov5s.pt'):
    
    device = select_device(device='',batch_size=None)   # usually cpu or cuda
    w = str(weights[0] if isinstance(weights, list) else weights) # model weights, from .pt file
    model = torch.jit.load(w) if 'torchscript' in w else attempt_load(weights, map_location=device) # initializing model in torch
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    
    return model, device, names

def detect_smoke(img0,imgsz,model,device,names,view_img = False):
    img0 = img0.copy()
    # weights=YOLOv5_ROOT / 'yolov5s.pt'  # model.pt path(s)
    # source=YOLOv5_ROOT / 'data/images'  # file/dir/URL/glob, 0 for webcam
    # imgsz=640  # inference size (pixels)
    conf_thres=0.25  # confidence threshold
    iou_thres=0.45  # NMS IOU threshold
    max_det=1000  # maximum detections per image
    # device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
    # view_img=False  # show results
    # save_txt=False  # save results to *.txt
    # save_conf=False  # save confidences in --save-txt labels
    # save_crop=False  # save cropped prediction boxes
    # nosave=False  # do not save images/videos
    classes=None  # filter by class: --class 0, or --class 0 2 3
    agnostic_nms=False  # class-agnostic NMS
    augment=False  # augmented inference
    visualize=False  # visualize features
    update=False  # update all models
    # project=YOLOv5_ROOT / 'runs/detect'  # save results to project/name
    # name='exp'  # save results to project/name
    # exist_ok=False  # existing project/name ok, do not increment
    # line_thickness=3  # bounding box thickness (pixels)
    # hide_labels=False  # hide labels
    # hide_conf=False  # hide confidences
    half=False  # use FP16 half-precision inference
    # dnn=False  # use OpenCV DNN for ONNX inference
    stride = 64
    
    # Padded resize
    img = letterbox(img0, stride=stride, auto=True)[0]
    # Convert
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.array((img,img,img))
    img = np.ascontiguousarray(img)
    # imgsz = img.shape
    seen = 0
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img = img / 255.0  # 0 - 255 to 0.0 - 1.0
    if len(img.shape) == 3:
        img = img[None]  # expand for batch dim
    
    pred = model(img, augment=augment, visualize=visualize)[0]
    #NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
    obj = [] # initializing output list
    # Process predictions
    for i, det in enumerate(pred):  # per image
        seen += 1
        # im0 = img0.copy()
        # p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)
        gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        
        annotator = Annotator(img0, line_width=1, example=str(names))
        
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
            # Write results
            for *xyxy, conf, cls in reversed(det):
                
                # extracting bounding box, confidence level, and name for each object
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                confidence = float(conf)
                object_class = names[int(cls)]
                # if save_img or save_crop or view_img:  # Add bbox to image
                if view_img:
                    c = int(cls)  # integer class
                    label = f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                # adding object to list
                obj.append(DetectedObject(np.array(xywh),confidence,object_class))

        im_with_boxes = annotator.result()
        
        
        # if VIEW_IMG:
        #     # im_with_boxes = annotator.result()
        #     cv2.imshow('gopro', im_with_boxes)
        #     cv2.waitKey(1)  # 1 millisecond


    #------return smoke with max confidence------------#
    bestsmoke = []
    bestconf = 0
    for ob in obj:
        if ob.object_class == 'smoke' and ob.confidence > bestconf:
            bestsmoke = [ob]
            bestconf = ob.confidence  

    return bestsmoke,im_with_boxes

class DetectedObject():
    def __init__(self,xywh=[],conf=0.,cls=''):
        self.bounding_box = xywh # normalized coordinates (by original image dimensions) of horizontal center, vertical center, width, height
        self.confidence=float(conf) # 0 is no confidence, 1 is full
        self.object_class = str(cls) # name of object (e.g., "person", or "smoke")

def RAFTflow(img1,img2,model):
    with torch.no_grad():

        img1 = torch.from_numpy(img1).permute(2, 0, 1).float()
        img2 = torch.from_numpy(img2).permute(2, 0, 1).float()
        img1 = img1[None].to(DEVICE)
        img2 = img2[None].to(DEVICE)

        padder = InputPadder(img1.shape)
        img1, img2 = padder.pad(img1, img2)

        _,flow = model(img1,img2,iters=20,test_mode=True)
        flow = flow[0].permute(1,2,0).cpu().numpy()
        img1 = img1[0].permute(1,2,0).cpu().numpy()
        img2 = img2[0].permute(1,2,0).cpu().numpy()
    return flow,img1,img2

def motionHomography(img1,img2,mask,debugging=False):
    """compensates for the motion of a camera between two image captures, 
        and warps the first frame to match the second using homography

    Args:
        img1 (float array): first image
        img2 (float array): second image

    Returns:
        float array: first image after warping to compensate for motion
    """

    #creating mask
    # mask = np.full_like(img1,fill_value = 1)
    # mask[roi[1]:roi[1]+roi[3],roi[0]:roi[0]+roi[2]] = 0

    # mask = np.ones_like(mask,dtype=np.uint8)
    # finding features
    # detector = cv.ORB_create(20000)
    # detector.setFastThreshold(0)
    detector = cv.SIFT_create(2000)
    kp1, des1 = detector.detectAndCompute(img1,mask)
    kp2, des2 = detector.detectAndCompute(img2,mask)

    # matching features
    # bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=False)
    bf = cv.BFMatcher(cv.NORM_L2, crossCheck=False)
    # Match descriptors.
    matches = bf.match(des1,des2)
    # filter out matches with GMS
    matches = matchGMS(img1.shape[:2], img2.shape[:2], kp1, kp2, matches, withScale=True, withRotation=True, thresholdFactor=10)
    if not matches:
        return img1
    # draw matches
    if debugging:
        img_matches = np.empty((max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1], 3), dtype=np.uint8)
        cv.drawMatches(img1, kp1, img2, kp2, matches, img_matches, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        plt.imshow(img_matches)
        plt.show()

    # removing bad features
    kp1_good = np.empty((len(matches),2), dtype=np.float32)
    kp2_good = np.empty((len(matches),2), dtype=np.float32)
    for i in range(len(matches)):
        #-- Get the keypoints from the good matches
        kp1_good[i,0] = kp1[matches[i].queryIdx].pt[0]
        kp1_good[i,1] = kp1[matches[i].queryIdx].pt[1]
        kp2_good[i,0] = kp2[matches[i].trainIdx].pt[0]
        kp2_good[i,1] = kp2[matches[i].trainIdx].pt[1]

    # computing 3x3 homography matrix
    H, _ = cv.findHomography(kp1_good, kp2_good, cv.RANSAC)

    # warping image 1 to minimize difference iwth iamge 2
    img1_warp = cv.warpPerspective(img1, H, (img1.shape[1], img1.shape[0]))

    return img1_warp

def plot_flow(prev,flow_inside,flow_outside,ROI,plot_outside_arrow = True,plot_outside_detail = False,show_figure=True):
    x1,x2,y1,y2 = ROI
    img = prev.copy()
    tmp2 = prev.copy()
    # drawing flow arrows
    step = 5
    for ii in range(0,flow_inside.shape[1],step):
        for jj in range(0,flow_inside.shape[0],step):
            if not any(np.isnan(flow_inside[jj,ii,:])):
                img = cv.arrowedLine(
                    img,
                    pt1 = np.array([ii,jj]) + np.array([x1,y1]),
                    pt2 = np.array([ii,jj]) + np.array([x1,y1]) + np.array([flow_inside[jj,ii,0],flow_inside[jj,ii,1]]).astype(int),
                    color=[0,255,0],
                    thickness=1,
                    tipLength=0.5
                )
    # adding bounding box
    img = cv.rectangle(img,(x1,y1),(x2,y2),color=[0,0,255],thickness=4)

    # drawing bulk motion arrow in bounding box
    bb_ux,bb_uy = (np.nanmedian(flow_inside[:,:,0].flatten()),np.nanmedian(flow_inside[:,:,1].flatten()))
    if np.isnan(bb_ux) or np.isnan(bb_uy):
        bb_ux = bb_uy = 0
    # print('dx,dy = %0.3f,%0.3f' % (bb_ux,bb_uy))
    pt1 = np.array([(x1+x2)//2,(y1+y2)//2],dtype=np.uint32)
    pt2 = np.array([(x1+x2)/2,(y1+y2)/2],dtype=np.uint32) + 10*np.array([bb_ux,bb_uy],dtype=np.uint32)
    pt2[1] = min(pt2[1],img.shape[0])
    pt2[0] = min(pt2[0],img.shape[1])

    img = cv.arrowedLine(
        img,
        pt1 = pt1,
        pt2 = pt2,
        color=[255,0,0],
        thickness=5,
        tipLength=0.5
    )
    

    # drawing bulk motion arrow from entire frame
    if plot_outside_arrow:
        flow_outside_x = np.nanmedian(flow_outside[:,:,0].flatten()[::1])
        flow_outside_y = np.nanmedian(flow_outside[:,:,1].flatten()[::1])
        
        pt1 = np.array([prev.shape[1]//2,prev.shape[0]//2],dtype=np.uint32)
        pt2 = np.array([prev.shape[1]//2,prev.shape[0]//2],dtype=np.uint32) + 10*np.array([flow_outside_x,flow_outside_y],dtype=np.uint32)
        pt2[1] = min(pt2[1],img.shape[0])
        pt2[0] = min(pt2[0],img.shape[1])
        img = cv.arrowedLine(
            img,
            pt1 = pt1,
            pt2 = pt2,
            color=[255,255,0],
            thickness=5,
            tipLength=0.5
        )

    if plot_outside_detail:
        step = 5
        for ii in range(0,flow_outside.shape[1],step):
            for jj in range(0,flow_outside.shape[0],step):
                if not any(np.isnan(flow_outside[jj,ii,:])):
                    img = cv.arrowedLine(
                        img,
                        pt1 = np.array([ii,jj]),
                        pt2 = np.array([ii,jj])+ np.array([flow_outside[jj,ii,0],flow_outside[jj,ii,1]]).astype(int),
                        color=[0,0,200],
                        thickness=1,
                        tipLength=0.5
                    )
    if show_figure:
        plt.figure(dpi=300)
        plt.imshow(cv.cvtColor(img,cv.COLOR_BGR2RGB))
        plt.show()
    return img



if __name__=='__main__':
    main()