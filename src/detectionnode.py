#!/home/ffil/gaia-feedback-control/gaia-fc-env/bin/python3
# license removed for brevity
from operator import truediv
from re import sub
import rospy
from rospy.client import init_node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
import numpy as np
import cv2
import os
# import PySpin
import sys, datetime
import argparse
from pathlib import Path
import time
import torch
print(f"Torch setup complete. Using torch {torch.__version__} ({torch.cuda.get_device_properties(0).name if torch.cuda.is_available() else 'CPU'})")

#------------------------OPTIONS---------------------#
target_name = 'person' # options: smoke,car,person
max_delay = 0.5 # [seconds] delay between last detectiona nd current image after which to just drop images to catch up
conf_thres=0.4  # confidence threshold
iou_thres=0.45  # NMS IOU threshold

VIEW_IMG=True
SAVE_IMG = True
save_format = '.avi'
#-----------------------------------------------------#

# file saving folder
username = os.getlogin( )
tmp = datetime.datetime.now()
stamp = ("%02d-%02d-%02d__%02d-%02d-%02d" % 
    (tmp.year, tmp.month, tmp.day, 
    tmp.hour, tmp.minute, tmp.second))
savedir = '/home/%s/1FeedbackControl/FeedbackControl_%s/detection/' % (username,stamp) #script cannot create folder, must already exist when run
os.makedirs(savedir)


# YOLO paths and importing
FILE = Path(__file__).resolve()
YOLOv5_ROOT = FILE.parents[1] / 'src/modules/yolov5'  # YOLOv5 root directory
if str(YOLOv5_ROOT) not in sys.path:
    sys.path.append(str(YOLOv5_ROOT))  # add YOLOv5_ROOT to PATH
# print(YOLOv5_ROOT)
YOLOv5_ROOT = Path(os.path.relpath(YOLOv5_ROOT, Path.cwd()))  # relative
from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device
from utils.plots import Annotator, colors
from utils.augmentations import letterbox

#global publisher and boundingbox
global pub,box, video, timelog
#global initialized variables for detection model
global imgsz, model, device, names

# labeling text on image
BLACK = (265,265,265)
font = cv2.FONT_HERSHEY_SIMPLEX
font_size = 1
font_color = BLACK
font_thickness = 2

def imagecallback(img):

    global pub,box,video,timelog
    global imgsz, model, device, names
    box = Detection2D()
    # print(img.header.stamp)
    # print('Time before running detection')
    # print('Image %d' % img.header.seq)
    # print(img.header.stamp)

    # adding to time stamp log
    timelog.write('%d,%f\n' % (img.header.seq,time.time()))

    # converting image to numpy array
    img_numpy = np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.width,-1)

    if rospy.Time.now() - img.header.stamp > rospy.Duration(max_delay):
        # print("DetectionNode: dropping old image from detection\n")
        # text_to_image = 'skipped'
        return
    else:
        # print('DetectionNode: Running detection inference')
        object,img_numpy = detection(img_numpy,imgsz,model,device,names,savenum=img.header.seq)
        
        # print(img.header)
        # print('Printing time stamps at exchange in detection')
        # print(img.header.stamp)
        box.header.seq = img.header.seq
        box.header.stamp = img.header.stamp
        box.header.frame_id = ''
        # print(box.header.stamp)
        box.source_img = img
        if len(object) != 0 and object[0].confidence > conf_thres:
            # print(object[0].bounding_box, object[0].confidence)
            box.bbox.center.x = object[0].bounding_box[0]
            box.bbox.center.y = object[0].bounding_box[1]
            box.bbox.center.theta = 0
            box.bbox.size_x = object[0].bounding_box[2]
            box.bbox.size_y = object[0].bounding_box[3]
        else:
            box.bbox.center.x = -1
            box.bbox.center.y = -1
            box.bbox.center.theta = -1
            box.bbox.size_x = -1
            box.bbox.size_y = -1
        pub.publish(box)
        text_to_image = 'processed'
        # print('Time after running detection')
        # print('Image %d' % box.source_img.header.seq)
        # print(box.source_img.header.stamp)
        
        # end = time.time()
        # print("finished callback for image", img.header.seq,"in",end-start, "seconds \n")
        img_numpy = cv2.putText(img_numpy,text_to_image,(10,30),font, font_size, font_color, font_thickness, cv2.LINE_AA)
    # viewing/saving images
    savenum=img.header.seq
    
    if SAVE_IMG:
        if save_format=='.raw':
            fid = open(savedir+'Detection-%06.0f.raw' % savenum,'wb')
            fid.write(img_numpy.flatten())
            fid.close()
        elif save_format == '.avi':
            video.write(img_numpy)
        else:
            cv2.imwrite(savedir+'Detection-%06.0f.jpg' % savenum,img_numpy)
    if VIEW_IMG:
        # im_with_boxes = annotator.result()
        cv2.imshow('gopro', img_numpy)
        cv2.waitKey(1)  # 1 millisecond

def init_detection_node():
    global pub,box,video,timelog
    pub = rospy.Publisher('/gaia/bounding_box', Detection2D, queue_size=1)
    box = Detection2D()

    # Initialize detection code before subscriber because this takes some time
    global imgsz, model, device, names
    print('Initializing YOLO model')
    
    if target_name == 'smoke':
        weights=YOLOv5_ROOT / 'smoke.pt'
    else:
        weights=YOLOv5_ROOT / 'yolov5s.pt'
    model, device, names = detect_init(weights)
    imgsz = [448,448] # scaled image size to run inference on
    model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.parameters())))  # run once
    
    # initializing video file
    if save_format=='.avi':
        codec = cv2.VideoWriter_fourcc('M','J','P','G')
        video = cv2.VideoWriter(savedir+'Detection'+save_format,
            fourcc=codec,
            fps=30,
            frameSize = (640,480)) # this size is specific to GoPro

    # initializing timelog
    timelog = open(savedir+'Timestamps.txt','w')
    timelog.write('FrameID,Timestamp\n')

    # initializing node
    rospy.init_node('detectionnode', anonymous=False)
    rospy.Subscriber('/camera/image', Image, imagecallback)
    

    rospy.spin()

def detection(img0,imgsz,model,device,names,savenum):
    
    # weights=YOLOv5_ROOT / 'yolov5s.pt'  # model.pt path(s)
    # source=YOLOv5_ROOT / 'data/images'  # file/dir/URL/glob, 0 for webcam
    # imgsz=640  # inference size (pixels)
    # conf_thres=0.25  # confidence threshold
    # iou_thres=0.45  # NMS IOU threshold
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
                if VIEW_IMG:
                    c = int(cls)  # integer class
                    label = f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                # adding object to list
                obj.append(DetectedObject(np.array(xywh),confidence,object_class))

        im_with_boxes = annotator.result()


    #------return object with max confidence------------#
    bestsmoke = []
    bestconf = 0
    for ob in obj:
        if ob.object_class == target_name and ob.confidence > bestconf:
            bestsmoke = [ob]
            bestconf = ob.confidence  

    return bestsmoke,im_with_boxes

## methods from yolov5_smoke/detect_fun.py
def detect_init(weights=YOLOv5_ROOT / 'yolov5s.pt'):
    
    device = select_device(device='',batch_size=None)   # usually cpu or cuda
    w = str(weights[0] if isinstance(weights, list) else weights) # model weights, from .pt file
    model = torch.jit.load(w) if 'torchscript' in w else attempt_load(weights, map_location=device) # initializing model in torch
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    
    return model, device, names

class DetectedObject():
    def __init__(self,xywh=[],conf=0.,cls=''):
        self.bounding_box = xywh # normalized coordinates (by original image dimensions) of horizontal center, vertical center, width, height
        self.confidence=float(conf) # 0 is no confidence, 1 is full
        self.object_class = str(cls) # name of object (e.g., "person", or "smoke")


if __name__ == '__main__':
    try:
        init_detection_node()
    except rospy.ROSInterruptException:
        pass

