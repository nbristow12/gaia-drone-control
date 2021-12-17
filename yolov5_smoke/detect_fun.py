# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on single image passed as input to detect function
Returns bounding box info

"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
print(f"Torch setup complete. Using torch {torch.__version__} ({torch.cuda.get_device_properties(0).name if torch.cuda.is_available() else 'CPU'})")
# import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
GAIA_DRONE_ROOT = FILE.parents[0]  # YOLOv5 GAIA_DRONE_root directory
if str(GAIA_DRONE_ROOT) not in sys.path:
    sys.path.append(str(GAIA_DRONE_ROOT))  # add GAIA_DRONE_ROOT to PATH
GAIA_DRONE_ROOT = Path(os.path.relpath(GAIA_DRONE_ROOT, Path.cwd()))  # relative

from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device

from utils.augmentations import letterbox


def main():
    
   
    print('Initializing model')
    weights=GAIA_DRONE_ROOT / 'yolov5s.pt'
    # weights=GAIA_DRONE_ROOT / 'smoke.pt'
    model, device, names = detect_init(weights)
    imgsz = [640,640] # scaled image size to run inference on
    model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.parameters())))  # run once
    
    
    print('Loading image')
    img_path = 'traffic.jpeg'
    # img_path = 'smoke_stack.jpeg'
    # img_path = str(GAIA_DRONE_ROOT / 'data/images/zidane.jpg')
    img = cv2.imread(img_path)
    
    
    print('Running inference')
    objects = detect(img,imgsz,model,device, names)
    
    print('Drawing bounding box')
    new = img.copy()
    if isinstance(objects,list):
        for object in objects:
            
            bb = object.bounding_box.copy()
            bb[[0,2]] = bb[[0,2]]*img.shape[1]
            bb[[1,3]] = bb[[1,3]]*img.shape[0]
            bb = (int(bb[0]-bb[2]/2) , int(bb[1]-bb[3]/2), int(bb[0]+bb[2]/2) , int(bb[1]+bb[3]/2))
            
            label = '%s: %0.2f' % (object.object_class,object.confidence)
            
            new = addBoxAndLabel(new,bb,label)
        cv2.imshow('frame',new)
        cv2.waitKey(0)
    else:
        print('x,y,w,h: ')
        print(objects)
    
    
    
    print('DONE')
    
    return


def detect(img0,imgsz,model,device,names):
    
    # weights=GAIA_DRONE_ROOT / 'yolov5s.pt'  # model.pt path(s)
    # source=GAIA_DRONE_ROOT / 'data/images'  # file/dir/URL/glob, 0 for webcam
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
    # project=GAIA_DRONE_ROOT / 'runs/detect'  # save results to project/name
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
    # Process predictions
    for i, det in enumerate(pred):  # per image
        seen += 1
        # im0 = img0.copy()
        # p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)
        gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
            # Write results
            obj = [] # initializing output list
            for *xyxy, conf, cls in reversed(det):
                
                # extracting bounding box, confidence level, and name for each object
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                confidence = float(conf)
                object_class = names[int(cls)]
                
                # adding object to list
                obj.append(DetectedObject(np.array(xywh),confidence,object_class))

    #------uncomment for just returning object with max confidence------------#
    # conf = np.array([ob.confidence for ob in obj])
    # ind = np.argmax(conf)
    # return obj[ind].bounding_box
    #-------------------------------------------------------------------------#

    #------uncomment for just returning car with max confidence------------#
    #types = np.array([ob.object_class for ob in obj])
    #print(types)
    #cars = obj[types == 'car']    
    #conf = np.array([ob.confidence for ob in cars])
    #ind = np.argmax(conf)
    #return np.array(cars[ind])
    bestcar = []
    bestconf = 0
    for ob in obj:
        if ob.object_class == 'car' and ob.confidence > bestconf:
            bestcar = [ob]
            bestconf = ob.confidence
    return bestcar
    #-------------------------------------------------------------------------#
    
    return obj

def detect_init(weights=GAIA_DRONE_ROOT / 'yolov5s.pt'):
    
    device = select_device(device='',batch_size=None)   # usually cpu or cuda
    w = str(weights[0] if isinstance(weights, list) else weights) # model weights, from .pt file
    model = torch.jit.load(w) if 'torchscript' in w else attempt_load(weights, map_location=device) # initializing model in torch
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    
    return model, device, names

def addBoxAndLabel(im,box, label='object', color=(128, 128, 128), txt_color=(255, 255, 255),lw=2):
    #  function from yolov5 for adding bounding box info
    p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
    cv2.rectangle(im, p1, p2, color, thickness=lw, lineType=cv2.LINE_AA)
    tf = max(lw - 1, 1)  # font thickness
    w, h = cv2.getTextSize(label, 0, fontScale=lw / 3, thickness=tf)[0]  # text width, height
    outside = p1[1] - h - 3 >= 0  # label fits outside box
    p2 = p1[0] + w, p1[1] - h - 3 if outside else p1[1] + h + 3
    cv2.rectangle(im, p1, p2, color, -1, cv2.LINE_AA)  # filled
    cv2.putText(im, label, (p1[0], p1[1] - 2 if outside else p1[1] + h + 2), 0, lw / 3, txt_color,
                thickness=tf, lineType=cv2.LINE_AA)
    return im

class DetectedObject():
    def __init__(self,xywh=[],conf=0.,cls=''):
        self.bounding_box = xywh # normalized coordinates (by original image dimensions) of horizontal center, vertical center, width, height
        self.confidence=float(conf) # 0 is no confidence, 1 is full
        self.object_class = str(cls) # name of object (e.g., "person", or "smoke")

if __name__ == '__main__':
    main()