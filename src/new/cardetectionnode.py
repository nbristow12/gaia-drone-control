#!/home/ffil/gaia-feedback-control/gaia-fc-env/bin/python3
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

import argparse
from pathlib import Path
import torch
print(f"Torch setup complete. Using torch {torch.__version__} ({torch.cuda.get_device_properties(0).name if torch.cuda.is_available() else 'CPU'})")

FILE = Path(__file__).resolve()
YOLOv5_ROOT = FILE.parents[1] / 'yolov5_smoke'  # YOLOv5 root directory
if str(YOLOv5_ROOT) not in sys.path:
    sys.path.append(str(YOLOv5_ROOT))  # add YOLOv5_ROOT to PATH
YOLOv5_ROOT = Path(os.path.relpath(YOLOv5_ROOT, Path.cwd()))  # relative

from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device
from utils.plots import Annotator, colors
from utils.augmentations import letterbox

import time

#global publisher and boundingbox
global pub,box
#global initialized variables for detection model
global imgsz, model, device, names


#--------OPTION TO VIEW DETECTION RESULTS IN REAL_TIME-------------#
VIEW_IMG=True
#-----------------------------------------------------#

def imagecallback(img):
    global pub,box
    global imgsz, model, device, names

    if rospy.Time.now() - img.header.stamp > rospy.Duration(.25):
        print("dropping old image\n")
        return

    box = BoundingBox2D()
    start = time.time()
    
    img_numpy = np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.width,-1)
    

    
    #TODO: Run network, set bounding box parameters
    car = detect_car(img_numpy,imgsz,model,device,names)
    if len(car) != 0 and car[0].confidence > 0.5:
        print(car[0].bounding_box, car[0].confidence)

        box.center.x = car[0].bounding_box[0]
        box.center.y = car[0].bounding_box[1]
        box.center.theta = 0
        box.size_x = car[0].bounding_box[2]
        box.size_y = car[0].bounding_box[3]
        pub.publish(box)
    end = time.time()
    
    # #image test code
    # if VIEW_IMG:
    #     cv2.imshow('image window',img_numpy)
    #     cv2.waitKey(0)
    
    print("finished callback for image", img.header.seq,"in",end-start, "seconds \n")

def init_detection_node():
    global pub,box
    pub = rospy.Publisher('/gaia/bounding_box', BoundingBox2D, queue_size=1)
    box = BoundingBox2D()

    # Initialize detection code before subscriber because this takes some time
    global imgsz, model, device, names
    print('Initializing model')
    weights=YOLOv5_ROOT / 'yolov5s.pt'
    # weights=YOLOv5_ROOT / 'smoke.pt'
    model, device, names = detect_init(weights)
    imgsz = [640,640] # scaled image size to run inference on
    model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.parameters())))  # run once
    
    
    # print('Loading image')
    # img_path = 'traffic.jpeg'
    # img_path = 'smoke_stack.jpeg'
    # img_path = str(YOLOv5_ROOT / 'data/images/zidane.jpg')
    # img = cv2.imread(img_path)

    # End detection initialization

    rospy.Subscriber('/camera/image', Image, imagecallback)
    rospy.init_node('detectionnode', anonymous=False)

    rospy.spin()

def detect_car(img0,imgsz,model,device,names):
    
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
    # print(img.shape)
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

        
        if VIEW_IMG:
            im_with_boxes = annotator.result()
            cv2.imshow('detection results', im_with_boxes)
            cv2.waitKey(1)  # 1 millisecond



    #------return car with max confidence------------#
    bestcar = []
    bestconf = 0
    for ob in obj:
        if ob.object_class == 'car' and ob.confidence > bestconf:
            bestcar = [ob]
            bestconf = ob.confidence
    return bestcar

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