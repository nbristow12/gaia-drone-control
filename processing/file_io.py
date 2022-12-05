from pathlib import Path
import cv2 as cv
import numpy as np
from PIL import Image

DATASETS = dict(
                snow =    Path(r'D:\GAIA\Snow_2021-04-01\cam1_ThinkPad\Snow_Solo_Run1_Cam1'),
                # snow =    Path(r'G:\Shared drives\Flow Field Imaging Lab\Projects\Ongoing\NSF-MRI-GAIA_2020\Data\Sample backyard snow\snowfall.mp4'),
                spray =   Path(r'G:\Shared drives\Flow Field Imaging Lab\Projects\Ongoing\NSF-MRI-GAIA_2020\Data\Sample data_SAF sprays\Sample data_SAF sprays_20210329_165220.mp4'),
                # chimney = Path(r'G:\Shared drives\Flow Field Imaging Lab\Projects\Ongoing\NSF-MRI-GAIA_2020\Data\Sample data_chimney vapor plume\SAFL chimney_20210215_085417.mp4'),
                chimney = Path(r'G:\Shared drives\Flow Field Imaging Lab\Projects\Ongoing\NSF-MRI-GAIA_2020\Data\Sample data_chimney vapor plume\East campus_20210212_125608.mp4'),
                peanuts = Path(r'D:\GAIA\PackingPeanuts_BlackBackground_2021-03-31'),
                forest = Path(r'G:\Shared drives\Flow Field Imaging Lab\Projects\Ongoing\NSF-MRI-GAIA_2020\Data\Forest vapors\vapor_condensation.mp4'),
                mars = Path(r'G:\Shared drives\Flow Field Imaging Lab\Projects\Ongoing\NSF-MRI-GAIA_2020\Data\Mars Dust\PIA24589.mp4')
            )

class Images:
    def __init__(self,path,first_frame=0,step=1,ext = '.raw',rows = 1464,cols = 1936):
        if path.suffix == '':
            self.skip = step-1
            self.type = 'photo'
            self.ext = ext    # file format
            self.dataset = list(path.glob(f'*{self.ext}'))    
            # sorting alphabetically
            self.dataset.sort()
            # selecting images
            self.dataset = self.dataset[first_frame:-1:step]
            self.num_frames = len(self.dataset)
            
            self.rows  = rows    # size of 2.8MP Blackfly S sensor
            self.cols = cols
            self.image_format = np.uint8  # bit format
            self.roi = ()
            # self.roi = (0,1464,0,1936)   # full FOV
        else:
            self.type = 'video'
            self.skip = step-1
            self.dataset = cv.VideoCapture(str(path))
            self.dataset.set(cv.CAP_PROP_POS_FRAMES , first_frame)
            self.rows = int(self.dataset.get(cv.CAP_PROP_FRAME_HEIGHT))
            self.cols = int(self.dataset.get(cv.CAP_PROP_FRAME_WIDTH))
            self.num_frames = int(self.dataset.get(cv.CAP_PROP_FRAME_COUNT)) - first_frame
            self.ext = path.suffix    
            self.roi = ()           # if unfilled, then full field of view



def load_frame(images,index,gray=True,mono=False):
    """loads a specified frame from either a video file or an image dataset
        converts to grayscale by default, otherwise loads

    Args:
        images (object): Image class, contains path and other image set info
        index (integer): frame index to load

    Returns:
        [im]: [grayscale image loaded from file]
    """
    # reading file
    if images.type == 'photo':
            if images.ext == '.raw':
                fd = open(str(images.dataset[index]),'rb')
                # read into numpy array
                f = np.fromfile(fd, dtype=images.image_format)    
                fd.close()
                bayer = f.reshape((images.rows,images.cols)) #notice row, column format
                if mono:
                    im = bayer
                else:
                    if gray:
                        im = cv.cvtColor(bayer,cv.COLOR_BayerBG2GRAY)
                    else:              
                        im = cv.cvtColor(bayer,cv.COLOR_BayerBG2BGR)
            else:
                # rgb = np.array(Image.open(str(images.dataset[index])))
                bgr = cv.imread(str(images.dataset[index]))
                if len(bgr.shape)==3:
                    if gray:
                        im = cv.cvtColor(bgr,cv.COLOR_BGR2GRAY)
                    else:
                        im = bgr
                        # im = cv.cvtColor(bgr,cv.COLOR_BGR2RGB)
                else:
                    im = bgr
        
    # reading video frame
    elif images.type == 'video':
        if index != 0:
            # buring through frames to skip
            tt = 1
            while tt<=images.skip:
                _,_, = images.dataset.read()
                # print('frame skipped')
                tt+=1
        # reading desired frame
        ret, im = images.dataset.read()

        if len(im.shape)==3 and gray==True:
            im = cv.cvtColor(im,cv.COLOR_BGR2GRAY)

    return im

def load_image(path):
    """loads image file into numpy array, as RGB if color

    Args:
        path ([type]): [description]

    Returns:
        [type]: [description]
    """
    if str(path)[-4:] == '.raw':
        fd = open(path,'rb')
        f = np.fromfile(fd, dtype=np.uint8)
        fd.close()
        n = len(f)
        if n==1464*1936:
            im = f.reshape((1464, 1936)) #notice row, column format
            im = cv.cvtColor(im,cv.COLOR_BayerBG2RGB)
        elif n==2448*2048:
            im = f.reshape((2048,2448)) #notice row, column format
        elif n==968*732:
            im = f.reshape((732, 968)) #notice row, column format
            im = cv.cvtColor(im,cv.COLOR_BayerBG2RGB) 
        elif n==1080*1440:
            im = f.reshape((1080, 1440)) #notice row, column format
        elif n==1088*1080:
            im = f.reshape((1080, 1440))
        elif n==3840*2160:
            im = f.reshape((2160, 3840))
        # else:
        #     im = f.reshape((732, 968)) #notice row, column format
        #     im = cv.cvtColor(im,cv.COLOR_BayerBG2RGB) 
    else:
        im = np.array(Image.open(str(path)))        

    return im