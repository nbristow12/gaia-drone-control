from tkinter import filedialog
from tkinter import *
from cv2 import flip
import numpy as np
# import matplotlib.pyplot as plt
from pathlib import Path
import glob
import cv2 as cv
from PIL import Image
from distutils.util import strtobool
import os
from file_io import load_image

# ext_from = input('Old image format (e.g., png, tiff, jpeg): ')
ext_from = 'raw'
ext_to = 'tif'
# ext_to = input('New image format (e.g., png, tiff, jpeg): ')

GRAYSCALE = False
flip_y = False

# picking file
def main():

    #-----GUI method------#
    root = Tk()
    root.withdraw()
    # folder = Path(filedialog.askdirectory())
    # files = list(folder.rglob('*.%s' % ext_from)) # includes all files in subfolders with extension
    filelist = filedialog.askopenfilenames(initialdir = '/home/ffil/Data',title='Choose files')
    files = [Path(name) for name in filelist]
    folder = files[0].parent
    
    # save_folder = Path(filedialog.askdirectory(title='Choose save directory'))
    save_folder = folder.joinpath(ext_to)
    os.makedirs(save_folder,exist_ok=True)
    print('Converting %d image files' % len(files))

    for ii,path in enumerate(files):
        
        
       
        print('Converting %s' % str(path.stem),end="\r")
        # try:
        #     fd = open(str(path),'rb')
        #     f = np.fromfile(fd, dtype=np.uint8,count=1464*1936)
        #     im = f.reshape((1464, 1936)) #notice row, column format
        #     im = cv.cvtColor(im,cv.COLOR_BayerBG2RGB)
        #     fd.close()
        # except:      
        #     try:  
        #         fd = open(str(path),'rb')
        #         f = np.fromfile(fd, dtype=np.uint8,count=1080*1440)
        #         im = f.reshape((1080, 1440)) #notice row, column format
        #         fd.close()
        #     except:
        #         im = cv.imread(str(path))
        #         if im.shape[-1]==2:
        #             im = cv.cvtColor(im,cv.COLOR_BGR2RGB)
        im = load_image(path)


        if GRAYSCALE:
            im = cv.cvtColor(im,cv.COLOR_BGR2GRAY)
        
        if flip_y:
            im = np.flipud(im)

        im = Image.fromarray(im)

        if ii==0:
            os.makedirs(name = str(path.parent.joinpath(ext_to)),exist_ok=True)
        elif lastparent != str(path.parent):
            os.makedirs(name = str(path.parent.joinpath(ext_to)),exist_ok=True)


        # im.save(path.parent.joinpath(ext_to,path.stem+'.'+ext_to)) # specify file extension (e.g., png) here
        im.save(save_folder.joinpath(path.stem+'.'+ext_to))

        lastparent = str(path.parent)

if __name__ == '__main__':
    main()