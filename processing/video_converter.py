
from tkinter import filedialog
from tkinter import *
from PIL import Image
import cv2 as cv
from pathlib import Path
from distutils.util import strtobool
import platform

from cv2 import INTER_AREA
from cv2 import INTER_CUBIC




def main():

    GRAYSCALE = False
    skip = 0
    flag_rename = False
    # ext = input('Enter file extension for saving [e.g., ".tiff"]: ')

    root = Tk()
    root.withdraw()
    file = Path(list(filedialog.askopenfilenames())[0])
    fol = file.parent

    cap = cv.VideoCapture(str(file))
    nframes = int(cap.get(cv.CAP_PROP_FRAME_COUNT))
    print('Total frames: %d' % nframes)
    if nframes==0:
        nframes = 100000 # this is in the case where the 
        print(f'Video not saved properly, no frame count possible')
        frame_select = False
    else:
        frame_select = bool(strtobool(input('Select frames? [y/n]: ')))
    # nframes = 75
    
    
    if frame_select:
        framemin = int(input('First frame (from 1): ')) - 1
        framemax = int(input('Last frame: '))
    else:
        framemin = 0
        framemax = nframes
    # framemin=1
    # framemax = nframes
    print('Choose output format...')
    print('1. avi')
    print('2. mp4')
    format_out = int(input(': '))
    if format_out==1:
        format_out = 'avi'
    elif format_out==2:
        format_out = 'mp4'
    else:
        print('ERROR: Wrong format selected')
        return
    
    print('Current video frame rate: %f' % cap.get(cv.CAP_PROP_FPS))
    videoFps_opt = bool(strtobool(input('Change fps? [y/n]: ')))
    if not videoFps_opt:
        videoFps = cap.get(cv.CAP_PROP_FPS)
    else:
        videoFps = float(input('Frame rate of new video?: '))
    cap.set(cv.CAP_PROP_POS_FRAMES , framemin)


    crop = bool(strtobool(input('Choose ROI for cropping? [y/n]: ')))
    # compress = bool(strtobool(input('Compress video? [y/n]: ')))
    resize = bool(strtobool(input('Change frame width? [y/n]: ')))
    prev_video_width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
    if resize:
        print(f'Current frame width: {prev_video_width} pixels')
        new_video_width = float(input('New video width (pixels): '))

    frame_choices = range(framemin,framemax,skip+1)
    
    
    ii=0
    for frame_id in frame_choices:
        
        try:
            if skip != 0:
                cap.set(cv.CAP_PROP_POS_FRAMES , frame_id)
            ret,im = cap.read()
            if not ret:
                cap.release()
                try:
                    video.release()
                finally:
                    return
            print(f'Writing frame {frame_id} of {framemax}',end='\r')


            if ii==0:
                
                if len(im.shape)==3:
                    rows, cols, bands = im.shape
                else:
                    rows, cols = im.shape
                if crop:
                    roi = cv.selectROI('Select ROI to analyze flow. Press enter when done', im, fromCenter=False)
                    print(f'ROI: {roi}')
                    if roi[2] == 0 and roi[3] == 0:
                        roi = (0, 0, cols, rows)
                else:
                    roi = (0, 0, cols, rows)
                cv.destroyAllWindows()
                # os.makedirs(str(fol.joinpath(f'{file.stem}_frames')),exist_ok=True)

            im = im[roi[1]:roi[1]+roi[3],roi[0]:roi[0]+roi[2],:]

            # resize frame for saving smaller files
            if resize:
                AR = im.shape[0]/im.shape[1]
                # height = new_video_width*im.shape[0]
                if new_video_width < prev_video_width:
                    im = cv.resize(im,dsize = (int(new_video_width),int(new_video_width*AR)),interpolation=INTER_AREA)
                else:
                    im = cv.resize(im,dsize = (int(new_video_width),int(new_video_width*AR)),interpolation=INTER_CUBIC)
                # im = cv.resize(im,dsize = (1280,720),interpolation=INTER_AREA)
                # im = cv.resize(im,dsize = (640,360),interpolation=INTER_AREA)


            if GRAYSCALE:
                im = cv.cvtColor(im,cv.COLOR_BGR2GRAY)
                im = cv.cvtColor(im,cv.COLOR_GRAY2BGR)

            if ii==0:
                # if file.suffix.lower()=='.mp4':
                #     codec = cv.VideoWriter_fourcc(*'XVID')
                #     codec = 0x7634706d
                # elif file.suffix.lower()=='.avi':
                # codec = cv.VideoWriter_fourcc('M','J','P','G')
                # codec = cv.VideoWriter_fourcc('X','2','6','4')
                if format_out =='avi':
                    codec = cv.VideoWriter_fourcc('M','J','P','G')
                elif format_out=='mp4':
                    os = platform.platform()
                    if 'macOS' in os:
                        codec = cv.VideoWriter_fourcc(*'avc1')
                    else:
                        cv.VideoWriter_fourcc('H','2','6','4')
                if frame_select:
                    savename = '%s_Frames%d-%d' % (file.stem,framemin+1,framemax)
                else:
                    savename = file.stem
                if videoFps_opt:
                    savename = '%s_%dfps' % (savename,videoFps)
                if resize:
                    savename = '%s_%dp' % (savename,new_video_width)
                if crop:
                    savename = '%s_%dx%d' % (savename,roi[3],roi[2])
                # savename = file.parent.joinpath('%s%s' % (savename,file.suffix))
                savename = file.parent.joinpath('%s.%s' % (savename,format_out))
                if str(savename) == str(file):
                    savename = savename.parent.joinpath(f'{savename.stem}_new{savename.suffix}')
                    flag_rename = True

                video = cv.VideoWriter(filename=str(savename),
                            fourcc=codec, 
                            fps=videoFps, 
                            frameSize=(im.shape[1],im.shape[0]))
            video.write(im)
            ii+=1
        except:
            video.release()
            # if the filename was the same as the savename, then replace the original, since all that was done was fixing the codec
            if flag_rename:
                import os
                os.remove(file)
                os.rename(savename,file)
            return

    video.release()
    
    # if the filename was the same as the savename, then replace the original, since all that was done was fixing the codec
    if flag_rename:
        import os
        os.remove(file)
        os.rename(savename,file)



    return


if __name__ == "__main__":
    main()