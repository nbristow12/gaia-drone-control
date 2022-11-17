#%%
from pathlib import Path
import numpy as np
import pandas as pd
import cv2 as cv
import matplotlib.pyplot as plt
import os
#%%
folder = Path('/Users/nate/UMN/FeedbackControl_2022-08-26_goodRuns/FeedbackControl_2022-08-25__11-07-16')

yolo_vid = cv.VideoCapture(str(folder.joinpath('detection','Detection.avi')))
yolo_data = pd.read_csv(folder.joinpath('detection','Timestamps.txt'))

#%%

flow_files = list(folder.joinpath('flow').glob('*.npz'))
flow_files.sort()
flow_images = list(folder.joinpath('flow').glob('*.jpg'))
flow_images.sort()

flow_frames = np.array([int(file.stem[-6:]) for file in flow_files])
#%%
# flow_data = []
# for file in flow_files[0:5]:
#     flow_data.append(np.load(file)['arr_2'])

# %% create new video
os.makedirs(folder.joinpath('merged'),exist_ok=True)
nframes = int(yolo_vid.get(cv.CAP_PROP_FRAME_COUNT))
yolo_vid.set(cv.CAP_PROP_POS_FRAMES , 0)
for ii in range(nframes):
    ret,yolo = yolo_vid.read()
    current_frame = yolo_data.FrameID[ii]
    print('current frame',current_frame)
    if ii==0:
        vid = cv.VideoWriter(str(folder.joinpath('merged','detection_with_flow.avi')),
                        fourcc = cv.VideoWriter_fourcc('M','J','P','G'),
                        fps=yolo_vid.get(cv.CAP_PROP_FPS),
                        frameSize=(yolo.shape[1],yolo.shape[0]))
    
    if any(flow_frames==current_frame):
        print('matched frame',flow_frames[np.argwhere(flow_frames==current_frame).squeeze()])
        combo_frame = cv.imread(str(flow_images[np.argwhere(flow_frames==current_frame).squeeze()]))
    else:
        print('no match found')
        combo_frame = yolo
    
    cv.imshow('combo',combo_frame)
    cv.waitKey(1)
    
    vid.write(combo_frame)
vid.release()
cv.destroyAllWindows()
