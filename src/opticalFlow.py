# import packages
import cv2 as cv
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

SAVE_VIDEO = True

# read video file
folder = Path(r'D:\GAIA\SmokePlumeDetection\1Experiments\2022-05-06_UMore\OutputImages_2022-05-06__11-28-55')
filename = 'flight19'
file = folder.joinpath(filename +'.avi')
cap = cv.VideoCapture(str(file))

videoFps = cap.get(cv.CAP_PROP_FPS)

first_frame = 5501
# first_frame = 5600
last_possible_frame = int(cap.get(cv.CAP_PROP_FRAME_COUNT))
skip = 5
cap.set(cv.CAP_PROP_POS_FRAMES , first_frame)


frame_id = first_frame
# loop and read video frames
while True:
    
    # reading current frame
    cap.set(cv.CAP_PROP_POS_FRAMES , frame_id)
    ret,prev = cap.read()

    try:
        # find corresponding bounding box
        bb_file = folder.joinpath('Detection_results','exp','labels','%s_%d.txt' % (filename,frame_id))
        bb = np.loadtxt(bb_file)

        # bounding box indices
        xx,w = (bb[[1,3]]*prev.shape[1]).astype(np.uint32)
        yy,h = (bb[[2,4]]*prev.shape[0]).astype(np.uint32)
        y1,y2,x1,x2 = yy-h//2, yy+h//2, xx-w//2, xx+w//2

        # Read next frame after skip
        cap.set(cv.CAP_PROP_POS_FRAMES , frame_id+skip)
        success, curr = cap.read()
        # if not success:
        #     break




        # convert to grayscale
        # prev_gray = cv.cvtColor(prev,cv.COLOR_BGR2GRAY)
        # prev_gray = prev[:,:,0] # use only blue band

    #     prev_gray = prev_gray[y1:y2,x1:x2]



    #     # find features in box
    #    #-------------------using SIFT points instead------------------------#
    #     sift = cv.SIFT_create()
    #     kp = sift.detect(prev_gray,None)
    #     prev_pts = []
    #     for pt in kp:
    #       if pt.size > 5:
    #         prev_pts.append(np.array(pt.pt))
    #     prev_pts = np.round(np.stack(prev_pts,axis=0).astype(np.float32).reshape((len(prev_pts),1,2)))
    #     #----------------------------------------------------------------------#


    #     # Convert to grayscale
    #     # curr_gray = cv.cvtColor(curr, cv.COLOR_BGR2GRAY)
    #     curr_gray = curr[:,:,0] # use only blue band
    #     # cropping
    #     curr_gray = curr_gray[y1:y2,x1:x2]

    #     # Calculate optical flow (i.e. track feature points)
    #     curr_pts, status, err = cv.calcOpticalFlowPyrLK(
    #         prev_gray, curr_gray, prev_pts,None)

    #     # Sanity check
    #     assert prev_pts.shape == curr_pts.shape

    #     # Filter only valid points
    #     idx = np.where(status == 1)[0]

    #     # Filter based on displacement
    #     thresh = 2
    #     diff = (curr_pts - prev_pts).squeeze()
    #     diff_x,diff_y = diff[:,0],diff[:,1]
    #     choice = np.stack((np.abs(diff_x) > thresh,np.abs(diff_y) > thresh),axis=0).any(axis=0)
    #     idx = idx[choice]
        
    #     prev_pts = prev_pts[idx]
    #     curr_pts = curr_pts[idx]

    #     # tmp = cv.cvtColor(prev_gray,cv.COLOR_GRAY2BGR)
    #     tmp = prev.copy()
    #     # for pt in prev_pts:
    #     #     tmp = cv.circle(tmp,center=pt.flatten().astype(np.uint32)+np.array([y1,x1]),radius=5,color=[0,255,0])
    #     # for pt in curr_pts:
    #     #     tmp = cv.drawMarker(tmp,position=pt.flatten().astype(np.uint32)+np.array([y1,x1]),color=[0,255,0])
    #     for ii,_ in enumerate(prev_pts):
    #         tmp = cv.arrowedLine(
    #                             tmp,
    #                             pt1=prev_pts[ii].flatten().astype(np.uint32)+np.array([y1,x1]),
    #                             pt2=curr_pts[ii].flatten().astype(np.uint32)+np.array([y1,x1]),
    #                             color=[0,255,0],
    #                             thickness=2,
    #                             tipLength=0.5
    #                             )
    #     tmp = cv.rectangle(tmp,(x1,y1),(x2,y2),color=[0,0,255],thickness=4)
    #     plt.imshow(cv.cvtColor(tmp,cv.COLOR_BGR2RGB))
    #     plt.show()
    #     fig,ax = plt.subplots(1,2)
    #     ax[0].hist(diff_x)
    #     ax[1].hist(diff_y)
    #     plt.show()

        # plt.imshow(cv.cvtColor(prev,cv.COLOR_BGR2RGB))
        # plt.show()


        # using dense optical flow with RLOF
        # flow = cv.optflow.calcOpticalFlowDenseRLOF(prev[y1:y2,x1:x2,:],curr[y1:y2,x1:x2,:],None) # using defaults for now
        
        # computing flow outside the bounding box
        flow_outside = cv.optflow.calcOpticalFlowDenseRLOF(prev,curr,None) # using defaults for now
        flow = flow_outside[y1:y2,x1:x2,:].copy()
        flow_outside[y1:y2,x1:x2,:] = np.nan
        flow_outside_x = np.nanmedian(flow_outside[:,:,0].flatten()[::1])
        flow_outside_y = np.nanmedian(flow_outside[:,:,1].flatten()[::1])

        # compensating for external motion using flow outside bounding box
        flow[:,:,0] -= flow_outside_x
        flow[:,:,1] -= flow_outside_y

        # filter out low displacements
        flow[np.abs(flow) < 5] = np.nan

        # filter out if not very white
        color_filter = np.mean(prev[y1:y2,x1:x2,:],axis=2) < 180
        flow[color_filter] = np.nan

        tmp = prev.copy()

        # drawing flow arrows
        step = 5
        for ii in range(0,flow.shape[1],step):
            for jj in range(0,flow.shape[0],step):
                if not any(np.isnan(flow[jj,ii,:])):
                    tmp = cv.arrowedLine(
                        tmp,
                        pt1 = np.array([ii,jj]) + np.array([x1,y1]),
                        pt2 = np.array([ii,jj]) + np.array([x1,y1]) + np.array([flow[jj,ii,0],flow[jj,ii,1]]).astype(int),
                        color=[0,255,0],
                        thickness=1,
                        tipLength=0.5
                    )
        # adding bounding box
        tmp = cv.rectangle(tmp,(x1,y1),(x2,y2),color=[0,0,255],thickness=4)

        # drawing bulk motion arrow in bounding box
        tmp = cv.arrowedLine(
            tmp,
            pt1 = np.array([(x1+x2)//2,(y1+y2)//2],dtype=np.uint32),
            pt2 = np.array([(x1+x2)/2,(y1+y2)/2],dtype=np.uint32) + 10*np.array([np.nanmedian(flow[:,:,0].flatten()),np.nanmedian(flow[:,:,1].flatten())],dtype=np.uint32),
            color=[255,0,0],
            thickness=5,
            tipLength=0.5
        )

        # drawing bulk motion arrow from entire frame
        tmp = cv.arrowedLine(
            tmp,
            pt1 = np.array([prev.shape[1]//2,prev.shape[0]//2],dtype=np.uint32),
            pt2 = np.array([prev.shape[1]//2,prev.shape[0]//2],dtype=np.uint32) + 10*np.array([flow_outside_x,flow_outside_y],dtype=np.uint32),
            color=[255,255,0],
            thickness=5,
            tipLength=0.5
        )

        # plt.imshow(cv.cvtColor(tmp,cv.COLOR_BGR2RGB))
        # plt.show()
        result = tmp


        # fig,ax = plt.subplots(1,2)
        # ax[0].hist(flow[:,:,0].flatten())
        # ax[1].hist(flow[:,:,1].flatten())
        # plt.show()
    except:
        result = prev           
    BLACK = (265,265,265)
    font = cv.FONT_HERSHEY_SIMPLEX
    font_size = 1
    font_color = BLACK
    font_thickness = 2
    result = cv.putText(result,'frame %d' % frame_id,(10,30),font, font_size, font_color, font_thickness, cv.LINE_AA)
    # result = cv.text

    if SAVE_VIDEO:
        if frame_id == first_frame:
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




# read next frame
# compute optical flow
# extract average or median of the flow

# use in feedback
# compare flow components ot the new velocities given by the bounding box centering error
# adjust velocities accordingly
