import sys, subprocess, os, time
import cv2
import numpy as np
from goprocam import GoProCamera
from goprocam import constants
# from PIL import Image


p = subprocess.Popen([sys.executable,
                    "/home/ffil/gaia-ws/src/GAIA-drone-control/src/goproapi/gopro_keepalive.py"],
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.STDOUT)


print('waiting 3 seconds to initialize camera')
time.sleep(3)

gpCam = GoProCamera.GoPro()
#gpCam.gpControlSet(constants.Stream.BIT_RATE, constants.Stream.BitRate.B2_4Mbps)
#gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.W480)
cap = cv2.VideoCapture("udp://127.0.0.1:10000")
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
# img = Image()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame.dtype()
    # Display the resulting frame
    if ret:
        cv2.imshow("GoPro: %d x %d" % (frame.shape[1], frame.shape[0]),frame)
        img_raw = frame.copy()
        width,height = img_raw.shape[:-1]
        print('Grabbed Image, width = %d, height = %d' % (width, height))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()