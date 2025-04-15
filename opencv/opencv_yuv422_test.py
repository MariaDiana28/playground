import cv2
import numpy as np
import time

def yuv422_to_yuv888(yuv422):

    # unpack to yuv888
    yuv888 = np.zeros((yuv422.shape[0], yuv422.shape[1], 3), dtype=np.uint8)
    
    # copy y channel
    yuv888[:,    :, 0] = yuv422[:,:,0]

    # copy u channel
    yuv888[:,  ::2, 1] = yuv422[:, ::2,1]
    yuv888[:, 1::2, 1] = yuv422[:, ::2,1]

    # copy y channel
    yuv888[:,  ::2, 2] = yuv422[:,1::2,1]
    yuv888[:, 1::2, 2] = yuv422[:,1::2,1]

    return yuv888


# Open the camera device
# use the default camera
# cap = cv2.VideoCapture(0)
# On NAO
# /dev/video0
cap = cv2.VideoCapture('/dev/video-top')
# /dev/video1
#cap = cv2.VideoCapture('/dev/video-bottom')


# https://docs.opencv.org/3.4/dd/d01/group__videoio__c.html
# disable builtin RGB conversion to capture native YUV422
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

#CV_CAP_MODE_BGR = 0,
#CV_CAP_MODE_RGB = 1,
#CV_CAP_MODE_GRAY = 2,
#CV_CAP_MODE_YUYV = 3 
cap.set(cv2.CAP_PROP_MODE, 3)

# print all availiable properties
#for s in dir(cv2):
#    if s.startswith("CAP_"):
#        print(s)


# Check if camera was opened correctly
if not (cap.isOpened()):
    print("Could not open video device")
    exit()
    
# wait a bit and skip the firt couple frames to give the camera time to initialize
for i in range(1, 10):
    ret, frame = cap.read()

    
# wait a bit before reading the first frame
time.sleep(2)

# read one frame
ret, frame = cap.read()

# (480, 640, 2)
# the frame is in yuyv format
print(frame.shape)

yuv888 = yuv422_to_yuv888(frame)

# convert to BGR for saving :)
bgr = cv2.cvtColor(yuv888, cv2.COLOR_YUV2BGR)

cv2.imwrite("outputImage.png", bgr)

cap.release()