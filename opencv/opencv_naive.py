import cv2
import numpy as np
import time

# Open the camera device
# use the default camera
# cap = cv2.VideoCapture(0)
# On NAO
# /dev/video0
cap = cv2.VideoCapture('/dev/video-top')
# /dev/video1
#cap = cv2.VideoCapture('/dev/video-bottom')

# Check if camera was opened correctly
if not (cap.isOpened()):
    print("Could not open video device")
    exit()
    
# wait a bit and skip the firt couple frames to give the camera time to initialize
for i in range(1, 10):
    ret, frame = cap.read()


# read one frame
ret, frame = cap.read()

# (480, 640, 3)
# the frame is in RGB format
print(frame.shape)

# save the image to file
cv2.imwrite("outputImage.png", frame)

# close the capture
cap.release()