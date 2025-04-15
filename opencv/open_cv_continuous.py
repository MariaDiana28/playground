import cv2
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

#Set the resolution
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Capture frame-by-frame
frameNr = 0
timestamp_old = 0
while frameNr < 1000:
    ret, frame = cap.read()

    timestamp = 0
    if ret:
        timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)
        
    # save
    #cv2.imwrite("outputImage.jpg", frame)
    cv2.imwrite("outputImage.png", frame)

    print(frameNr, ret, timestamp-timestamp_old)
    frameNr += 1
    timestamp_old = timestamp

# When everything done, release the capture
cap.release()