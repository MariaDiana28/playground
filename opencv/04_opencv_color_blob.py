import cv2
import numpy as np



def ball(img):

    # convert to hsv
    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    # regular: H \in [0,360]
    # opencv : H \in [0,180]
    h = 20/2
    lower = np.array([h-20, 100, 140])
    upper = np.array([h+20, 255, 255])

    # mask color
    mask = cv2.inRange(hsv, lower, upper)
    #cv2.imshow('color mask',mask)


    ## filters
    # https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html
    blur = cv2.GaussianBlur(img,(5,5),0)
    #cv2.imshow('blurry',blur)

    # adapt the mask with morphological operators
    # erode, dilate
    kernel = np.ones((3,3),np.uint8)
    mask = cv2.erode(mask,kernel,iterations = 3)
    #cv2.imshow('erode',mask)
    mask = cv2.dilate(mask,kernel,iterations = 6)
    #cv2.imshow('dilate',mask)
    #cv2.imshow('color mask + morphological operations',mask)

    # MORPH_OPEN = dilate + erode
    #opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Bitwise-AND mask and original image
    #blob = cv2.bitwise_and(img,img, mask= mask)
    #cv2.imshow('blob',blob)


    # detect contours
    contours, hierarchy = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img, contours, -1, (0,255,0), 1)

    # iterate over all contours
    for c in contours:

        # calculate and draw the bounding box :)
        rect = cv2.boundingRect(c)
        print(rect)
        x,y,w,h = rect # unpack
        cv2.rectangle(img, (x,y),(x+w,y+h), (255, 0, 0), 1)

        # calculate moments
        M = cv2.moments(c)
        
        # moment_{0,0} = Area/ Number of Pixels inside the contour
        a = M["m00"]
        
        # non empty
        if a > 0:
            # calculate the center of mass (COM)
            cX = int(M["m10"] / a) # average x-coordinate
            cY = int(M["m01"] / a) # average y-coordinate
            cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1)
            print("{}: a = {}, x = {}, y={}".format(len(c), a, cX, cY))

    #cv2.imshow('image',img)
    return img


# Oeffne das Bild
img = cv2.imread('outputImage.png')

img = ball(img)
cv2.imshow('image',img)

# show all images
cv2.waitKey(0)
cv2.destroyAllWindows()
