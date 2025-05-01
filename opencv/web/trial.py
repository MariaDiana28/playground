import cv2
import numpy as np

def ball(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Adjust this if your ball has a different color!
    h = 20 / 2  # ~10 in OpenCV scale (yellow)
    lower = np.array([h - 10, 100, 140])
    upper = np.array([h + 10, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=3)
    mask = cv2.dilate(mask, kernel, iterations=6)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 1)

    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)

        M = cv2.moments(c)
        a = M["m00"]

        if a > 0:
            cX = int(M["m10"] / a)
            cY = int(M["m01"] / a)
            cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1)
            print(f"Area = {a:.1f}, Center = ({cX}, {cY})")

    return img

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    result = ball(frame)
    cv2.imshow("Live Ball Detection", result)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
