import math
import cv2
import numpy as np
from Nao import Nao

# --- Initialize NAO ---

nao = Nao()
# --- Set seated posture support ---
# Prevents standing up and adds stability while seated
stable_joints = [
    "LHipPitch", "RHipPitch",
    "LHipRoll", "RHipRoll",
    "LKneePitch", "RKneePitch",
    "LAnklePitch", "RAnklePitch"
]

for name in nao.joint_names:
    nao.jointStiffnessData[name] = 0.0

# Slightly lock legs to maintain posture (won’t stand up)
for joint in stable_joints:
    nao.jointStiffnessData[joint] = 1.0
    # Optional: also explicitly freeze current position
    nao.jointMotorData[joint] = nao.jointSensorData[joint]

# Keep head mobile
nao.jointStiffnessData["HeadYaw"] = 0.2
nao.jointStiffnessData["HeadPitch"] = 0.2
# --- Camera setup ---
frame_width = 320
frame_height = 240
center_x = frame_width // 2
center_y = frame_height // 2
fov_h = 60  # Horizontal FOV in degrees
fov_v = 47.6  # Vertical FOV in degrees (approx. for NAO)
gain = 0.4  # Proportional control gain for smooth tracking

# HSV color range for orange/yellow ball
lower = np.array([5, 100, 140])
upper = np.array([25, 255, 255])

# Connect to NAO's camera
cap = cv2.VideoCapture('/dev/video-bottom')

if not cap.isOpened():
    print("Could not open webcam.")
    exit()

# --- Start timing ---
t_0 = nao.getTime()

# --- Main loop ---
while nao.update():
    t = nao.getTime() - t_0
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Ball detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=3)
    mask = cv2.dilate(mask, kernel, iterations=6)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    ball_center = None

    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            ball_center = (cX, cY)
            print(f"Area = {M['m00']:.1f}, Center = ({cX}, {cY})")

    # Get current joint values
    current_yaw = nao.jointSensorData["HeadYaw"]
    current_pitch = nao.jointSensorData["HeadPitch"]

    if ball_center:
        cX, cY = ball_center

        # --- Calculate angle offsets ---
        offset_x = cX - center_x
        offset_y = cY - center_y

        angle_offset_yaw = math.radians((offset_x / float(frame_width)) * fov_h)
        angle_offset_pitch = -math.radians((offset_y / float(frame_height)) * fov_v)

        # --- Apply proportional control ---
        target_yaw = current_yaw + gain * (angle_offset_yaw - current_yaw)
        target_pitch = current_pitch + gain * (angle_offset_pitch - current_pitch)

        # --- Clamp pitch to NAO's physical limits ---
        target_pitch = max(-0.67, min(0.5, target_pitch))  # pitch limits

        # --- Set head movement commands ---
        nao.jointMotorData["HeadYaw"] = -target_yaw/1.5
        nao.jointMotorData["HeadPitch"] = -target_pitch/1.5
    else:
        # Optional idle sweep or freeze
        #nao.jointMotorData["HeadYaw"] = math.sin(t)
        #nao.jointMotorData["HeadPitch"] = 0.0
        pass

    print(f"{t:.2f}s | Yaw: {current_yaw:.3f} -> {nao.jointMotorData['HeadYaw']:.3f} | "
          f"Pitch: {current_pitch:.3f} -> {nao.jointMotorData['HeadPitch']:.3f}")

# --- Cleanup ---
cap.release()