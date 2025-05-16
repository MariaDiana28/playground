
import os
import math
import cv2
import numpy as np
import threading
import time
from Nao import Nao

# --- Initialize NAO ---
nao = Nao()

# Posture support
stable_joints = [
    "LHipPitch", "RHipPitch",
    "LHipRoll", "RHipRoll",
    "LKneePitch", "RKneePitch",
    "LAnklePitch", "RAnklePitch"
]
for name in nao.joint_names:
    nao.jointStiffnessData[name] = 0.0
for joint in stable_joints:
    nao.jointStiffnessData[joint] = 1.0
    nao.jointMotorData[joint] = nao.jointSensorData[joint]

nao.jointStiffnessData["HeadYaw"] = 0.2
nao.jointStiffnessData["HeadPitch"] = 0.2

# Camera setup
frame_width = 320
frame_height = 240
center_x = frame_width // 2
center_y = frame_height // 2
fov_h = 60
fov_v = 47.6
gain = 0.4

# HSV color range for orange/yellow ball
lower = np.array([5, 100, 140])
upper = np.array([25, 255, 255])

# Video capture
cap_bottom = cv2.VideoCapture('/dev/video-bottom')
cap_top = cv2.VideoCapture('/dev/video-top')
if not cap_bottom.isOpened() or not cap_top.isOpened():
    print("Could not open one or both cameras.")
    exit()

# Shared data
ball_position_bottom = [None]
ball_position_top = [None]
lock_bottom = threading.Lock()
lock_top = threading.Lock()

# --- Ball Detection Thread ---
def detect_ball_from_camera(cap, lock, position_var, label=""):
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame from {label} camera.")
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=3)
        mask = cv2.dilate(mask, kernel, iterations=6)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        with lock:
            position_var[0] = None
            if contours:
                largest = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    position_var[0] = (cX, cY)
        time.sleep(0.01)

# --- Head Movement Thread ---
def track_ball():
    # Sound and state for ball detection
    #SOUND_FILE = "../Media/weeeee.wav"
    SOUND_FILE = "../Media/bip_power_off.wav"
    found_prev = False

    t_0 = nao.getTime()
    previous_yaw = nao.jointSensorData["HeadYaw"]
    previous_pitch = nao.jointSensorData["HeadPitch"]

    deadzone_pixels = 10
    alpha = 0.2

    while nao.update():
        t = nao.getTime() - t_0

        # Prioritize bottom camera
        with lock_bottom:
            pos = ball_position_bottom[0]
        if not pos:
            with lock_top:
                pos = ball_position_top[0]

        # Trigger sound once when ball is found
        if pos and not found_prev:
            print("FOUND_BALL")
            os.system("aplay " + SOUND_FILE + " &")
            found_prev = True
        elif not pos:
            found_prev = False

        current_yaw = nao.jointSensorData["HeadYaw"]
        current_pitch = nao.jointSensorData["HeadPitch"]

        if pos:
            cX, cY = pos
            offset_x = cX - center_x
            offset_y = cY - center_y

            if abs(offset_x) > deadzone_pixels or abs(offset_y) > deadzone_pixels:
                angle_offset_yaw = math.radians((offset_x / float(frame_width)) * fov_h)
                angle_offset_pitch = -math.radians((offset_y / float(frame_height)) * fov_v)

                target_yaw = current_yaw + gain * (angle_offset_yaw - current_yaw)
                target_pitch = current_pitch + gain * (angle_offset_pitch - current_pitch)
            else:
                target_yaw = current_yaw
                target_pitch = current_pitch
        else:
            # Idle sweep
            target_yaw = 0.5 * math.sin(t * 0.5)
            target_pitch = 0.1 * math.sin(t * 1.2)

        target_pitch = max(-0.67, min(0.5, target_pitch))

        smoothed_yaw = (1 - alpha) * previous_yaw + alpha * target_yaw
        smoothed_pitch = (1 - alpha) * previous_pitch + alpha * target_pitch

        nao.jointMotorData["HeadYaw"] = -smoothed_yaw / 1.5
        nao.jointMotorData["HeadPitch"] = -smoothed_pitch / 1.5

        previous_yaw = smoothed_yaw
        previous_pitch = smoothed_pitch

        print(f"{t:.2f}s | Yaw: {current_yaw:.3f} -> {nao.jointMotorData['HeadYaw']:.3f} | "
              f"Pitch: {current_pitch:.3f} -> {nao.jointMotorData['HeadPitch']:.3f}")

        time.sleep(0.06)

# --- Start Threads ---
threading.Thread(target=detect_ball_from_camera, args=(cap_bottom, lock_bottom, ball_position_bottom, "bottom"), daemon=True).start()
threading.Thread(target=detect_ball_from_camera, args=(cap_top, lock_top, ball_position_top, "top"), daemon=True).start()

# Start head tracking
track_ball()
