"""
ball_tracker_sender_pid_python.py

- Computes PID in Python
- Sends final servo angles (X, Y) directly to Arduino
- Single camera view
"""

import cv2
import numpy as np
from pypylon import pylon
import struct
import serial
import time

# ---------- CONFIG ----------
ARDUINO_PORT = 'COM3'
BAUD_RATE = 115200

DESIRED_WIDTH = 580
DESIRED_HEIGHT = 580

THRESHOLD_VALUE = 100
MIN_CONTOUR_AREA = 100
MIN_RADIUS = 6
MAX_RADIUS = 120

PRINT_EVERY = 60

# Servo config
SERVO_X_OFFSET = 90
SERVO_Y_OFFSET = 90
SERVO_MIN = 45
SERVO_MAX = 135
MAX_SERVO_CHANGE = 4

# PID gains
kP_x, kI_x, kD_x = 20.0, 0.05, 10.0
kP_y, kI_y, kD_y = 20.0, 0.05, 10.0
deadzone = 0.01

UPDATE_INTERVAL = 0.02  # 20 ms
# --------------------------------

# ---------- SERIAL ----------
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.001)
    time.sleep(2)
except:
    arduino = None

def send_servo_angles(x_angle, y_angle):
    if arduino:
        # send as two floats
        arduino.write(struct.pack('<ff', x_angle, y_angle))

# ---------- CAMERA ----------
tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
if not devices:
    raise RuntimeError("No Basler camera found")

camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
camera.Open()
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

frame_w, frame_h = camera.Width.Value, camera.Height.Value

# ---------- PID STATE ----------
error_x = error_y = 0
last_error_x = last_error_y = 0
integral_x = integral_y = 0
output_x = output_y = 0
current_x = SERVO_X_OFFSET
current_y = SERVO_Y_OFFSET
last_servo_x = SERVO_X_OFFSET
last_servo_y = SERVO_Y_OFFSET

last_update = time.time()
frame_idx = 0

# ---------- UTIL FUNCTIONS ----------
def detect_ball(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    _, mask = cv2.threshold(blur, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_CONTOUR_AREA:
        return None, mask
    ((x,y),r) = cv2.minEnclosingCircle(c)
    if r < MIN_RADIUS or r > MAX_RADIUS:
        return None, mask
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, mask
    cx = int(M["m10"]/M["m00"])
    cy = int(M["m01"]/M["m00"])
    return (cx,cy), mask

def normalize(cx, cy):
    return ((cx - DESIRED_WIDTH/2)/(DESIRED_WIDTH/2),
            (cy - DESIRED_HEIGHT/2)/(DESIRED_HEIGHT/2))

def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))

# ---------- MAIN LOOP ----------
try:
    while camera.IsGrabbing():
        grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
        frame = converter.Convert(grab).GetArray()
        grab.Release()
        frame_idx += 1

        cx, cy = frame_w//2, frame_h//2
        crop = frame[
            cy-DESIRED_HEIGHT//2:cy+DESIRED_HEIGHT//2,
            cx-DESIRED_WIDTH//2:cx+DESIRED_WIDTH//2
        ]
        if crop.shape[:2] != (DESIRED_HEIGHT, DESIRED_WIDTH):
            crop = cv2.resize(crop, (DESIRED_WIDTH, DESIRED_HEIGHT))

        center, mask = detect_ball(crop)
        ball_detected = center is not None

        if ball_detected:
            x_norm, y_norm = normalize(*center)

            # PID calculations
            error_x = -x_norm * 1.2
            error_y = y_norm * 1.2
            error_x = 0 if abs(error_x) < deadzone else error_x
            error_y = 0 if abs(error_y) < deadzone else error_y

            integral_x += error_x
            integral_y += error_y
            derivative_x = error_x - last_error_x
            derivative_y = error_y - last_error_y

            output_x = kP_x*error_x + kI_x*integral_x + kD_x*derivative_x
            output_y = kP_y*error_y + kI_y*integral_y + kD_y*derivative_y

            integral_x = clamp(integral_x, -50, 50)
            integral_y = clamp(integral_y, -50, 50)

            servo_x = SERVO_X_OFFSET + clamp(output_x, SERVO_MIN-SERVO_X_OFFSET, SERVO_MAX-SERVO_X_OFFSET)
            servo_y = SERVO_Y_OFFSET + clamp(output_y, SERVO_MIN-SERVO_Y_OFFSET, SERVO_MAX-SERVO_Y_OFFSET)

            # Rate limiting
            servo_x = clamp(servo_x, last_servo_x-MAX_SERVO_CHANGE, last_servo_x+MAX_SERVO_CHANGE)
            servo_y = clamp(servo_y, last_servo_y-MAX_SERVO_CHANGE, last_servo_y+MAX_SERVO_CHANGE)

            last_servo_x = servo_x
            last_servo_y = servo_y

            send_servo_angles(float(servo_x), float(servo_y))

            cv2.circle(crop, center, 6, (0,255,0), -1)

            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] X_angle={servo_x:.1f} Y_angle={servo_y:.1f}")

        else:
            # Ball lost → return to center gradually
            if current_x < SERVO_X_OFFSET: current_x += 1
            elif current_x > SERVO_X_OFFSET: current_x -= 1
            if current_y < SERVO_Y_OFFSET: current_y += 1
            elif current_y > SERVO_Y_OFFSET: current_y -= 1

            last_servo_x = current_x
            last_servo_y = current_y
            send_servo_angles(float(current_x), float(current_y))

            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] Ball NOT detected")

        cv2.imshow("Cropped", crop)
        cv2.imshow("Mask", mask)
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break

finally:
    camera.StopGrabbing()
    camera.Close()
    if arduino and arduino.is_open:
        arduino.close()
    cv2.destroyAllWindows()
