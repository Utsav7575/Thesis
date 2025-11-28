"""
ball_tracker_sender_single_view.py

Single camera view ONLY (for delay checking):
- Basler camera
- Normalized coordinates
- Print every 60 frames
- One display window
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
# --------------------------------

# ---------- SERIAL ----------
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0)
    time.sleep(2.0)
    arduino.flushInput()
    arduino.flushOutput()
except:
    arduino = None

def send_packet(x, y, detected):
    if arduino is None:
        return
    packet = struct.pack('<ffB', np.float32(x), np.float32(y), detected)
    arduino.write(packet)

# ---------- CAMERA ----------
tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
if not devices:
    raise RuntimeError("No Basler camera found")

camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
camera.Open()
camera.Width.SetValue(min(camera.Width.GetMax(), 1288))
camera.Height.SetValue(min(camera.Height.GetMax(), 720))
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

frame_w = camera.Width.Value
frame_h = camera.Height.Value

# ---------- UTILS ----------
def detect_ball(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blur, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_CONTOUR_AREA:
        return None

    ((x, y), r) = cv2.minEnclosingCircle(c)
    if r < MIN_RADIUS or r > MAX_RADIUS:
        return None

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy

def normalize(cx, cy, w, h):
    return (cx - w / 2) / (w / 2), (cy - h / 2) / (h / 2)

# ---------- MAIN ----------
frame_idx = 0

try:
    while camera.IsGrabbing():
        grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
        frame = converter.Convert(grab).GetArray()
        grab.Release()
        frame_idx += 1

        cx, cy = frame_w // 2, frame_h // 2
        crop = frame[
            cy - DESIRED_HEIGHT // 2 : cy + DESIRED_HEIGHT // 2,
            cx - DESIRED_WIDTH // 2  : cx + DESIRED_WIDTH // 2
        ]

        if crop.shape[:2] != (DESIRED_HEIGHT, DESIRED_WIDTH):
            crop = cv2.resize(crop, (DESIRED_WIDTH, DESIRED_HEIGHT))

        center = detect_ball(crop)

        if center:
            x, y = normalize(center[0], center[1],
                             DESIRED_WIDTH, DESIRED_HEIGHT)
            send_packet(x, y, 1)
            cv2.circle(crop, center, 6, (0, 255, 0), -1)

            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] x={x:+.3f}, y={y:+.3f}")
        else:
            send_packet(0.0, 0.0, 0)
            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] Ball NOT detected")

        cv2.imshow("Camera View (Latency Check)", crop)
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break

finally:
    camera.StopGrabbing()
    camera.Close()
    if arduino and arduino.is_open:
        arduino.close()
    cv2.destroyAllWindows()
