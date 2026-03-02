import cv2
import numpy as np
from pypylon import pylon
import struct
import serial
import time
import csv
from datetime import datetime

loop_start_time = time.perf_counter()
last_report_time = time.perf_counter()
frame_counter_for_fps = 0

# ---------- CONFIG ----------
ARDUINO_PORT = 'COM3'  # Arduino serial port
BAUD_RATE = 115200      # Serial baud rate

DESIRED_WIDTH = 580     # Crop width
DESIRED_HEIGHT = 580    # Crop height

THRESHOLD_VALUE = 100   # Threshold for binary mask
MIN_CONTOUR_AREA = 100  # Minimum area to consider a contour as ball
MIN_RADIUS = 6          # Minimum ball radius
MAX_RADIUS = 120        # Maximum ball radius

PRINT_EVERY = 60        # Print info every 60 frames
# --------------------------------

# ---------- SERIAL ----------
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0)  # Connect to Arduino
    time.sleep(2.0)  # Wait for Arduino to initialize
    arduino.flushInput()
    arduino.flushOutput()
except:
    arduino = None  # If connection fails, continue without Arduino

def send_packet(x, y, detected):
    # Send normalized coordinates and detection flag to Arduino
    if arduino is None:
        return
    packet = struct.pack('<ffB', np.float32(x), np.float32(y), detected)
    t_serial_start = time.perf_counter()
    arduino.write(packet)
    serial_time = (time.perf_counter() - t_serial_start) * 1000

# ---------- CAMERA ----------
tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()  # List connected cameras
if not devices:
    raise RuntimeError("No Basler camera found")  # Exit if no camera

camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))  # Select first camera
camera.Open()
camera.Width.SetValue(min(camera.Width.GetMax(), 1288))  # Set camera width
camera.Height.SetValue(min(camera.Height.GetMax(), 720))  # Set camera height
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)  # Start grabbing latest image

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

frame_w = camera.Width.Value  # Camera frame width
frame_h = camera.Height.Value  # Camera frame height

# ---------- UTILS ----------
def detect_ball(img):
    # Convert image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur (Reduces noise)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Threshold the image to get binary mask (White = ball, Black = background)
    _, mask = cv2.threshold(blur, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)

    # Erode removes small noise, Dilate restores main object Stabilizes contour detection
    mask = cv2.erode(mask, None, 1)
    mask = cv2.dilate(mask, None, 2)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None  # No contours found

    c = max(contours, key=cv2.contourArea)  # Select largest contour
    if cv2.contourArea(c) < MIN_CONTOUR_AREA:
        return None  # Contour too small

    ((x, y), r) = cv2.minEnclosingCircle(c)  # Minimum enclosing circle
    if r < MIN_RADIUS or r > MAX_RADIUS:
        return None  # Radius out of bounds

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None  # Avoid division by zero

    cx = int(M["m10"] / M["m00"])  # Centroid x
    cy = int(M["m01"] / M["m00"])  # Centroid y
    return cx, cy


def normalize(cx, cy, w, h):
    # Normalize coordinates to [-1, 1]
    return (cx - w / 2) / (w / 2), (cy - h / 2) / (h / 2)


log_file = open("ball_tracker_simple.csv", "w", newline="")
writer = csv.writer(log_file)
writer.writerow(["timestamp", "x_norm", "y_norm", "detected"])

# ---------- MAIN ----------
frame_idx = 0  # Frame counter

# Fixed-rate ball tracker with 20ms packet send to Arduino

SEND_INTERVAL = 0.02  # 20 ms send interval
last_send_time = time.perf_counter()  # Track last send
frame_idx = 0  # Frame counter

# Variables to hold last measured coordinates
last_x, last_y, last_detected = 0.0, 0.0, 0

try:
    while camera.IsGrabbing():

        frame_loop_start = time.perf_counter()  # Start loop timer
        frame_idx += 1

        # ----------------- Grab frame -----------------
        grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
        frame = converter.Convert(grab).GetArray()
        grab.Release()

        # ----------------- Crop center region -----------------
        cx, cy = frame_w // 2, frame_h // 2
        crop = frame[
            cy - DESIRED_HEIGHT // 2 : cy + DESIRED_HEIGHT // 2,
            cx - DESIRED_WIDTH // 2  : cx + DESIRED_WIDTH // 2
        ]
        # Safety resize
        if crop.shape[:2] != (DESIRED_HEIGHT, DESIRED_WIDTH):
            crop = cv2.resize(crop, (DESIRED_WIDTH, DESIRED_HEIGHT))

        # ----------------- Detect ball -----------------
        t0 = time.perf_counter()  # Start processing timer
        center = detect_ball(crop)
        processing_time = (time.perf_counter() - t0) * 1000  # ms

        # Update last known coordinates if ball detected
        if center:
            last_x, last_y = normalize(center[0], center[1],
                                       DESIRED_WIDTH, DESIRED_HEIGHT)
            last_detected = 1
        else:
            last_x, last_y = 0.0, 0.0
            last_detected = 0

        # ----------------- Send coordinates at fixed interval -----------------
        now = time.perf_counter()
        if now - last_send_time >= SEND_INTERVAL:
            send_packet(last_x, last_y, last_detected)
            last_send_time = now

        # ----------------- Log CSV -----------------
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        writer.writerow([timestamp, last_x, last_y, last_detected])

        # ----------------- Create display mask -----------------
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(blur, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
        mask = cv2.erode(mask, None, 1)
        mask = cv2.dilate(mask, None, 2)

        display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        if center:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                ((_, _), r) = cv2.minEnclosingCircle(c)
                # Draw detected ball
                cv2.circle(display, center, int(r), (0, 255, 0), 2)
                cv2.circle(display, center, 3, (0, 0, 255), -1)

        cv2.imshow("Camera View", display)

        # ----------------- Print status every PRINT_EVERY frames -----------------
        if frame_idx % PRINT_EVERY == 0:
            print(f"[Frame {frame_idx}] x={last_x:+.3f}, y={last_y:+.3f}, detected={last_detected}")
            print(f"Processing Time: {processing_time:.2f} ms")
            print(f"Vision Loop Time: {(time.perf_counter() - frame_loop_start)*1000:.2f} ms")

        # ----------------- Exit on 'q' or ESC -----------------
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break

finally:
    camera.StopGrabbing()
    camera.Close()
    if arduino and arduino.is_open:
        arduino.close()
    cv2.destroyAllWindows()
    log_file.close()
