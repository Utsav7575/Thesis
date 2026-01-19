import cv2
import numpy as np
from pypylon import pylon
import struct
import serial
import time
import csv

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
    arduino.write(packet)

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
    # Apply Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # Threshold the image to get binary mask
    _, mask = cv2.threshold(blur, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
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


log_file = open("log_simple.csv", "w", newline="")   # for simple code

writer = csv.writer(log_file)
writer.writerow(["time", "x", "y", "detected"])


# ---------- MAIN ----------
frame_idx = 0  # Frame counter

try:
    while camera.IsGrabbing():
        grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
        frame = converter.Convert(grab).GetArray()  # Convert to OpenCV format
        grab.Release()
        frame_idx += 1

        # Crop center region
        cx, cy = frame_w // 2, frame_h // 2
        crop = frame[
            cy - DESIRED_HEIGHT // 2 : cy + DESIRED_HEIGHT // 2,
            cx - DESIRED_WIDTH // 2  : cx + DESIRED_WIDTH // 2
        ]

        # Resize if crop dimensions are off
        if crop.shape[:2] != (DESIRED_HEIGHT, DESIRED_WIDTH):
            crop = cv2.resize(crop, (DESIRED_WIDTH, DESIRED_HEIGHT))

        center = detect_ball(crop)  # Detect ball

        if center:
            x, y = normalize(center[0], center[1],
                             DESIRED_WIDTH, DESIRED_HEIGHT)  # Normalize
            send_packet(x, y, 1)  # Send to Arduino
            cv2.circle(crop, center, 6, (0, 255, 0), -1)  # Draw detection circle

            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] x={x:+.3f}, y={y:+.3f}")  # Print info
        else:
            send_packet(0.0, 0.0, 0)  # No ball detected
            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] Ball NOT detected")

        writer.writerow([time.time(), x, y, 1])
        writer.writerow([time.time(), 0.0, 0.0, 0])

        cv2.imshow("Camera View (Latency Check)", crop)  # Show crop
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):  # Exit on ESC or 'q'
            break

finally:
    camera.StopGrabbing()
    camera.Close()
    if arduino and arduino.is_open:
        arduino.close()
    cv2.destroyAllWindows()  # Close OpenCV windows
    log_file.close()
