import cv2
import numpy as np
from pypylon import pylon
from collections import deque
import serial
import time

# ---------------- Ball Detection Settings ----------------
lower_color = np.array([0, 0, 200])
upper_color = np.array([180, 30, 255])
min_contour_area = 100
smoothing_window = 3

# ---------------- Serial Settings ----------------
ARDUINO_PORT = 'COM3'
BAUD_RATE = 115200
SEND_INTERVAL = 1

# ---------------- Initialize Serial ----------------
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"Connected to Arduino on {ARDUINO_PORT}")
except serial.SerialException:
    print(f"Failed to connect to Arduino on {ARDUINO_PORT}")
    arduino = None

# ---------------- Tracking ----------------
previous_centers = deque(maxlen=smoothing_window)
frame_count = 0
PRINT_INTERVAL = 1  # seconds
last_print_time = 0

def normalize_coordinates(center, width, height):
    return center[0]/width, center[1]/height

def center_coordinates(center, width, height):
    return (center[0]-width/2)/(width/2), (center[1]-height/2)/(height/2)

def smooth_position(center, prev_centers):
    prev_centers.append(center)
    return tuple(map(int, np.mean(prev_centers, axis=0)))

def send_to_arduino(x_c, y_c, detected):
    if arduino and arduino.is_open:
        message = f"X:{x_c:.3f},Y:{y_c:.3f},D:{1 if detected else 0}\n"
        arduino.write(message.encode())

# ---------------- Camera ----------------
tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
camera.Open()
camera.Width.SetValue(1288)
camera.Height.SetValue(720)
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

frame_width = camera.Width.Value
frame_height = camera.Height.Value
print(f"Camera initialized: {frame_width}x{frame_height}")

desired_width = 580
desired_height = 580

while camera.IsGrabbing():
    grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if not grab_result.GrabSucceeded():
        continue

    image = converter.Convert(grab_result)
    frame = image.GetArray()
    frame_count += 1

    # Crop center
    cx, cy = frame_width//2, frame_height//2
    crop_x1 = max(cx - desired_width//2, 0)
    crop_x2 = min(cx + desired_width//2, frame_width)
    crop_y1 = max(cy - desired_height//2, 0)
    crop_y2 = min(cy + desired_height//2, frame_height)
    cropped = frame[crop_y1:crop_y2, crop_x1:crop_x2]
    if cropped.shape[:2] != (desired_height, desired_width):
        cropped = cv2.resize(cropped, (desired_width, desired_height))

    # Ball detection
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball_detected = False
    center = (0,0)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > min_contour_area:
            M = cv2.moments(largest)
            if M["m00"] > 0:
                c = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
                center = smooth_position(c, previous_centers)
                ball_detected = True
                cv2.circle(cropped, center, 5, (0,0,255), -1)

    x_norm, y_norm = normalize_coordinates(center, desired_width, desired_height)
    x_centered, y_centered = center_coordinates(center, desired_width, desired_height)

    send_to_arduino(x_centered, y_centered, ball_detected)

    # Print only every PRINT_INTERVAL (~2 FPS)
    current_time = time.time()
    if current_time - last_print_time > PRINT_INTERVAL:
        print(f"Frame {frame_count} | Normalized: X={x_norm:.3f}, Y={y_norm:.3f} | "
              f"Centered: X={x_centered:.3f}, Y={y_centered:.3f} | Ball: {'Yes' if ball_detected else 'No'}")
        last_print_time = current_time

    cv2.imshow("Cropped Frame", cropped)
    cv2.imshow("Mask", mask)
    if cv2.waitKey(1) & 0xFF in [27, ord('q'), ord('Q')]:
        break

    grab_result.Release()

camera.StopGrabbing()
camera.Close()
if arduino and arduino.is_open:
    arduino.close()
cv2.destroyAllWindows()
