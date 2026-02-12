# ======================= PYTHON: vision + PID + servo commands =======================
import cv2
import numpy as np
from pypylon import pylon
import struct
import serial
import time
import csv
from datetime import datetime

# ---------- SERIAL / ARDUINO ----------
ARDUINO_PORT = 'COM3'
BAUD_RATE = 115200

try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0)
    time.sleep(2.0)
    arduino.flushInput()
    arduino.flushOutput()
except Exception as e:
    print("Warning: could not open Arduino serial:", e)
    arduino = None

# ---------- CAMERA CONFIG ----------
DESIRED_WIDTH = 580
DESIRED_HEIGHT = 580

THRESHOLD_VALUE = 100
MIN_CONTOUR_AREA = 100
MIN_RADIUS = 6
MAX_RADIUS = 120

PRINT_EVERY = 60

# ---------- SERVO / PID CONFIG (all PID now in Python) ----------
SERVO_MIN = 45
SERVO_MAX = 135
SERVO_X_OFFSET = 90
SERVO_Y_OFFSET = 90

# PID gains (start similar to your Arduino values; tune here)
kP_x = 14.0
kI_x = 0.05
kD_x = 200.0

kP_y = 14.0
kI_y = 0.05
kD_y = 200.0

deadzone = 0.01
MAX_SERVO_CHANGE = 4          # max angle change per update (deg)
I_CLAMP = 50                  # integral clamp

# target setpoint in normalized coords (ball center)
SETPOINT_X = 0.0
SETPOINT_Y = 0.0

UPDATE_INTERVAL = 0.02        # ~50 Hz servo/PID update

# ---------- PID STATE ----------
error_x = error_y = 0.0
last_error_x = last_error_y = 0.0
integral_x = integral_y = 0.0
output_x = output_y = 0.0

# servo angles in Python
current_servo_x = SERVO_X_OFFSET
current_servo_y = SERVO_Y_OFFSET
target_servo_x = SERVO_X_OFFSET
target_servo_y = SERVO_Y_OFFSET

last_update_time = time.time()

# ---------- SERIAL PACKET: send servo angles only ----------
# 3-byte packet: uint8 servo_x, uint8 servo_y, uint8 detected_flag
def send_servo_packet(servo_x, servo_y, detected):
    if arduino is None:
        return
    sx = int(np.clip(servo_x, 0, 180))
    sy = int(np.clip(servo_y, 0, 180))
    detected_flag = 1 if detected else 0
    packet = struct.pack('<BBB', sx, sy, detected_flag)
    arduino.write(packet)

# ---------- CAMERA INITIALIZATION ----------
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
    mask = cv2.erode(mask, None, 1)
    mask = cv2.dilate(mask, None, 2)
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
    # returns [-1,1] coords (0,0) at image center
    return (cx - w / 2) / (w / 2), (cy - h / 2) / (h / 2)

def pid_step(meas_x, meas_y, ball_detected):
    global error_x, error_y
    global last_error_x, last_error_y
    global integral_x, integral_y
    global output_x, output_y
    global target_servo_x, target_servo_y
    global current_servo_x, current_servo_y

    # --- if no ball: drive servos back to center and reset PID ---
    if not ball_detected:
        integral_x = integral_y = 0.0
        last_error_x = last_error_y = 0.0
        output_x = output_y = 0.0
        target_servo_x = SERVO_X_OFFSET
        target_servo_y = SERVO_Y_OFFSET
    else:
        # error = setpoint - measurement; adjust sign to match mechanics
        error_x = (SETPOINT_X - meas_x) * 1.2
        error_y = -(SETPOINT_Y - meas_y) * 1.2

        if abs(error_x) < deadzone:
            error_x = 0.0
        if abs(error_y) < deadzone:
            error_y = 0.0

        integral_x += error_x
        integral_y += error_y

        # clamp integral
        integral_x = max(-I_CLAMP, min(I_CLAMP, integral_x))
        integral_y = max(-I_CLAMP, min(I_CLAMP, integral_y))

        derivative_x = error_x - last_error_x
        derivative_y = error_y - last_error_y

        output_x = kP_x * error_x + kI_x * integral_x + kD_x * derivative_x
        output_y = kP_y * error_y + kI_y * integral_y + kD_y * derivative_y

        last_error_x = error_x
        last_error_y = error_y

        # convert PID output to servo target
        target_servo_x = SERVO_X_OFFSET + output_x
        target_servo_y = SERVO_Y_OFFSET + output_y

        # respect servo limits
        target_servo_x = max(SERVO_MIN, min(SERVO_MAX, target_servo_x))
        target_servo_y = max(SERVO_MIN, min(SERVO_MAX, target_servo_y))

    # smooth motion: rate limit from current_servo_* to target_servo_*
    if current_servo_x < target_servo_x:
        current_servo_x = min(current_servo_x + MAX_SERVO_CHANGE, target_servo_x)
    elif current_servo_x > target_servo_x:
        current_servo_x = max(current_servo_x - MAX_SERVO_CHANGE, target_servo_x)

    if current_servo_y < target_servo_y:
        current_servo_y = min(current_servo_y + MAX_SERVO_CHANGE, target_servo_y)
    elif current_servo_y > target_servo_y:
        current_servo_y = max(current_servo_y - MAX_SERVO_CHANGE, target_servo_y)

    # final clamp
    current_servo_x = max(SERVO_MIN, min(SERVO_MAX, current_servo_x))
    current_servo_y = max(SERVO_MIN, min(SERVO_MAX, current_servo_y))

    return current_servo_x, current_servo_y

# ---------- LOGGING ----------
log_file = open("ball_tracker_pid_python.csv", "w", newline="")
writer = csv.writer(log_file)
writer.writerow(["timestamp", "ball_x_norm", "ball_y_norm", "servo_x", "servo_y", "detected"])

# ---------- MAIN LOOP ----------
frame_idx = 0
last_pid_update = time.time()
ball_detected = False
meas_x = 0.0
meas_y = 0.0

try:
    while camera.IsGrabbing():
        grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
        frame = converter.Convert(grab).GetArray()
        grab.Release()
        frame_idx += 1

        cx_f, cy_f = frame_w // 2, frame_h // 2
        crop = frame[
            cy_f - DESIRED_HEIGHT // 2 : cy_f + DESIRED_HEIGHT // 2,
            cx_f - DESIRED_WIDTH // 2  : cx_f + DESIRED_WIDTH // 2
        ]

        if crop.shape[:2] != (DESIRED_HEIGHT, DESIRED_WIDTH):
            crop = cv2.resize(crop, (DESIRED_WIDTH, DESIRED_HEIGHT))

        center = detect_ball(crop)

        if center is not None:
            ball_detected = True
            cx, cy = center
            meas_x, meas_y = normalize(cx, cy, DESIRED_WIDTH, DESIRED_HEIGHT)

            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] x={meas_x:+.3f}, y={meas_y:+.3f}")
        else:
            ball_detected = False
            meas_x = meas_y = 0.0
            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] Ball NOT detected")

        # run PID at fixed interval
        now = time.time()
        if now - last_pid_update >= UPDATE_INTERVAL:
            servo_x, servo_y = pid_step(meas_x, meas_y, ball_detected)
            send_servo_packet(servo_x, servo_y, ball_detected)
            last_pid_update = now

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

            writer.writerow([
                timestamp,
                meas_x,
                meas_y,
                servo_x,
                servo_y,
                int(ball_detected)
            ])

        # ---- same mask visualization as code 1 (black background) ----
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(blur, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
        mask = cv2.erode(mask, None, 1)
        mask = cv2.dilate(mask, None, 2)

        display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        if center is not None:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                ((_, _), r) = cv2.minEnclosingCircle(c)
                cv2.circle(display, (cx, cy), int(r), (0, 255, 0), 2)  # outer circle
                cv2.circle(display, (cx, cy), 3, (0, 0, 255), -1)      # center dot

        cv2.imshow("Camera View (PID in Python)", display)
        # -------------------------------------------------------------

        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break


finally:
    camera.StopGrabbing()
    camera.Close()
    if arduino and arduino.is_open:
        arduino.close()
    cv2.destroyAllWindows()
    log_file.close()
