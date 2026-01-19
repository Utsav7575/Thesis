"""
ball_tracker_sender.py
Optimized vision + sender:
- Basler pylon camera
- MOSSE tracker with periodic full-detection reinit
- Kalman filter (x,y,vx,vy)
- Predicts forward by measured latency + extra margin
- Sends binary packets: <float32 x_centered, float32 y_centered, uint8 detected>
"""

import cv2
import numpy as np
from pypylon import pylon
import struct
import serial
import time
from collections import deque

# ---------- CONFIG ----------
ARDUINO_PORT = 'COM3'       # <-- set Arduino port (Windows COM3 or /dev/ttyUSBx)
BAUD_RATE = 115200          # must match Arduino
DESIRED_WIDTH = 580
DESIRED_HEIGHT = 580
DETECT_EVERY = 18           # full detection every N frames
THRESHOLD_VALUE = 100       # brightness threshold (0-255) - tune for ball/lights
MIN_CONTOUR_AREA = 100
MIN_RADIUS = 6
MAX_RADIUS = 120
ARDUINO_EXTRA_LATENCY = 0.02  # seconds to account for Arduino processing & actuation
PROCESS_TIME_WINDOW = 20
SERIAL_TIME_WINDOW = 50
PRINT_EVERY = 60           # print coordinates every N frames
# --------------------------------

# Serial init
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.001)
    time.sleep(2.0)  # allow Arduino reset
    arduino.flushInput()
    arduino.flushOutput()
    print(f"[PY] Serial open {ARDUINO_PORT} @ {BAUD_RATE}")
except Exception as e:
    arduino = None
    print(f"[PY] Warning: serial open failed: {e}")

# Basler camera init
tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
if len(devices) == 0:
    raise RuntimeError("No Basler camera found")
camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
camera.Open()
# choose supported resolution or camera defaults
camera.Width.SetValue(min(camera.Width.GetMax(), 1288))
camera.Height.SetValue(min(camera.Height.GetMax(), 720))
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
frame_w = camera.Width.Value
frame_h = camera.Height.Value
print(f"[PY] Camera initialized {frame_w}x{frame_h}")

# Kalman setup: state [x, y, vx, vy], measurement [x, y]
kf = cv2.KalmanFilter(4, 2)
kf.transitionMatrix = np.array([[1,0,1,0],
                                [0,1,0,1],
                                [0,0,1,0],
                                [0,0,0,1]], dtype=np.float32)
kf.measurementMatrix = np.array([[1,0,0,0],
                                 [0,1,0,0]], dtype=np.float32)
kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-3
kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
kf.errorCovPost = np.eye(4, dtype=np.float32)

# Tracker state
tracker = None
tracker_initialized = False
frame_idx = 0

# timing stats
proc_times = deque(maxlen=PROCESS_TIME_WINDOW)
serial_times = deque(maxlen=SERIAL_TIME_WINDOW)

def detect_ball(crop):
    # convert to grayscale for brightness detection
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    # blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # binary threshold: white ball = high values, dark plate = low values
    _, mask = cv2.threshold(blurred, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
    # clean up mask
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_CONTOUR_AREA:
        return None, mask
    ((x,y), r) = cv2.minEnclosingCircle(largest)
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None, mask
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    if r < MIN_RADIUS or r > MAX_RADIUS:
        return None, mask
    return ((cx, cy), int(r)), mask

def center_coords_to_normalized(center, w, h):
    # convert pixel center (0..w-1,0..h-1) -> centered coords (-1..1)
    x_c = (center[0] - w/2) / (w/2)
    y_c = (center[1] - h/2) / (h/2)
    return float(x_c), float(y_c)

def send_packet(xc, yc, detected):
    if arduino is None:
        return
    # pack little-endian float32, float32, uint8
    packet = struct.pack('<ffB', np.float32(xc), np.float32(yc), 1 if detected else 0)
    t0 = time.perf_counter()
    try:
        arduino.write(packet)
    except Exception as e:
        print(f"[PY] Serial write error: {e}")
    serial_times.append(time.perf_counter() - t0)

try:
    while camera.IsGrabbing():
        t0 = time.perf_counter()
        grab = camera.RetrieveResult(3000, pylon.TimeoutHandling_ThrowException)
        if not grab.GrabSucceeded():
            grab.Release()
            continue
        img = converter.Convert(grab)
        frame = img.GetArray()
        grab.Release()
        frame_idx += 1

        # crop center ROI
        cx, cy = frame_w//2, frame_h//2
        x1 = max(cx - DESIRED_WIDTH//2, 0)
        x2 = min(cx + DESIRED_WIDTH//2, frame_w)
        y1 = max(cy - DESIRED_HEIGHT//2, 0)
        y2 = min(cy + DESIRED_HEIGHT//2, frame_h)
        cropped = frame[y1:y2, x1:x2]
        if cropped.shape[0] != DESIRED_HEIGHT or cropped.shape[1] != DESIRED_WIDTH:
            cropped = cv2.resize(cropped, (DESIRED_WIDTH, DESIRED_HEIGHT))

        center = None
        radius = 0
        mask = np.zeros((DESIRED_HEIGHT, DESIRED_WIDTH), dtype=np.uint8)

        # Re-detect periodically or if tracker lost
        if (frame_idx % DETECT_EVERY == 0) or (not tracker_initialized):
            det, mask = detect_ball(cropped)
            if det is not None:
                center, radius = det
                # initialize Kalman
                kf.statePre = np.array([[center[0]],[center[1]],[0.],[0.]], dtype=np.float32)
                kf.statePost = kf.statePre.copy()
                # init tracker from bbox
                try:
                    tracker = cv2.TrackerMOSSE_create()
                    bbox = (center[0]-radius, center[1]-radius, radius*2, radius*2)
                    tracker.init(cropped, bbox)
                    tracker_initialized = True
                except Exception:
                    tracker_initialized = False
        else:
            if tracker_initialized and tracker is not None:
                ok, bbox = tracker.update(cropped)
                if ok:
                    x, y, w, h = bbox
                    center = (int(x + w/2), int(y + h/2))
                    radius = int(max(w,h)/2)
                else:
                    tracker_initialized = False

        proc_times.append(time.perf_counter() - t0)

        if center is not None:
            meas = np.array([[np.float32(center[0])],[np.float32(center[1])]])
            kf.correct(meas)
            pred = kf.predict()
            px, py, vx, vy = float(pred[0]), float(pred[1]), float(pred[2]), float(pred[3])

            avg_proc = float(np.mean(proc_times)) if len(proc_times)>0 else 0.02
            avg_serial = float(np.mean(serial_times)) if len(serial_times)>0 else 0.002
            tau = avg_proc + avg_serial + ARDUINO_EXTRA_LATENCY

            x_pred = px + vx * tau
            y_pred = py + vy * tau

            # clamp
            x_pred = max(0.0, min(DESIRED_WIDTH-1, x_pred))
            y_pred = max(0.0, min(DESIRED_HEIGHT-1, y_pred))

            x_centered, y_centered = center_coords_to_normalized((x_pred, y_pred), DESIRED_WIDTH, DESIRED_HEIGHT)

            send_packet(x_centered, y_centered, True)

            # print coordinates every PRINT_EVERY frames
            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] Detected: x={x_centered:+.3f}, y={y_centered:+.3f})")

            # draw markers
            cv2.circle(cropped, (int(px), int(py)), max(2, int(radius)), (0,255,0), 2)
            cv2.circle(cropped, (int(x_pred), int(y_pred)), 4, (255,0,0), -1)
            cv2.putText(cropped, f"{x_centered:+.3f},{y_centered:+.3f}", (8,18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        else:
            send_packet(0.0, 0.0, False)
            # print when ball is lost
            if frame_idx % PRINT_EVERY == 0:
                print(f"[Frame {frame_idx}] Ball NOT detected")

        # show
        cv2.imshow("Cropped", cropped)
        cv2.imshow("Mask", mask)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break

finally:
    camera.StopGrabbing()
    camera.Close()
    if arduino is not None and arduino.is_open:
        arduino.close()
    cv2.destroyAllWindows()
    print("[PY] Exiting")