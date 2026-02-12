"""
- Basler pylon camera
- MOSSE tracker with periodic full-detection reinit
- Kalman filter for smoothing
- Single display window showing mask + ball + center dot
- FPS & processing stats printed in terminal
"""

import cv2
import numpy as np
from pypylon import pylon
import struct
import serial
import time
from collections import deque
import csv
from datetime import datetime

# ---------- CONFIG ----------
ARDUINO_PORT = 'COM3'       # Arduino port
BAUD_RATE = 115200           # Serial baud rate
DESIRED_WIDTH = 580
DESIRED_HEIGHT = 580
DETECT_EVERY = 18            # full detection every N frames
THRESHOLD_VALUE = 100
MIN_CONTOUR_AREA = 100
MIN_RADIUS = 6
MAX_RADIUS = 120
ARDUINO_EXTRA_LATENCY = 0.02 # extra seconds for Arduino + actuation
PROCESS_TIME_WINDOW = 20     # rolling window for processing time
SERIAL_TIME_WINDOW = 50      # rolling window for serial time
PRINT_EVERY = 60             # print every N frames
# --------------------------------

# ---------- SERIAL INIT ----------
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.001)
    time.sleep(2.0)  # allow Arduino reset
    arduino.flushInput()
    arduino.flushOutput()
    print(f"Serial open {ARDUINO_PORT} @ {BAUD_RATE}")
except Exception as e:
    arduino = None
    print(f"Warning: serial open failed: {e}")

# ---------- CAMERA INIT ----------
tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
if len(devices) == 0:
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
print(f"Camera initialized {frame_w}x{frame_h}")

# ---------- KALMAN FILTER SETUP ----------
kf = cv2.KalmanFilter(4,2)
kf.transitionMatrix = np.array([[1,0,1,0],
                                [0,1,0,1],
                                [0,0,1,0],
                                [0,0,0,1]], dtype=np.float32)
kf.measurementMatrix = np.array([[1,0,0,0],
                                 [0,1,0,0]], dtype=np.float32)
kf.processNoiseCov = np.eye(4, dtype=np.float32)*1e-3
kf.measurementNoiseCov = np.eye(2, dtype=np.float32)*1e-1
kf.errorCovPost = np.eye(4, dtype=np.float32)

# ---------- TRACKER STATE ----------
tracker = None
tracker_initialized = False
frame_idx = 0
proc_times = deque(maxlen=PROCESS_TIME_WINDOW)
serial_times = deque(maxlen=SERIAL_TIME_WINDOW)

# ---------- UTILS ----------
def detect_ball(crop):
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    _, mask = cv2.threshold(blurred, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
    mask = cv2.erode(mask,None,1)
    mask = cv2.dilate(mask,None,2)
    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest)<MIN_CONTOUR_AREA:
        return None, mask
    ((x,y),r) = cv2.minEnclosingCircle(largest)
    M = cv2.moments(largest)
    if M['m00']==0: return None, mask
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    if r<MIN_RADIUS or r>MAX_RADIUS: return None, mask
    return ((cx,cy),int(r)), mask

def center_coords_to_normalized(center,w,h):
    x_c = (center[0]-w/2)/(w/2)
    y_c = (center[1]-h/2)/(h/2)
    return float(x_c), float(y_c)

def send_packet(xc,yc,detected):
    if arduino is None: return
    packet = struct.pack('<ffB', np.float32(xc), np.float32(yc), 1 if detected else 0)
    t0 = time.perf_counter()
    try:
        arduino.write(packet)
    except Exception as e:
        print(f"Serial write error: {e}")
    serial_times.append(time.perf_counter()-t0)


log_file = open("ball_tracker_with_kalman_filter.csv", "w", newline="") # for kalman code
writer = csv.writer(log_file)
writer.writerow(["timestamp", "x_norm", "y_norm", "detected"])

# ---------- MAIN LOOP ----------
fps_start = time.time()
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
    cx,cy = frame_w//2, frame_h//2
    x1,x2 = max(cx-DESIRED_WIDTH//2,0), min(cx+DESIRED_WIDTH//2,frame_w)
    y1,y2 = max(cy-DESIRED_HEIGHT//2,0), min(cy+DESIRED_HEIGHT//2,frame_h)
    cropped = frame[y1:y2,x1:x2]
    if cropped.shape[0]!=DESIRED_HEIGHT or cropped.shape[1]!=DESIRED_WIDTH:
        cropped = cv2.resize(cropped,(DESIRED_WIDTH,DESIRED_HEIGHT))

    center = None
    radius = 0
    mask = np.zeros((DESIRED_HEIGHT,DESIRED_WIDTH),dtype=np.uint8)

    # periodic detection or tracker lost
    if (frame_idx%DETECT_EVERY==0) or (not tracker_initialized):
        det, mask = detect_ball(cropped)
        if det is not None:
            center,radius = det
            kf.statePre = np.array([[center[0]],[center[1]],[0.],[0.]],dtype=np.float32)
            kf.statePost = kf.statePre.copy()
            try:
                tracker = cv2.TrackerMOSSE_create()
                bbox = (center[0]-radius,center[1]-radius,radius*2,radius*2)
                tracker.init(cropped,bbox)
                tracker_initialized=True
            except: 
                tracker_initialized=False
    else:
        if tracker_initialized and tracker is not None:
            ok,bbox = tracker.update(cropped)
            if ok:
                x,y,w,h = bbox
                center = (int(x+w/2),int(y+h/2))
                radius = int(max(w,h)/2)
            else:
                tracker_initialized=False

    proc_times.append(time.perf_counter()-t0)

    # Kalman prediction and send packet
    if center:
        meas = np.array([[np.float32(center[0])],[np.float32(center[1])]])
        kf.correct(meas)
        pred = kf.predict()
        px,py,vx,vy = float(pred[0]),float(pred[1]),float(pred[2]),float(pred[3])

        avg_proc = float(np.mean(proc_times)) if len(proc_times)>0 else 0.02
        avg_serial = float(np.mean(serial_times)) if len(serial_times)>0 else 0.002
        tau = avg_proc + avg_serial + ARDUINO_EXTRA_LATENCY

        x_pred = px + vx*tau
        y_pred = py + vy*tau
        x_pred = max(0,min(DESIRED_WIDTH-1,x_pred))
        y_pred = max(0,min(DESIRED_HEIGHT-1,y_pred))

        x_centered,y_centered = center_coords_to_normalized((x_pred,y_pred),DESIRED_WIDTH,DESIRED_HEIGHT)
        send_packet(x_centered,y_centered,True)

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        writer.writerow([timestamp, x_centered, y_centered, 1])

        # draw ball and center dot
        display = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        cv2.circle(display,center,radius,(0,255,0),2)
        cv2.circle(display,center,3,(0,0,255),-1)
        cv2.putText(display,f"X:{x_centered:+.3f} Y:{y_centered:+.3f}",(10,30),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
    else:
        send_packet(0.0,0.0,False)
        display = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        cv2.putText(display,"Ball NOT detected",(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        writer.writerow([timestamp, 0.0, 0.0, 0])

    # show single window
    cv2.imshow("Camera View",display)
    key = cv2.waitKey(1) & 0xFF
    if key in (ord('q'),27): 
        break

    # print FPS & coordinates
    if frame_idx % PRINT_EVERY==0:
        fps = PRINT_EVERY / (time.time()-fps_start)
        print(f"[Frame {frame_idx}] FPS: {fps:.2f} | "
              f"{'Ball detected' if center else 'Lost'} | "
              f"X:{x_centered if center else 0:+.3f} Y:{y_centered if center else 0:+.3f}")
        fps_start = time.time()

# ---------- CLEANUP ----------
camera.StopGrabbing()
camera.Close()
if arduino is not None and arduino.is_open: 
    arduino.close()
cv2.destroyAllWindows()
print("Exiting")
log_file.close()
