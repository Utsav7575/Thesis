import cv2
import numpy as np
from collections import deque
import serial
import time

# Camera and Frame Settings
desired_width = 1920   
desired_height = 1080
desired_fps = 60

# Frame processing options
use_native_frame = True  # True = use camera's native size, False = resize frame
target_frame_width = 480   # Only used if use_native_frame = False
target_frame_height = 480  # Only used if use_native_frame = False

# Camera control settings
disable_autofocus = True  # True = disable autofocus, False = keep autofocus on

# Ball detection settings (assuming a bright/white ball on darker background)
# I use HSV here because it's more robust to lighting than pure BGR thresholding
lower_color = np.array([0, 0, 200])    # HSV lower bound for white/bright objects
upper_color = np.array([180, 30, 255]) # HSV upper bound for white/bright objects
min_radius = 10          # Minimum ball radius to detect
min_contour_area = 100   # Minimum contour area to consider as ball
max_lost_frames = 10     # Consider ball lost after this many frames
smoothing_window = 5     # Number of previous positions for smoothing (moving average)

# Serial communication settings
# NOTE: This port is specific to my MacBook setup and may need to be changed on other machines
ARDUINO_PORT = '/dev/cu.usbmodem14201'
BAUD_RATE = 9600
SEND_INTERVAL = 5  # Send data every 5 frames to avoid overwhelming Arduino

# Initialize serial connection
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Give Arduino time to initialize after opening the port
    print(f"✅ Connected to Arduino on {ARDUINO_PORT}")
except serial.SerialException:
    # I treat serial as optional: tracking still runs even if Arduino is not available
    print(f"❌ Failed to connect to Arduino on {ARDUINO_PORT}")
    arduino = None

# Initialize tracking variables
previous_centers = deque(maxlen=smoothing_window)  # Ring buffer for smoothing positions
ball_lost_counter = 0
frame_count = 0

def normalize_coordinates(center, frame_width, frame_height):
    """Convert pixel coordinates to normalized coordinates (0.0 to 1.0)"""
    # I keep this as a simple linear scaling in image coordinates
    x_norm = center[0] / frame_width
    y_norm = center[1] / frame_height
    return x_norm, y_norm

def center_coordinates(center, frame_width, frame_height):
    """Convert pixel coordinates to centered coordinates (-1.0 to 1.0)"""
    # This puts (0,0) in the middle of the frame for easier controller usage
    x_centered = (center[0] - frame_width/2) / (frame_width/2)
    y_centered = (center[1] - frame_height/2) / (frame_height/2)
    return x_centered, y_centered

def smooth_position(center, previous_centers):
    """Apply moving average smoothing to ball position"""
    # I only call this when there is at least one previous point
    if len(previous_centers) > 0:
        previous_centers.append(center)
        # I use a simple mean over the deque; this is cheap and stable enough for my use case
        smooth_center = tuple(map(int, np.mean(previous_centers, axis=0)))
        return smooth_center
    # If no history is available, I just return the raw center
    return center

def send_to_arduino(x_centered, y_centered, ball_detected):
    """Send ball coordinates to Arduino via serial"""
    # I keep the protocol human-readable for easier debugging on the Arduino side
    if arduino and arduino.is_open:
        try:
            # Format: "X:0.123,Y:-0.456,D:1\n" where D is ball detected (1) or lost (0)
            message = f"X:{x_centered:.3f},Y:{y_centered:.3f},D:{1 if ball_detected else 0}\n"
            arduino.write(message.encode())
            
            # Optional: Read response from Arduino (e.g., for debugging or ACKs)
            if arduino.in_waiting > 0:
                response = arduino.readline().decode().strip()
                if response:
                    print(f"Arduino response: {response}")
                    
        except Exception as e:
            # If something goes wrong, I just log it and continue instead of killing the whole app
            print(f"Error sending to Arduino: {e}")

def send_to_controller(center_x, center_y, ball_detected, frame_count):
    """Send coordinates to controller - modify this for the specific controller logic"""
    # I only print to console every 30 frames to avoid spamming (roughly 1 Hz at 30 FPS)
    if frame_count % 30 == 0:
        if ball_detected:
            print(f"Frame {frame_count:4d} | Controller: X={center_x:6.3f}, Y={center_y:6.3f} | Ball: DETECTED")
        else:
            print(f"Frame {frame_count:4d} | Controller: X={0.000:6.3f}, Y={0.000:6.3f} | Ball: LOST")
    
    # I send data to Arduino more frequently than I print, but still not every frame
    if frame_count % SEND_INTERVAL == 0:
        send_to_arduino(center_x, center_y, ball_detected)

# Open Logitech camera (index 0 assumes it's the default camera)
cap = cv2.VideoCapture(0)

# I request high resolution and FPS; actual values may differ depending on camera/USB limits
cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
cap.set(cv2.CAP_PROP_FPS, desired_fps)

# Set autofocus based on setting (not all cameras/drivers respect this flag)
if disable_autofocus:
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # 0 = disable
else:
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # 1 = enable

# Get and display actual camera settings to verify what the driver really applied
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print("="*60)
print("BALL TRACKING SYSTEM INITIALIZED - LOGITECH CAMERA")
print("="*60)
print(f"Requested Camera: {desired_width}x{desired_height} @ {desired_fps} FPS")
print(f"Actual Camera: {width}x{height} @ {fps} FPS")
print(f"Frame Processing: {'Native' if use_native_frame else f'Resized to {target_frame_width}x{target_frame_height}'}")
print(f"Autofocus: {'Disabled' if disable_autofocus else 'Enabled'}")
print(f"Min Ball Radius: {min_radius}px | Min Area: {min_contour_area}px²")
print(f"Smoothing Window: {smoothing_window} frames")
print(f"Arduino Communication: {'✅ Connected' if arduino else '❌ Not Connected'}")
print("="*60)
print("Press 'Q', 'q', or Escape to quit")
print("="*60)

while True:
    ret, frame = cap.read()
    if not ret:
        # If I can't grab a frame, I break out instead of looping forever
        print("Error: Could not read frame from camera")
        break
    
    frame_count += 1

    # Optionally downscale the frame for faster processing (if use_native_frame is False)
    if not use_native_frame:
        frame = cv2.resize(frame, (target_frame_width, target_frame_height))
    
    # Get current frame dimensions (this handles both native and resized cases)
    frame_height, frame_width = frame.shape[:2]

    # Convert to HSV color space to perform robust color thresholding
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask for the ball color and clean it with morphological operations
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the binary mask
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables for this frame
    ball_detected = False
    center = None
    radius = 0

    # Process contours if any are found
    if contours:
        # I take the largest contour, assuming it's the ball
        largest = max(contours, key=cv2.contourArea)
        contour_area = cv2.contourArea(largest)
        
        if contour_area > min_contour_area:
            ((x, y), radius) = cv2.minEnclosingCircle(largest)
            M = cv2.moments(largest)
            
            if M["m00"] > 0 and radius > min_radius:
                # Calculate center using moments (more accurate than circle center alone)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                # Apply smoothing based on previous positions
                if len(previous_centers) > 0:
                    center = smooth_position(center, previous_centers)
                else:
                    # First detection: just add the raw center as starting point
                    previous_centers.append(center)
                
                ball_detected = True
                ball_lost_counter = 0  # Reset lost counter if we see the ball
                
                # Draw detection visualization: outer circle + center point
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)  # Green circle
                cv2.circle(frame, center, 5, (0, 0, 255), -1)  # Red center dot
                
                # Calculate normalized and centered coordinates for controller/Arduino
                x_norm, y_norm = normalize_coordinates(center, frame_width, frame_height)
                x_centered, y_centered = center_coordinates(center, frame_width, frame_height)
                
                # Display tracking information on the frame
                cv2.putText(frame, f"Ball: {center}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(frame, f"Normalized: ({x_norm:.3f}, {y_norm:.3f})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(frame, f"Centered: ({x_centered:.3f}, {y_centered:.3f})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(frame, f"Radius: {radius:.1f} | Area: {contour_area:.0f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                
                # Arduino status indicator on the frame (gives me quick feedback if serial is alive)
                arduino_status = "Arduino: ✅" if arduino and arduino.is_open else "Arduino: ❌"
                cv2.putText(frame, arduino_status, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0) if arduino else (0,0,255), 2)
                
                # Send coordinates to controller (using centered coordinates for symmetry around zero)
                send_to_controller(x_centered, y_centered, True, frame_count)

    # Handle ball lost scenario if nothing was detected or conditions failed
    if not ball_detected:
        ball_lost_counter += 1
        
        # If I lose the ball for too long, I clear the smoothing history
        if ball_lost_counter > max_lost_frames:
            previous_centers.clear()
            cv2.putText(frame, "BALL LOST", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        else:
            cv2.putText(frame, f"Searching... ({ball_lost_counter}/{max_lost_frames})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Arduino status indicator even when ball is lost
        arduino_status = "Arduino: ✅" if arduino and arduino.is_open else "Arduino: ❌"
        cv2.putText(frame, arduino_status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0) if arduino else (0,0,255), 2)
        
        # When the ball is lost, I send zeros so the controller knows to hold or recentre
        send_to_controller(0.0, 0.0, False, frame_count)

    # Display basic frame info (useful for tuning resolution/FPS)
    cv2.putText(frame, f"Frame: {frame_width}x{frame_height}", (frame_width - 260, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    cv2.putText(frame, f"FPS Target: {desired_fps}", (frame_width - 260, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    # Show the main annotated frame and the mask for debugging the color threshold
    cv2.imshow("Ball Tracking System - Logitech Camera", frame)
    cv2.imshow("Detection Mask", mask)

    # Break on 'q' or 'Q' or Escape key
    key = cv2.waitKey(1) & 0xFF
    if key in [ord('q'), ord('Q'), 27]:  # 27 is the ASCII code for Escape
        break

# Cleanup
cap.release()
if arduino and arduino.is_open:
    arduino.close()
    print("Arduino connection closed")
cv2.destroyAllWindows()
print("\nBall tracking system stopped.")
print(f"Total frames processed: {frame_count}")