import cv2
import numpy as np
from pypylon import pylon
from collections import deque
import serial
import time

# Ball detection settings
lower_color = np.array([0, 0, 200])  # Lower bound for off-white (light)
upper_color = np.array([180, 30, 255])  # Upper bound for off-white (bright)
min_radius = 10  # Minimum radius of the ball
max_radius = 30# Maximum radius of the ball (adjust this as needed)
min_contour_area = 100
max_lost_frames = 10
smoothing_window = 5

# Serial communication settings
ARDUINO_PORT = '/dev/cu.usbmodem14201'  # MacBook Arduino port
BAUD_RATE = 9600
SEND_INTERVAL = 5  # Send data every 5 frames to avoid overwhelming Arduino

# Initialize serial connection
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Give Arduino time to initialize
    print(f"Connected to Arduino on {ARDUINO_PORT}")
except serial.SerialException:
    print(f"Failed to connect to Arduino on {ARDUINO_PORT}")
    arduino = None

# Initialize tracking variables
previous_centers = deque(maxlen=smoothing_window)
ball_lost_counter = 0
frame_count = 0

def normalize_coordinates(center, frame_width, frame_height):
    """Convert pixel coordinates to normalized coordinates (0.0 to 1.0)"""
    x_norm = center[0] / frame_width
    y_norm = center[1] / frame_height
    return x_norm, y_norm

def center_coordinates(center, frame_width, frame_height):
    """Convert pixel coordinates to centered coordinates (-1.0 to 1.0)"""
    x_centered = (center[0] - frame_width / 2) / (frame_width / 2)
    y_centered = (center[1] - frame_height / 2) / (frame_height / 2)
    return x_centered, y_centered

def smooth_position(center, previous_centers):
    """Apply moving average smoothing to ball position"""
    if len(previous_centers) > 0:
        previous_centers.append(center)
        smooth_center = tuple(map(int, np.mean(previous_centers, axis=0)))
        return smooth_center
    return center

def send_to_arduino(x_centered, y_centered, ball_detected):
    """Send ball coordinates to Arduino via serial"""
    if arduino and arduino.is_open:
        try:
            # Format: "X:0.123,Y:-0.456,D:1\n" where D is ball detected (1) or lost (0)
            message = f"X:{x_centered:.3f},Y:{y_centered:.3f},D:{1 if ball_detected else 0}\n"
            arduino.write(message.encode())
            
            # Optional: Read response from Arduino
            if arduino.in_waiting > 0:
                response = arduino.readline().decode().strip()
                if response:
                    print(f"Arduino response: {response}")
                    
        except Exception as e:
            print(f"Error sending to Arduino: {e}")

def send_to_controller(center_x, center_y, ball_detected, frame_count):
    """Display tracking info and send to Arduino"""
    if frame_count % 30 == 0:  # Print every 30 frames
        if ball_detected:
            print(f"Frame {frame_count:4d} | Controller: X={center_x:6.3f}, Y={center_y:6.3f} | Ball: DETECTED")
        else:
            print(f"Frame {frame_count:4d} | Controller: X={0.000:6.3f}, Y={0.000:6.3f} | Ball: LOST")
    
    # Send to Arduino every SEND_INTERVAL frames
    if frame_count % SEND_INTERVAL == 0:
        send_to_arduino(center_x, center_y, ball_detected)

# Initialize Basler camera
tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
camera.Open()

# Set the camera resolution to the desired values (within supported range)
camera.Width.SetValue(1288)  # Maximum width supported by your camera
camera.Height.SetValue(720)  # Set an appropriate height (within the camera's range)

# Start grabbing frames with the full resolution
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Get the original frame dimensions
frame_width = camera.Width.Value
frame_height = camera.Height.Value

print(f"Camera initialized with resolution: {frame_width}x{frame_height}")
print("Press 'Q', 'q', or Escape to quit")

# Define desired frame size (this is the cropped size you want)
desired_width = 580
desired_height = 580

while camera.IsGrabbing():
    grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grab_result.GrabSucceeded():
        image = converter.Convert(grab_result)
        frame = image.GetArray()
        frame_count += 1

        # Calculate cropping coordinates for the center
        center_x, center_y = frame_width // 2, frame_height // 2

        # Ensure the cropping window fits inside the frame
        crop_x1 = max(center_x - desired_width // 2, 0)
        crop_x2 = min(center_x + desired_width // 2, frame_width)
        crop_y1 = max(center_y - desired_height // 2, 0)
        crop_y2 = min(center_y + desired_height // 2, frame_height)

        # Crop the center region
        cropped_frame = frame[crop_y1:crop_y2, crop_x1:crop_x2]

        # Optionally resize the cropped frame if needed (e.g., if it’s slightly smaller than desired size)
        if cropped_frame.shape[0] != desired_height or cropped_frame.shape[1] != desired_width:
            cropped_frame = cv2.resize(cropped_frame, (desired_width, desired_height))

        # Convert to HSV color space
        hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_detected = False
        center = None
        radius = 0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(largest)

            if contour_area > min_contour_area:
                ((x, y), radius) = cv2.minEnclosingCircle(largest)
                M = cv2.moments(largest)

                if M["m00"] > 0 and min_radius <= radius <= max_radius:  # Check both min and max radius
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    if len(previous_centers) > 0:
                        center = smooth_position(center, previous_centers)
                    else:
                        previous_centers.append(center)

                    ball_detected = True
                    ball_lost_counter = 0

                    # Draw the ball on the frame
                    cv2.circle(cropped_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.circle(cropped_frame, center, 5, (0, 0, 255), -1)

                    # Calculate normalized and centered coordinates
                    x_norm, y_norm = normalize_coordinates(center, desired_width, desired_height)
                    x_centered, y_centered = center_coordinates(center, desired_width, desired_height)

                    send_to_controller(x_centered, y_centered, True, frame_count)

        if not ball_detected:
            ball_lost_counter += 1
            if ball_lost_counter > max_lost_frames:
                previous_centers.clear()
            send_to_controller(0.0, 0.0, False, frame_count)

        # Show the cropped frame and the mask
        cv2.imshow("Cropped Frame", cropped_frame)
        cv2.imshow("Detection Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('q'), ord('Q'), 27]:
            break

        grab_result.Release()

# Cleanup
camera.StopGrabbing()
camera.Close()
if arduino and arduino.is_open:
    arduino.close()
    print("Arduino connection closed")
cv2.destroyAllWindows()
print("\nBall tracking system stopped.")
print(f"Total frames processed: {frame_count}")
