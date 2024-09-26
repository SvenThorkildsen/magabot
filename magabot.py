import atexit
import cv2
import numpy as np
import pygame
import serial
import threading
import time
from picamera2 import Picamera2
from time import sleep

# Constants
MAX_VELOCITY = 20  # Maximal velocity
MIN_ACTUATION_VELOCITY = 3  # Minimum velocity to actuate motors
MOTOR_BYTE = 0x86
dir1 = 0  # Direction for left wheel
dir2 = 0  # Direction for right wheel
DELAY_BETWEEN_COMMANDS = 0.02  # 20ms delay between sending each command to ensure Arduino can process each byte

# Serial communication setup for Raspberry Pi to Arduino
ser = serial.Serial('/dev/ttyS0', 9600)

# Initialize pygame
pygame.init()
pygame.joystick.init()

MOTOR_BYTE = 0x86
REVERSE_VELOCITY = 5
center_margin = 1

# PID controller parameters
Kp = 1.06
Ki = 0.004
Kd = 0.08

# Initialize global variables
prev_error = 0
integral = 0
previous_time = time.time()
last_target_time = time.time()
last_send_time = 0 
TARGET_LOSS_TIMEOUT = 0.5 # Stops Magabot if no person has been detected (seconds)
frame = None
frame_counter = 0  # Frame counter to subsample frames
camera_mode = False

# MobileNet-SSD model paths
MODEL_DIR = 'MobileNet-SSD-master'
deploy_prototxt_path = f'{MODEL_DIR}/deploy.prototxt'
caffemodel_path = f'{MODEL_DIR}/mobilenet_iter_73000.caffemodel'

# Load the pre-trained MobileNetSSD model
net = cv2.dnn.readNetFromCaffe(deploy_prototxt_path, caffemodel_path)

# TEST: Set backend and target for DNN inference
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)  # Use OpenCV's default backend optimized for CPU
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)  # Use the CPU for inference
# net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL_FP16) # Alternative method, not suitable for RPi

# print(cv2.getBuildInformation()) # Verify cv2's build configuration

# List of class labels MobileNetSSD can detect
# CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
#            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
#            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
#            "sofa", "train", "tvmonitor"]

# TARGET_OBJECT = "person"
PERSON_CLASS_ID = 15 # Select the "person" class directly


def send_message_to_arduino(message):
    global last_send_time  # Access the global last send time
    
    current_time = time.time()

    # Check if enough time has passed since the last command
    if current_time - last_send_time >= DELAY_BETWEEN_COMMANDS:
        try:
            # Send message to Arduino over Serial
            ser.write(message)
            print(f"Message sent to Arduino: {list(message)}")
            
            # Update the last send time
            last_send_time = time.time()
            
        except serial.SerialException as e:
            print(f"Error sending message to Arduino: {e}")


def convert_to_velocity(axis_value):
    """ Map joystick axis input (-1.0 to 1.0) to motor velocity """
    velocity = int(axis_value * MAX_VELOCITY)
    
    # Apply the minimum actuation velocity if above 0
    if abs(velocity) > 0:
        velocity = max(MIN_ACTUATION_VELOCITY, abs(velocity)) * (1 if velocity > 0 else -1)
    
    return velocity

def convert_to_bytes(value, dir):
    """ Convert motor velocity and direction to byte form for serial communication """
    global dir1, dir2
    if value > 0:
        if dir == 1:
            dir1 = 0  # Forward for left wheel
        elif dir == 2:
            dir2 = 0  # Forward for right wheel
    elif value < 0:
        if dir == 1:
            dir1 = 1  # Backward for left wheel
        elif dir == 2:
            dir2 = 1  # Backward for right wheel
        value = -value

    value = int(value)  # Convert value to an integer
    value = value & 0xFF  # Ensure value is within byte range
    return value

DEAD_ZONE = 0.1  # Define a small threshold for the joystick dead zone

def apply_dead_zone(value):
    """ Apply dead zone to joystick input to prevent minor unintended movements """
    return 0 if abs(value) < DEAD_ZONE else value

def apply_min_velocity(velocity):
    """ Ensure that the velocity is at least the minimum actuation velocity unless in the stop zone """
    if velocity == 0:
        return 0
    return max(MIN_ACTUATION_VELOCITY, abs(velocity)) * (1 if velocity > 0 else -1)

def send_controller_state_to_arduino(joystick):
    """ Compute and send motor commands based on joystick input """
    # Get the axis values (left-right, forward-backward)
    axis_x = apply_dead_zone(joystick.get_axis(0))  # Horizontal axis (left-right)
    axis_y = apply_dead_zone(joystick.get_axis(1))  # Vertical axis (forward-backward)

    # Velocity based on forward/backward movement
    velocity = apply_min_velocity(convert_to_velocity(axis_y))
    
    # Turning velocity based on left/right movement
    turn_velocity = apply_min_velocity(convert_to_velocity(axis_x))

    # Check for forward/backward movement first (vertical axis)
    if axis_y != 0:
        # Forward or backward movement
        if axis_y < 0:  # Forward
            if axis_x > 0:  # Forward Right (upper-right zone)
                # Smoothly reduce right wheel velocity based on axis_x, making both wheels more similar as axis_x approaches 0
                vel1 = velocity  # Left wheel full speed
                vel2 = velocity - (velocity * abs(axis_x) * 0.5)  # Right wheel slower with smoother scaling
            elif axis_x < 0:  # Forward Left (upper-left zone)
                vel1 = velocity - (velocity * abs(axis_x) * 0.5)  # Left wheel slower with smoother scaling
                vel2 = velocity  # Right wheel full speed
            else:  # Straight Forward
                vel1 = vel2 = velocity  # Both wheels at full forward speed

            dir1 = 0  # Forward left wheel
            dir2 = 0  # Forward right wheel

        elif axis_y > 0:  # Backward
            if axis_x > 0:  # Backward Right (lower-right zone)
                vel1 = velocity  # Left wheel full speed backward
                vel2 = velocity - (velocity * abs(axis_x) * 0.5)  # Right wheel slower with smoother scaling
            elif axis_x < 0:  # Backward Left (lower-left zone)
                vel1 = velocity - (velocity * abs(axis_x) * 0.5)  # Left wheel slower with smoother scaling
                vel2 = velocity  # Right wheel full speed backward
            else:  # Straight Backward
                vel1 = vel2 = velocity  # Both wheels at full backward speed

            dir1 = 1  # Backward left wheel
            dir2 = 1  # Backward right wheel

    # Turning in place (no forward/backward input)
    elif axis_y == 0:
        if axis_x > 0:  # Turn Right in place (smooth turning)
            vel1 = apply_min_velocity(turn_velocity)  # Left wheel forward
            vel2 = -apply_min_velocity(turn_velocity)  # Right wheel backward
            dir1 = 0  # Forward left wheel
            dir2 = 1  # Backward right wheel
        elif axis_x < 0:  # Turn Left in place (smooth turning)
            vel1 = -apply_min_velocity(turn_velocity)  # Left wheel backward
            vel2 = apply_min_velocity(turn_velocity)  # Right wheel forward
            dir1 = 1  # Backward left wheel
            dir2 = 0  # Forward right wheel
        else:
            # No movement (stop)
            vel1 = vel2 = 0
            dir1 = dir2 = 0

    # Convert velocities to bytes and send to Arduino
    vel1 = convert_to_bytes(vel1, 1)
    vel2 = convert_to_bytes(vel2, 2)
    byte_array = bytes([MOTOR_BYTE, vel1, dir1, vel2, dir2])

    send_message_to_arduino(byte_array)


def stop_motors():
    """Sends the stop command to the motors."""
    byte_array = bytes([MOTOR_BYTE, 0x00, 0x00, 0x00, 0x00])
    send_message_to_arduino(byte_array)
    # print("Motors stopped.")


def adjust_velocity_and_send(error, max_error, reversing=False):
    # start_time = time.time()  # Start timer for velocity adjustment
    
    if reversing:
        vel1 = REVERSE_VELOCITY
        vel2 = REVERSE_VELOCITY
        dir1 = dir2 = 1  # Reverse both wheels
    else:
        forward_velocity, rotation_velocity = map_error_to_velocity(error, max_error)
        dir1 = dir2 = 0  # Forward by default

        if abs(error) <= center_margin:
            left_wheel_speed = right_wheel_speed = forward_velocity
        else:
            if error > 0:
                left_wheel_speed = forward_velocity - rotation_velocity
                right_wheel_speed = forward_velocity + rotation_velocity
            elif error < 0:
                left_wheel_speed = forward_velocity + rotation_velocity
                right_wheel_speed = forward_velocity - rotation_velocity

        vel1 = convert_to_bytes(int(left_wheel_speed), dir1)
        vel2 = convert_to_bytes(int(right_wheel_speed), dir2)

    byte_array = bytes([MOTOR_BYTE, vel1, dir1, vel2, dir2])
    send_message_to_arduino(byte_array)
    # end_time = time.time()  # End timer for velocity adjustment
    # print(f"Velocity adjustment and send took {end_time - start_time:.4f} seconds")


def map_error_to_velocity(error, max_error):
    normalized_error = abs(error) / max_error
    forward_velocity = MAX_VELOCITY - (MAX_VELOCITY - MIN_ACTUATION_VELOCITY) * normalized_error
    rotation_velocity = (MAX_VELOCITY - MIN_ACTUATION_VELOCITY) * normalized_error
    forward_velocity = int(max(MIN_ACTUATION_VELOCITY, min(forward_velocity, MAX_VELOCITY)))
    rotation_velocity = int(max(0, rotation_velocity)) * 0.3 # Scaling factor, reduces rotation speed - Added for testing purposes
    return forward_velocity, rotation_velocity


def find_object_position(frame, frame_skip=4):
    global frame_counter

    frame_counter += 1
    if frame_counter % frame_skip != 0: # frame_skip defines how many frames should be skipped before processing a frame
        return None # Skip frame

    # Timer for object detection - Debugging tool for analyzing performance of computer vision model
    overall_start_time = time.time()

    (h, w) = frame.shape[:2]
    frame_aspect_ratio = h / w # 2464 / 3280 based on RPi camera's maximum resolution

    scaling_factor = 0.075  # Scaling factor for reducing size of frame (~0.05 should provide the minimal required size)
    scaled_width = int(w * scaling_factor)  # Scale down the resolution of the frame
    scaled_height = int(scaled_width * frame_aspect_ratio)  # Maintain the ratio
    print("Frame size: (" + str(w) + ", " + str(h) + ")") # Size of captured frame
    print("Scaled frame: (" + str(scaled_width) + ", " + str(scaled_height) + ")") # Size of downscaled frame

    if frame.shape[2] == 4:  # Check if the image has 4 channels
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Convert from BGRA to BGR

    # Scale image and create blob for computer vision model
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (scaled_width, scaled_height), 127.5)
    net.setInput(blob)

 
    detections = net.forward() # Run object detection on the computer vision model

    for i in np.arange(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.4:  # Keep a high confidence threshold
            idx = int(detections[0, 0, i, 1])

            # Only process the "person" class (class id 15)
            if idx == PERSON_CLASS_ID:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])  # Scale back to original frame size
                (startX, startY, endX, endY) = box.astype("int")
                cX = int((startX + endX) / 2)

                overall_end_time = time.time()  # End timing the object detection function
                print(f"Total object detection took {overall_end_time - overall_start_time:.4f} seconds\n")

                # Early exit when object is found
                print("Pos: " + str(cX))
                return cX

    overall_end_time = time.time()  # End timing the object detection function
    print(f"Total object detection took {overall_end_time - overall_start_time:.4f} seconds (no object found)\n")

    return None


def camera_thread():
    global frame
    picam2 = Picamera2()

    # Set resolution of camera (Define FOV)
    config = picam2.create_preview_configuration(main={"size": (3280, 1232)}) # RPi camera max resolution: 3280x2464
    picam2.configure(config)
    picam2.start()

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

# Run camera in a parallel thread
threading.Thread(target=camera_thread, daemon=True).start()


def main():
    # Initialize joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Define variables
    global prev_error, integral, previous_time, last_target_time, frame, camera_mode
    alpha = 0.1
    filtered_error = 0

    # Register the stop_motors function to be called on program exit
    atexit.register(stop_motors)

    # Wait for the first frame to be captured
    while frame is None:
        time.sleep(0.1)

    while True:
        pygame.event.pump()  # Check which buttons/joysticks have been pushed
        
        if joystick.get_button(3): # Y button
            camera_mode = True
            print("CAMERA MODE")
        elif joystick.get_button(1): # B button
            camera_mode = False
            print("JOYSTICK MODE")

        # Magabot follows person
        if camera_mode:
            pos = find_object_position(frame)

            if pos is not None:
                cX = pos
                target_position = frame.shape[1] // 2
                error = target_position - cX
                max_error = target_position

                current_time = time.time()
                delta_time = current_time - previous_time

                if delta_time > 0:
                    derivative = (error - prev_error) / delta_time
                    integral += error * delta_time

                filtered_error = alpha * error + (1 - alpha) * filtered_error
                control = Kp * filtered_error + Ki * integral + Kd * derivative

                prev_error = error
                previous_time = current_time

                adjust_velocity_and_send(filtered_error, max_error)
                last_target_time = time.time()

            else:
                if time.time() - last_target_time > TARGET_LOSS_TIMEOUT:
                    stop_motors()

        # Remote control of Magabot
        else:            
            send_controller_state_to_arduino(joystick)
            sleep(0.016)  # 60 FPS
        
if __name__ == "__main__":
    main()