import atexit  # To handle program termination
import cv2
import numpy as np
import pygame
import serial
import threading
import time
from picamera2 import Picamera2
from time import sleep

# Constants
MAX_VELOCITY = 20  # Max joystick-driven velocity
MIN_ACTUATION_VELOCITY = 3  # Minimum velocity to actuate motors
MOTOR_BYTE = 0x86
dir1 = 0  # Direction for left wheel
dir2 = 0  # Direction for right wheel
DELAY_BETWEEN_COMMANDS = 0.08  # 50ms delay between sending each command

# Serial communication setup for Raspberry Pi to Arduino
ser = serial.Serial('/dev/ttyS0', 9600)

# Initialize pygame
pygame.init()
pygame.joystick.init()

# Flag for mode of operation
camera_mode = False

# Camera stuff
# Constants and settings
MOTOR_BYTE = 0x86
# MAX_VELOCITY = 10
# MIN_VELOCITY = 3
REVERSE_VELOCITY = 5
center_margin = 30

# PID controller parameters
Kp = 0.05
Ki = 0.005
Kd = 0.01

# Initialize variables
prev_error = 0
integral = 0
previous_time = time.time()

# Initialize update timer
timestamp = time.time()
last_target_time = time.time()
TARGET_LOSS_TIMEOUT = 1.0

# MobileNet-SSD model paths
MODEL_DIR = 'MobileNet-SSD-master'
deploy_prototxt_path = f'{MODEL_DIR}/deploy.prototxt'
caffemodel_path = f'{MODEL_DIR}/mobilenet_iter_73000.caffemodel'

# Load the pre-trained MobileNetSSD model
net = cv2.dnn.readNetFromCaffe(deploy_prototxt_path, caffemodel_path)

# List of class labels MobileNetSSD can detect
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

TARGET_OBJECT = "person"

# Global variable for the frame
frame = None

def send_message_to_arduino(message):
    try:
        # Send message to Arduino over Serial
        ser.write(message)
        print(f"Message sent successfully to Arduino: {list(message)}")
        sleep(DELAY_BETWEEN_COMMANDS)  # Ensure Arduino has time to process
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

    # Log joystick input and velocities for debugging
    # print(f"Joystick Input -> axis_x: {axis_x}, axis_y: {axis_y}")
    # print(f"Calculated Velocities -> vel1: {velocity}, turn_velocity: {turn_velocity}")

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
    print("Motors stopped.")


def adjust_velocity_and_send(error, max_error, reversing=False):
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

def map_error_to_velocity(error, max_error):
    normalized_error = abs(error) / max_error
    forward_velocity = MAX_VELOCITY - (MAX_VELOCITY - MIN_ACTUATION_VELOCITY) * normalized_error
    rotation_velocity = (MAX_VELOCITY - MIN_ACTUATION_VELOCITY) * normalized_error
    forward_velocity = int(max(MIN_ACTUATION_VELOCITY, min(forward_velocity, MAX_VELOCITY)))
    rotation_velocity = int(max(0, rotation_velocity))
    return forward_velocity, rotation_velocity

def find_object_position(frame):
    # Ensure frame has 3 channels (BGR), not 4 (BGRA)
    if frame.shape[2] == 4:  # Check if the image has 4 channels
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Convert from BGRA to BGR

    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    for i in np.arange(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.4:
            idx = int(detections[0, 0, i, 1])
            label = CLASSES[idx]

            if label == TARGET_OBJECT:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                cX = int((startX + endX) / 2)

                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(frame, f"{label}: {confidence:.2f}", (startX, startY - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                return cX, startX, startY, endX, endY
    return None

def camera_thread():
    global frame
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(main={"size": (3280, 2464)})
    picam2.configure(config)
    picam2.start()

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

threading.Thread(target=camera_thread, daemon=True).start()


def main():
    # Joystick stuff
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Camera stuff
    global prev_error, integral, previous_time, last_target_time, frame, camera_mode
    alpha = 0.1
    filtered_error = 0

    # Register the stop_motors function to be called on program exit
    atexit.register(stop_motors)

    # Wait for the first frame to be captured
    while frame is None:
        time.sleep(0.1)

    while True:
        pygame.event.pump()  # Update internal states
        
        # Check if Y button (typically index 3) is pressed
        if joystick.get_button(3):
            camera_mode = True
        elif joystick.get_button(1):
            camera_mode = False

        if camera_mode:
            print("Camera mode enabled")
            # sleep(1)
            result = find_object_position(frame)

            if result is not None:
                cX, startX, startY, endX, endY = result
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

        else:
            send_controller_state_to_arduino(joystick)
            sleep(0.16)  # 60 FPS



if __name__ == "__main__":
    main()