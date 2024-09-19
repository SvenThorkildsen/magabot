from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
import threading
import atexit  # To handle program termination

# Constants and settings
MOTOR_BYTE = 0x86
MAX_VELOCITY = 10
MIN_VELOCITY = 3
REVERSE_VELOCITY = 5
center_margin = 30

# PID controller parameters
Kp = 0.05
Ki = 0.005
Kd = 0.01

# Serial communication setup for Raspberry Pi to Arduino
ser = serial.Serial('/dev/ttyS0', 9600)

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
        ser.write(message)
        print(f"Message sent successfully to Arduino: {list(message)}")
        time.sleep(0.05)  # Ensure Arduino has time to process the message
    except serial.SerialException as e:
        print(f"Error sending message to Arduino: {e}")

def convert_to_bytes(value, direction):
    if value > 0:
        direction = 0  # Forward
    elif value < 0:
        direction = 1  # Backward
        value = -value

    # Ensure the value is within the byte range (0-255)
    return value & 0xFF, direction

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

        vel1, dir1 = convert_to_bytes(int(left_wheel_speed), dir1)
        vel2, dir2 = convert_to_bytes(int(right_wheel_speed), dir2)

    byte_array = bytes([MOTOR_BYTE, vel1, dir1, vel2, dir2])
    send_message_to_arduino(byte_array)

def stop_motors():
    """Sends the stop command to the motors."""
    byte_array = bytes([MOTOR_BYTE, 0x00, 0x00, 0x00, 0x00])
    send_message_to_arduino(byte_array)
    print("Motors stopped.")

def map_error_to_velocity(error, max_error):
    normalized_error = abs(error) / max_error
    forward_velocity = MAX_VELOCITY - (MAX_VELOCITY - MIN_VELOCITY) * normalized_error
    rotation_velocity = (MAX_VELOCITY - MIN_VELOCITY) * normalized_error
    forward_velocity = int(max(MIN_VELOCITY, min(forward_velocity, MAX_VELOCITY)))
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
    global prev_error, integral, previous_time, last_target_time, frame

    alpha = 0.1
    filtered_error = 0

    # Register the stop_motors function to be called on program exit
    atexit.register(stop_motors)

    # Wait for the first frame to be captured
    while frame is None:
        time.sleep(0.1)

    while True:
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

        cv2.imshow('Object Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
