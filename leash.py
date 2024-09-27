import pygame
import serial
from time import sleep

# Constants
MAX_VELOCITY = 20  # Max joystick-driven velocity
MIN_ACTUATION_VELOCITY = 3  # Minimum velocity to actuate motors
MOTOR_BYTE = 0x86
dir1 = 0  # Direction for left wheel
dir2 = 0  # Direction for right wheel
DELAY_BETWEEN_COMMANDS = 0.02  # 50ms delay between sending each command
FORWARD_MARGIN = 0.1  # Margin for detecting straight forward movement
DEAD_ZONE = 0.15  # Define a threshold for the joystick dead zone to stop the robot

# Serial communication setup for Raspberry Pi to Arduino
ser = serial.Serial('/dev/ttyS0', 9600)

# Initialize pygame
pygame.init()
pygame.joystick.init()

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

def apply_dead_zone(value):
    """ Apply dead zone to joystick input to prevent minor unintended movements """
    return 0 if abs(value) < DEAD_ZONE else value

def apply_min_velocity(velocity):
    """ Ensure that the velocity is at least the minimum actuation velocity unless in the stop zone """
    if velocity == 0:
        return 0
    return max(MIN_ACTUATION_VELOCITY, abs(velocity)) * (1 if velocity > 0 else -1)

def gradual_speed_increase(velocity, axis_y):
    """ Gradually increase the speed of the slower wheel as it approaches the forward zone """
    # Axis value ranges from -1 to 0; closer to 0 means closer to the forward zone
    factor = abs(axis_y)
    return int(MAX_VELOCITY * factor)

def send_controller_state_to_arduino(joystick):
    """ Compute and send motor commands based on joystick input """
    # Get the axis values (left-right, forward-backward)
    axis_x = apply_dead_zone(joystick.get_axis(0))  # Horizontal axis (left-right)
    axis_y = apply_dead_zone(joystick.get_axis(1))  # Vertical axis (forward-backward)

    # If both axes are within the dead zone, stop the robot
    if axis_x == 0 and axis_y == 0:
        stop_motors()
        return

    # Velocity based on forward/backward movement
    velocity = apply_min_velocity(convert_to_velocity(axis_y))
    
    # Turning velocity based on left/right movement
    turn_velocity = apply_min_velocity(convert_to_velocity(axis_x))

    # Log joystick input and velocities for debugging
    print(f"Joystick Input -> axis_x: {axis_x}, axis_y: {axis_y}")
    print(f"Calculated Velocities -> vel1: {velocity}, turn_velocity: {turn_velocity}")

    # First and Second Quadrants: Forward movement
    if axis_y < -FORWARD_MARGIN:  # Forward movement (axis_y negative for forward)
        if axis_x > 0:  # Forward Right (upper-right zone)
            # Left wheel at full speed, right wheel gradually increasing from half to full speed
            vel1 = MAX_VELOCITY  # Left wheel at full speed
            vel2 = gradual_speed_increase(MAX_VELOCITY, axis_y)  # Right wheel gradually increases
            dir1 = 0  # Forward left wheel
            dir2 = 0  # Forward right wheel
        elif axis_x < 0:  # Forward Left (upper-left zone)
            # Right wheel at full speed, left wheel gradually increasing from half to full speed
            vel1 = gradual_speed_increase(MAX_VELOCITY, axis_y)  # Left wheel gradually increases
            vel2 = MAX_VELOCITY  # Right wheel at full speed
            dir1 = 0  # Forward left wheel
            dir2 = 0  # Forward right wheel
        else:  # Straight forward
            vel1 = vel2 = velocity  # Both wheels at full forward speed
            dir1 = dir2 = 0  # Forward both wheels

    # Third and Fourth Quadrants: Constant speed turning in place
    elif axis_y >= 0:  # In the "turning in place" zone (axis_y positive)
        if axis_x > 0:  # Turn Right in place (lower-right zone)
            vel1 = MAX_VELOCITY  # Left wheel forward
            vel2 = -MAX_VELOCITY  # Right wheel backward
            dir1 = 0  # Forward left wheel
            dir2 = 1  # Backward right wheel
        elif axis_x < 0:  # Turn Left in place (lower-left zone)
            vel1 = -MAX_VELOCITY  # Left wheel backward
            vel2 = MAX_VELOCITY  # Right wheel forward
            dir1 = 1  # Backward left wheel
            dir2 = 0  # Forward right wheel
        else:  # No horizontal input, rotate in place with both wheels moving in opposite directions
            vel1 = MAX_VELOCITY  # Left wheel forward
            vel2 = -MAX_VELOCITY  # Right wheel backward
            dir1 = 0  # Forward left wheel
            dir2 = 1  # Backward right wheel

    # Stop condition when no input
    else:
        vel1 = vel2 = 0  # Stop the robot
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

if __name__ == "__main__":
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        pygame.event.pump()  # Update internal states
        send_controller_state_to_arduino(joystick)
        sleep(0.016)  # 60 FPS
