import pygame
import serial
from time import sleep

'''
NOTE: There is something wrong witht the left/right turning of the joystick and arrows.
'''

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

def send_message_to_arduino(message):
    try:
        # Send message to Arduino over Serial
        ser.write(message)
        print(f"Message sent successfully to Arduino: {list(message)}")
        sleep(DELAY_BETWEEN_COMMANDS)  # Ensure Arduino has time to process
    except serial.SerialException as e:
        print(f"Error sending message to Arduino: {e}")

def convert_to_velocity(axis_value):
    # Map joystick axis input (-1.0 to 1.0) to motor velocity
    velocity = int(axis_value * MAX_VELOCITY)
    
    # Apply the minimum actuation velocity if above 0
    if abs(velocity) > 0:
        velocity = max(MIN_ACTUATION_VELOCITY, abs(velocity)) * (1 if velocity > 0 else -1)
    
    return velocity

def convert_to_bytes(value, dir):
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

    value = value & 0xFF
    return value

def send_controller_state_to_arduino(joystick):
    # Get the axis values (left-right, forward-backward)
    axis_x = joystick.get_axis(0)  # Axis 0 for horizontal movement
    axis_y = joystick.get_axis(1)  # Axis 1 for vertical movement

    # Invert the y-axis to match the expected forward/backward control
    velocity = convert_to_velocity(-axis_y)  # Positive y moves forward
    turn_velocity = convert_to_velocity(axis_x)  # Positive x turns right

    # Stop if joystick is near the center (no movement)
    if abs(axis_x) < 0.1 and abs(axis_y) < 0.1:
        byte_array = bytes([MOTOR_BYTE, 0, 0, 0, 0])
    else:
        # Forward/Backward + Turn logic
        if axis_y < 0:  # Forward (Inverted axis, so -y is forward)
            if axis_x > 0:  # Forward Right
                vel1 = velocity
                vel2 = velocity - turn_velocity  # Reduce right wheel speed
            elif axis_x < 0:  # Forward Left
                vel1 = velocity + turn_velocity  # Reduce left wheel speed
                vel2 = velocity
            else:  # Straight Forward
                vel1 = velocity
                vel2 = velocity

            dir1 = 0  # Forward left wheel
            dir2 = 0  # Forward right wheel

        elif axis_y > 0:  # Backward
            if axis_x > 0:  # Backward Right
                vel1 = velocity
                vel2 = velocity + turn_velocity  # Reduce right wheel speed
            elif axis_x < 0:  # Backward Left
                vel1 = velocity + turn_velocity  # Reduce left wheel speed
                vel2 = velocity
            else:  # Straight Backward
                vel1 = velocity
                vel2 = velocity

            dir1 = 1  # Backward left wheel
            dir2 = 1  # Backward right wheel

        elif axis_x > 0:  # Turn Right (left wheel forward, right wheel backward)
            vel1 = turn_velocity
            vel2 = -turn_velocity
            dir1 = 0  # Left wheel forward
            dir2 = 1  # Right wheel backward

        elif axis_x < 0:  # Turn Left (right wheel forward, left wheel backward)
            vel1 = -turn_velocity
            vel2 = turn_velocity
            dir1 = 1  # Left wheel backward
            dir2 = 0  # Right wheel forward

        # Convert velocities to bytes and send to Arduino
        vel1 = convert_to_bytes(vel1, 1)
        vel2 = convert_to_bytes(vel2, 2)
        byte_array = bytes([MOTOR_BYTE, vel1, dir1, vel2, dir2])

    send_message_to_arduino(byte_array)

if __name__ == "__main__":
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        pygame.event.pump()  # Update internal states
        send_controller_state_to_arduino(joystick)
        sleep(0.16)  # 60 FPS
