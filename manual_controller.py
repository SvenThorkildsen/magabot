import pygame
import serial
from time import sleep

'''
NOTE:
The controller works for forward/backward movement and turning in place.
It struggles with combining speed and turning when both are pushed simultaneously.

This program uses left joystick for forward/backward movement, and right joystick for turning.
'''

# Constants
MAX_VELOCITY = 20  # Max joystick-driven velocity
MIN_ACTUATION_VELOCITY = 3  # Minimum velocity to actuate motors
MOTOR_BYTE = 0x86
DELAY_BETWEEN_COMMANDS = 0.08  # Delay between sending each command (in seconds)

# Serial communication setup for Raspberry Pi to Arduino
ser = serial.Serial('/dev/ttyS0', 9600)

# Initialize pygame
pygame.init()
pygame.joystick.init()

def send_message_to_arduino(message):
    try:
        # Send message to Arduino over Serial
        ser.write(message)
        print(f"Message sent to Arduino: {list(message)}")
        sleep(DELAY_BETWEEN_COMMANDS)  # Ensure Arduino has time to process the message
    except serial.SerialException as e:
        print(f"Error sending message to Arduino: {e}")

def convert_to_velocity(axis_value):
    # Map joystick axis input (-1.0 to 1.0) to motor velocity
    velocity = int(axis_value * MAX_VELOCITY)
    
    # Apply the minimum actuation velocity if above 0
    if abs(velocity) > 0:
        velocity = max(MIN_ACTUATION_VELOCITY, abs(velocity))  # Make sure velocity is at least the minimum actuation velocity
    
    # Ensure that the velocity keeps the direction
    if axis_value < 0:
        velocity = -velocity
		
    return velocity

def convert_to_bytes(value, direction):
    if value > 0:
        direction = 0  # Forward
    elif value < 0:
        direction = 1  # Backward
        value = -value  # Convert to positive for byte value

    # Ensure the value fits in a byte (0-255)
    value = max(0, min(value, 255))  # Clamp value between 0 and 255
    
    return value, direction

def send_stop_signal():
    """Sends a stop signal to the robot."""
    byte_array = bytes([MOTOR_BYTE, 0, 0, 0, 0])
    send_message_to_arduino(byte_array)

def send_controller_state_to_arduino(joystick):
    # Get the axis values (left-right, forward-backward)
    axis_turn = joystick.get_axis(3)  # Right joystick, Axis 3 for turning (fix the right joystick mapping)
    axis_move = joystick.get_axis(1)  # Left joystick, Axis 1 for forward/backward movement

    # Invert the y-axis to match the expected forward/backward control
    velocity = convert_to_velocity(-axis_move)  # Positive y moves forward, negative y moves backward
    turn_velocity = convert_to_velocity(axis_turn)  # Positive x turns right

    # Detect if the controller is idle (both joysticks near center)
    if abs(axis_turn) < 0.1 and abs(axis_move) < 0.1:
        send_stop_signal()  # Send stop signal if both axes are near zero
        return

    # If forward/backward movement is applied (left joystick)
    if abs(axis_move) > 0.1:
        if axis_turn > 0:  # Turning right
            left_wheel_speed = velocity  # Left wheel goes faster
            right_wheel_speed = velocity - turn_velocity  # Right wheel goes slower
        elif axis_turn < 0:  # Turning left
            left_wheel_speed = velocity + turn_velocity  # Left wheel goes slower
            right_wheel_speed = velocity  # Right wheel goes faster
        else:  # No turning, move straight
            left_wheel_speed = right_wheel_speed = velocity

        # Determine direction for forward/backward
        left_wheel_speed, left_wheel_dir = convert_to_bytes(left_wheel_speed, 0 if axis_move > 0 else 1)
        right_wheel_speed, right_wheel_dir = convert_to_bytes(right_wheel_speed, 0 if axis_move > 0 else 1)

    # If no forward/backward movement, turn in place (right joystick only)
    else:
        if axis_turn > 0:  # Turn right (left wheel forward, right wheel backward)
            left_wheel_speed = turn_velocity  # Left wheel moves forward
            right_wheel_speed = turn_velocity  # Right wheel moves backward
            left_wheel_dir = 0  # Forward
            right_wheel_dir = 1  # Backward
        elif axis_turn < 0:  # Turn left (left wheel backward, right wheel forward)
            left_wheel_speed = -turn_velocity  # Left wheel moves backward
            right_wheel_speed = -turn_velocity  # Right wheel moves forward
            left_wheel_dir = 1  # Backward
            right_wheel_dir = 0  # Forward

    # Ensure that wheel speeds are valid byte values before sending
    left_wheel_speed = max(0, min(left_wheel_speed, 255))
    right_wheel_speed = max(0, min(right_wheel_speed, 255))

    byte_array = bytes([MOTOR_BYTE, left_wheel_speed, left_wheel_dir, right_wheel_speed, right_wheel_dir])
    send_message_to_arduino(byte_array)

if __name__ == "__main__":
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        pygame.event.pump()  # Update internal states
        send_controller_state_to_arduino(joystick)
        sleep(0.16)  # 60 FPS
