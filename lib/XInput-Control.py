import pygame
import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial(port='COM4', baudrate=115200, timeout=1)  # Replace 'COM3' with your Arduino port
time.sleep(2)  # Wait for Arduino to initialize

# Initialize pygame
pygame.init()

# Initialize the joystick
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)  # Use the first connected joystick
joystick.init()

print(f"Controller connected: {joystick.get_name()}")

def send_command_to_arduino(command):
    arduino.write(f"{command}\n".encode())
    print(f"Sent to ESP: {command}")

try:
    while True:
        
        # Read ESP serial monitor
        print(arduino.readline())
        
        pygame.event.pump()  # Process input events

        # Read joystick inputs
        axis_left_x = joystick.get_axis(0)  # Left joystick horizontal
        axis_left_y = joystick.get_axis(1)  # Left joystick vertical
        button_a = joystick.get_button(0)   # 'A' button
        button_b = joystick.get_button(1)   # 'B' button

        # Example: Map joystick and button inputs to Arduino commands
        if button_a:
            send_command_to_arduino("A_PRESSED")
        if button_b:
            send_command_to_arduino("B_PRESSED")
        if abs(axis_left_x) > 0.1 or abs(axis_left_y) > 0.1:
            send_command_to_arduino(f"JOYSTICK {axis_left_x:.2f} {axis_left_y:.2f}")

        time.sleep(0.1)  # Small delay to reduce CPU usage
except KeyboardInterrupt:
    print("Exiting...")
finally:
    pygame.joystick.quit()
    arduino.close()
