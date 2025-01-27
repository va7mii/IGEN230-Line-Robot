import pygame
import keyboard
import time

# Initialize serial communication with Arduino
#arduino = serial.Serial(port='COM4', baudrate=115200, timeout=1)  # Replace 'COM3' with your Arduino port
#time.sleep(2)  # Wait for Arduino to initialize

# Initialize pygame
pygame.init()

# Initialize the joystick
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)  # Use the first connected joystick
joystick.init()

print(f"Controller connected: {joystick.get_name()}")

try:
    while True:
        
        # Read ESP serial monitor
        #print(arduino.readline())
        
        pygame.event.pump()  # Process input events

        # Read joystick inputs
        #axis_left_x = joystick.get_axis(0)  # Left joystick horizontal
        axis_left_y = joystick.get_axis(1)  # Left joystick vertical
        axis_right_y = joystick.get_axis(3) # Right joystick vertical
        button_a = joystick.get_button(0)   # 'A' button
        button_b = joystick.get_button(1)   # 'B' button
        left_bumper = joystick.get_button(9) # left bumper
        right_bumper = joystick.get_button(10) # right bumper
        

        # Example: Map joystick and button inputs to Arduino commands
        if button_a:
            keyboard.write('m\n') # manual mode
        if button_b:
            keyboard.write('a\n') # auto mode
        if left_bumper:     
            keyboard.write('c\n');   #calibrate shortcut
        if right_bumper: #trigger joystick
            keyboard.write('s' + str(-int(axis_left_y * 20)) + " " + str(-int(axis_right_y * 20)) + '\n')

        time.sleep(0.1)  # Small delay to reduce CPU usage
except KeyboardInterrupt:
    print("Exiting...")
finally:
    pygame.joystick.quit()
    #arduino.close()
