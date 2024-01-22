import cv2
import numpy as np #Numerical Operations Library 
from PIL import Image
import serial
import time

# Function to get HSV limits for a given color
def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

# Function to communicate with Arduino
def arduino_communication(number):
    try:
        # Open the serial port if it's not already open
        if not ser.is_open:
            ser.open()

        # Write the data to the serial port
        ser.write(str(number).encode())
    except Exception as e:
        print(f"Error writing to serial port: {e}")
    print(number)
    time.sleep(1)

# Color for object tracking
yellow = [0, 255, 255]

# Initialize Raspberry Pi camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)

# Serial port setup for Arduino communication
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
except serial.SerialException as e:
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

try:
    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Convert frame to HSV
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get HSV limits for the specified color (yellow)
        lowerLimit, upperLimit = get_limits(color=yellow)

        # Create a mask based on color detection
        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

        # Convert mask to PIL Image to find bounding box
        mask_ = Image.fromarray(mask)
        bbox = mask_.getbbox()

        if bbox is not None:
            # Extract bounding box coordinates
            x1, y1, x2, y2 = bbox

            # Draw rectangle around the detected object
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

            # Calculate the center position of the object
            xc = (x1 + x2) // 2

            # Adjust Arduino based on object position
            if xc - 400 > 200:
                arduino_communication(-10)
            elif xc - 400 < -200:
                arduino_communication(10)
            elif xc - 400 > 100:
                arduino_communication(-5)
            elif xc - 400 < -100:
                arduino_communication(5)
            elif xc - 400 > 50:
                arduino_communication(-3)
            elif xc - 400 < -50:
                arduino_communication(3)
            elif xc - 400 > 25:
                arduino_communication(-2)
            elif xc - 400 < -25:
                arduino_communication(2)
            else:
                print("0")

        # Display the frame
        cv2.imshow('frame', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Release the camera and close the serial port
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
