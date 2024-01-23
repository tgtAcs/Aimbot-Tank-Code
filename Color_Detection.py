import cv2 #OpenCV
import numpy as np #Numerical Operations Library 
from PIL import Image #Image processing Library
import serial #Arduino Serial Communication Library
import time #Library needed for function sleep

def get_limits(color): #Set the limits of the color to be detected
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue swap
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


def arduino_communication(number): # Function to communicate with Arduino via serial
    try: #Open Serial port
        if not ser.is_open:
            ser.open()

        ser.write(str(number).encode()) #Write in Serial
    except Exception as e:
        print(f"Error writing to serial port: {e}")
    print(number)
    time.sleep(1) #Wait for arduino to respond


# Color for object tracking
yellow = [0, 255, 255]

# Initialize Raspberry Pi camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)


# Serial port setup for Arduino communication, usually port is ACM0, but to incase ACM1 became the one
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
except serial.SerialException as e:
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

#Main Function
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

            #Different angles to send to arduino to maximize turning speed and accuracy
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
finally:
    # Release the camera and close the serial port
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
