import cv2
from PIL import Image
from util import get_limits
import serial
import time
import subprocess
import sys

try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    # Your code here that uses the serial connection

except serial.SerialException as e:
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)


def arduino(number):
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


yellow = [0, 255, 255]
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)


try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lowerLimit, upperLimit = get_limits(color=yellow)

        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

        mask_ = Image.fromarray(mask)

        bbox = mask_.getbbox()

        if bbox is not None:
            x1, y1, x2, y2 = bbox

            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

            xc = (x1 + x2) / 2
              
            if(xc - 400) > 200:
                arduino(-10)
            elif(xc-400) < -200:
                arduino(10)
            elif(xc - 400) > 100:
                arduino(-5)
            elif(xc - 400) < -100:
                arduino(5)
            elif(xc - 400) > 50:
                arduino(-3)
            elif(xc - 400) < -50:
                arduino(3)
            elif(xc - 400) > 25:
                arduino(-2)
            elif (xc - 400) < -25:
                arduino(2)
            else:
                print("0")
        
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    ser.close()  # Close the serial port
