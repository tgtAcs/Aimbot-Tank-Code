import cv2 #Opencv
import time #Sleep function
import serial

#Object Detection
x_Res = 640
y_Res = 200
thres = 0.50

#Set the path to the required files and initialize the frame and object sizes
classNames = []
classFile = "/home/pi/Desktop/Object_Detection_Files/coco.names"
configPath = "/home/pi/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/pi/Desktop/Object_Detection_Files/frozen_inference_graph.pb"
with open(classFile, "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

#Object Detect Function
def getObjects(img, thres, nms, draw=True, objects=[]):
    
    # Detect objects in the image using the pre-trained neural network
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)

    #Don't do anything if nothing is found
    if len(objects) == 0:
        objects = classNames
    objectInfo = []

    #If object are detected
    if len(classIds) != 0:
        #loop though all detected objects
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box, className])
                #If object name is set to visiable, draw a rectangle aroudn the most prosiable one
                if draw:
                    cv2.rectangle(img, box, color=(0, 255, 0), thickness=1)
                    cv2.putText(img, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                    cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
                break  # Stop processing after the first detected object
    return img, objectInfo


#Serial communication with arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
horAngle = 90;
verAngle = 90;
def arduino(number):
    ser.write(str(number).encode())
    

#Main Function
if __name__ == "__main__":
    #Initialization
    cap = cv2.VideoCapture(0)
    cap.set(3, x_Res)
    cap.set(4, y_Res)
    cap.set(10,70)

    object_center_x, object_center_y = 0, 0
    toRight, toDown = 0, 0
    
    while True:
        success, img = cap.read()
        result, objectInfo = getObjects(img, 0.50, 0.5, objects=['mouse']) #Target is a mouse
        center_x, center_y = 0, 0
        cv2.circle(img, (int(x_Res / 2), int(y_Res / 2)), 3, (255, 255, 255), -1) #Draw a white dot on the center as crosshair

        #If detected target
        if objectInfo:
            box = objectInfo[0][0]

            #Find center coordinate of the object and draw a circle around the center
            object_center_x = int((2 * box[0] + box[2]) / 2)
            object_center_y = int((2 * box[1] + box[3]) / 2)
            toRight = object_center_x - x_Res / 2
            toDown  = object_center_y - y_Res / 2
            cv2.circle(img, (object_center_x, object_center_y), 20, (0, 255, 0), 1)

            #adjust servo if center is outside of 60pixels from middle
            if toRight < -30:
                horAngle = horAngle + 3
            elif toRight > 30:
                horAngle = horAngle - 3

        #Print in serial and wait for arduino to respond
        arduino(horAngle)
        time.sleep(0.7)

        print((toRight, toDown, horAngle))
        cv2.imshow("Output", img)
        cv2.waitKey(1)
