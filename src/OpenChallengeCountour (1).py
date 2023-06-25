import cv2
import numpy as np
from picamera2 import Picamera2
import serial
from time import sleep
import RPi.GPIO as GPIO

#angles: Straight - 98 degrees
ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) 
ser.flush()

color = (0, 255, 255)
thickness = 4

def write(value): 
    ser.write((str(value)).encode('utf-8'))
    print("signal sent", value)

def displayROI(img, ROI1, ROI2):
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[2], ROI1[1]), color, thickness)
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[0], ROI1[3]), color, thickness)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[2], ROI1[1]), color, thickness)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[0], ROI1[3]), color, thickness)
    
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[2], ROI2[1]), color, thickness)
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[0], ROI2[3]), color, thickness)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[2], ROI2[1]), color, thickness)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[0], ROI2[3]), color, thickness)
    
if __name__ == '__main__':
    
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 60
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    
    kp = 0.0131
    kd = 0
    
    straightConst = 98

    # order: x1, y1, x2, y2
    ROI1 = [40, 200, 180, 275]
    ROI2 = [520, 170, 640, 255]

    width = 640
    height = 480
  
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  
    speed = 1424
    
    sharpRight = 2068
    sharpLeft = 2128
    
    turns = 0
    
    aDiff = 0
    prevDiff = 0

    while True:
        if GPIO.input(5) == GPIO.LOW:
            break
 
    write(speed)
    write(angle)

    while True: 

        rightArea, leftArea = 0, 0

        img = picam2.capture_array()
  
        displayROI(img, ROI1, ROI2)
        
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, imgThresh = cv2.threshold(imgGray, 60, 255, cv2.THRESH_BINARY_INV)

        contours_left, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        contours_right, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for i in range(len(contours_left)):
            cnt = contours_left[i]
            area = cv2.contourArea(cnt)
            
            leftArea = max(area, leftArea) 
        
        for i in range(len(contours_right)):
            cnt = contours_right[i]
            area = cv2.contourArea(cnt)

            rightArea = max(area, rightArea)

        print(leftArea, rightArea)
        
        aDiff = rightArea - leftArea
        
        angle = int(straightConst + aDiff * kp + (aDiff - prevDiff) * kd) + 2000
        
        if sharpLeft - angle <= 10:
            angle = sharpLeft
        
        if angle - sharpRight <= 10:
            angle = sharpRight
        
        prevDiff = aDiff
        
        cv2.imshow("finalColor", img)
        
        angle = min(max(angle, sharpRight), sharpLeft)
            
        write(angle)
        
        if cv2.waitKey(1)==ord('q'):      
            write(1500)
            write(2098)
            break
        
    cv2.destroyAllWindows()
