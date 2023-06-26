import cv2
import numpy as np
from picamera2 import Picamera2
import serial
from time import sleep
import RPi.GPIO as GPIO

def write(value): 
    ser.write((str(value)).encode('utf-8'))
    print("signal sent", value)

def displayROI(img, ROI1, ROI2):
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)
    
if __name__ == '__main__':
    
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 60
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) 
    ser.flush()
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # order: x1, y1, x2, y2
    ROI1 = [40, 200, 180, 275]
    ROI2 = [520, 170, 640, 255]
    
    kp = 0.0131
    kd = 0
    
    straightConst = 98 #angle in which car goes straight
    
    angle = 2098
    sharpRight = 2068
    sharpLeft = 2128
  
    speed = 1424
    
    turns = 0
    
    aDiff = 0
    prevDiff = 0

    sleep(8)

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

        for cnt in contours_left:
            area = cv2.contourArea(cnt)
            
            leftArea = max(area, leftArea) 
        
        for cnt in contours_right:
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
            
        write(angle) 
        
        if cv2.waitKey(1)==ord('q'):     
            write(1500) 
            write(2098)
            break
        
    cv2.destroyAllWindows()
