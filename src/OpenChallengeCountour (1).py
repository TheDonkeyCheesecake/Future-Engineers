import cv2
import numpy as np
from picamera2 import Picamera2
import serial
from time import sleep
import RPi.GPIO as GPIO

#angles: Straight - 98 degrees

if __name__ == '__main__':
    
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 60
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    
    kp = 0.0131
    sConst = 98

    # order: x1, y1, x2, y2
    ROI1 = [40, 200, 180, 275]
    ROI2 = [520, 170, 640, 255]
    color = (0, 255, 255)
    thickness = 4

    width = 640
    height = 480
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) 
    ser.flush()
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    angle = 2090
    speed = 1424
    tSpeed = 1400

    rightAngle = 2078
    leftAngle = 2118
    
    sharpRight = 2068
    sharpLeft = 2128
    
    turns = 0
    
    laneThresh = 10000
    largeLaneThresh = 10000
    val = True

    while True:
        if GPIO.input(5) == GPIO.LOW:
            break
    
    
    
    ser.write((str(speed)).encode('utf-8'))
    print("signal sent", speed)
    ser.write((str(angle)).encode('utf-8'))
    print("signal sent", angle)

    while True: 

        rightArea, leftArea = 0, 0

        img = picam2.capture_array()
  
        image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[2], ROI1[1]), color, thickness)
        image = cv2.line(img, (ROI1[0], ROI1[1]), (ROI1[0], ROI1[3]), color, thickness)
        image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[2], ROI1[1]), color, thickness)
        image = cv2.line(img, (ROI1[2], ROI1[3]), (ROI1[0], ROI1[3]), color, thickness)
    
        image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[2], ROI2[1]), color, thickness)
        image = cv2.line(img, (ROI2[0], ROI2[1]), (ROI2[0], ROI2[3]), color, thickness)
        image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[2], ROI2[1]), color, thickness)
        image = cv2.line(img, (ROI2[2], ROI2[3]), (ROI2[0], ROI2[3]), color, thickness)
        
        

        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, imgThresh = cv2.threshold(imgGray, 60, 255, cv2.THRESH_BINARY_INV)

        contours_left, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        contours_right, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for i in range(len(contours_left)):
            cnt = contours_left[i]
            area = cv2.contourArea(cnt)
            
            cv2.drawContours(img, contours_left, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
            
            leftArea = max(area, leftArea) 
        
        for i in range(len(contours_right)):
            cnt = contours_right[i]
            area = cv2.contourArea(cnt)
            
            cv2.drawContours(img, contours_right, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
            
            rightArea = max(area, rightArea)

        print(leftArea, rightArea)
        
        aDifference = rightArea - leftArea
        
        angle = int(sConst + aDifference * kp) + 2000
        
        if sharpLeft - angle <= 10:
            angle = sharpLeft
        
        if angle - sharpRight <= 10:
            angle = sharpRight
        
        #print(count)
        
        cv2.imshow("finalColor", img)
            
        if sharpRight <= angle <= sharpLeft: 
            ser.write((str(angle)).encode('utf-8'))
            print("signal sent", angle)
        
        if cv2.waitKey(1)==ord('q'):      
            ser.write((str(1500)).encode('utf-8'))
            print(1500)
            ser.write((str(2090)).encode('utf-8'))
            print(2090)
            break
        
    cv2.destroyAllWindows()
